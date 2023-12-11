from __future__ import annotations

import ast
from abc import ABC, abstractmethod
from functools import partial
from threading import Lock
from typing import Any, Callable, Dict, Optional

# Status code constants for actions
READY = 0
EXECUTING = 1
COMPLETE = 2
CANCELLING = 3
CANCELLED = 4
ERROR = 5


class Action(ABC):
    """
    Action is an abstract class for implementing an action interface,
    allowing us to easily create a set of block modules generated off
    of LLM text. The goal is that each action will operate in a pre-
    defined duck-typed manner for easy portability and integration,
    and handling implementation specific details in a single location.

    Actions are designed to be initialized, then cloned and utilized
    at execution time. This is to allow for advanced functionality
    like asynchronous execution and cancellation and error tracking.

    Implementers of the action interface are expected to implement
    the following methods:

    _execute() - the actual execution - parameters are variable based
        on what's needed for the Action
    _cancel() - a method that handles special cancellation logic for
        the action. For instance, if you have a long running network
        call, this would send a signal to kill it. If there is no
        special logic, just have the function pass.
    clone() - a method that returns a copy of the action as necessary

    Actions are expected to, if there's a return, use _set_result to
    set the result of the action. Any exceptions raised by the
    _execute function is saved against the action for future reference.
    """

    def __init__(
        self,
        action_type: str,
    ):
        """
        Creates a new Action object. The type designates the class of
        action it is (based on the implementing class). The
        llm_description is the description of the action as provided
        to the LLM for an explanation of capabilities for the planner.
        The llm_examples is a block of text providing examples of
        action usage for the LLM for few-shot prompting. reasoning is
        the provided reasoning the planner used to justify utilizing
        the action, and parameters is a dict of the parameters passed
        to the action function by the LLM.
        """
        super().__init__()

        self.action_type: str = action_type

        self.__status_lock = Lock()
        self.__status: int = 0

        self.__result_lock = Lock()
        self.__result: Optional[Any] = None

        self.error: Optional[Exception] = None

    @abstractmethod
    def _execute(self, *args, **kwargs) -> Any:
        """
        _execute is the actual implementation of the action. It is
        on the implementing class to handle the logic of the action
        itself, as well as implement the possibility for cancellation
        within it.
        """
        pass

    def execute(self, *args, **kwargs) -> Any:
        """
        execute performs the action; handled within the abstract class
        to handle status control during the action.
        """
        with self.__status_lock:
            # Check first prior to execution to see if we have been
            # cancelled and should quit out.
            if self.__status == CANCELLING or self.__status == CANCELLED:
                raise CancellationTriggeredException()
            # If we aren't being cancelled but aren't ready, we need
            # to abort the action
            if self.__status != READY:
                raise ActionNotReadyException(self.action_type)
            self.__status = EXECUTING

        try:
            self._execute(*args, **kwargs)

            result: Any = self._get_result()

            with self.__status_lock:
                if self.__status == EXECUTING:
                    self.__status = COMPLETE

            return result
        except Exception as e:
            self.error = e
            raise e

    def _set_result(self, result: Any):
        """
        _set_result is a helper method for the implementing class
        that sets the result of the action.
        """
        with self.__result_lock:
            self.__result = result

    def _get_result(self) -> Any:
        """
        _get_result is a helper method for the implementing class
        that returns the result of the action.
        """
        with self.__result_lock:
            return self.__result

    def _cancel_check(self):
        """
        _cancel_check is a helper method for the implementing class
        that checks whether or not if cancellation has been called
        for. If so, it will raise a CancellationTriggeredException.

        If triggered within the generated code, it will result in
        cancellation of the exec without ending the parent process.
        """
        with self.__status_lock:
            if self.__status == CANCELLING:
                raise CancellationTriggeredException()

    def _is_cancelled(self):
        """
        _is_cancelled is a thread safe method for implementing classes
        to check if the action has been cancelled.
        """
        with self.__status_lock:
            return self.__status == CANCELLING or self.__status == CANCELLED

    @abstractmethod
    def _cancel(self):
        """
        _cancel is the implementing class's method for cancelling
        its action in progress. It is on the implementing class to
        check when convenient for and trigger cancellation.
        """
        pass

    def cancel(self):
        """
        cancel will trigger the action to cancel, if possible. It will
        be blocking until the action is cancelled or errors out.
        Internally it will set the status to CANCELLING, and then call
        the implementing class's _cancel method.
        """
        with self.__status_lock:
            if self.__status != EXECUTING:
                raise ActionNotCancellableException(
                    self.action_type, "Action not currently executing"
                )
            self.__status = CANCELLING

        try:
            self.__cancel()
            self.set_status(CANCELLED)
        except Exception as e:
            self._set_error(e)
            raise e

    @abstractmethod
    def clone(self) -> Action:
        """
        clone returns a copy of the current action, complete with
        configuration. It is on the implementing class to handle
        proper instantiation and cloning.
        """
        pass

    def set_status(self, status: int):
        """
        set_status will thread-safe change the status of the action
        """
        with self.__status_lock:
            self.__status = status

    def get_status(self) -> int:
        """
        get_status will thread-safe return the status of the action
        """
        with self.__status_lock:
            return self.__status

    def __str__(self) -> str:
        # Format the parameters
        # parameters = ""
        # for key, value in self.parameters.items():
        #     if len(parameters) > 0:
        #         parameters += ", "
        #     parameters += f"{key}={value}"

        # return f"{self.action_type}({parameters})"
        return f"{self.action_type}()"

    def __eq__(self, __value: Action) -> bool:
        if __value is None:
            return False
        return self.action_type == __value.action_type


class ActionPlanner:
    """
    ActionPlanner is a tool that accepts a Dict of Actions with
    associated function names as a key. It can then parse pythonic
    code where calls to the specified function names will clone
    and trigger the associated action.

    The ActionPlanner allows easy tracking of the status of the
    plan, allow cancelling it, and handles syntax checking as well.

    A single planner can work with multiple generated pythonic plans.
    """

    def __init__(
        self,
        actions: Dict[str, Action],
        functions: Dict[str, Callable],
        on_call_callback: Optional[Callable] = None,
    ):
        """
        Creates a new action plan instance with the provided set of
        actions, instantiating to a READY status.
        """
        self.actions = actions
        self.functions = functions

        self.__action_lock = Lock()
        self.__current_action: Optional[Action] = None

        self.__on_call_callback: Optional[Callable] = on_call_callback

    def code_check(self, code: str):
        """
        code_check will check the provided code for syntax errors
        that could trigger prior to execution. It checks via ast for
        tokenization errors and finally compiles the code to check for
        additional errors. It is not perfect given that Python is a
        runtime language.
        """
        try:
            ast.parse(code)
            compile(code, "<string>", "exec")
        except Exception as e:
            raise CouldNotParseActionPlanException(e)

    def __action_wrapper(self, function_name: str, *args, **kwargs) -> Callable:
        """
        __action_wrapper generates the action clone for the specified action,
        saves it as the current action, and then executes it, returning the
        resulting outcome.
        """
        if self.__on_call_callback is not None:
            self.__on_call_callback(function_name, *args, **kwargs)

        # First we create a new action of the specific type
        action = self.actions[function_name].clone()

        # Set this as our target action
        with self.__action_lock:
            self.__current_action = action

        # Execute the action
        return action.execute(*args, **kwargs)

    def __generate_lambdas(self) -> Dict[str, Callable]:
        """
        __generate_lambdas creates a set of lambda functions to be passed
        into the code that wraps it with __action_wrapper, where the action
        object is cloned, set as the current object for tracking and
        cancellation, and then executed as called.
        """
        lambdas: Dict[str, Callable] = {}
        for function_name, action in self.actions.items():
            lambdas[function_name] = partial(self.__action_wrapper, function_name)

        return lambdas

    def __wrapped_function(self, function_name: str, *args, **kwargs):
        """
        __wrapped_function is a wrapper function that will call the provided
        function, then call the on_call_callback if provided.
        """
        if self.__on_call_callback is not None:
            self.__on_call_callback(function_name, *args, **kwargs)

        return self.functions[function_name](*args, **kwargs)

    def __wrap_functions(self) -> Dict[str, Callable]:
        """
        __wrap_functions wraps the provided functions with a function that
        will call the provided function, then call the on_call_callback if
        provided.
        """
        wrapped_functions: Dict[str, Callable] = {}
        for function_name, function in self.functions.items():
            wrapped_functions[function_name] = partial(
                self.__wrapped_function, function_name
            )

        return wrapped_functions

    def execute(self, code: str):
        """
        execute will begin executing the plan, blocking until the
        plan is complete or errors out.
        """
        # First we need to check the syntax prior to executing
        self.code_check(code)

        # If we reach this point, the code is fine and can continue and
        # attempt to execute the code as written
        try:
            # Create a series of lambdas that create new Actions
            # when a given action is generated
            globals = self.__generate_lambdas()
            # Expand our wrapped actions with the provided functions
            functions = self.__wrap_functions()
            globals.update(functions)
            locals = {}
            exec(code, globals, locals)

        except CancellationTriggeredException:
            # If the cancellation is triggered then we can simply
            # return at this point - there is no additional work
            # to do.
            return

        except Exception as e:
            # Record the error and raise it
            self.__set_error(e)
            raise e

    def cancel(self):
        """
        cancel will trigger the plan to cancel, if possible. It will
        be blocking until the plan is cancelled or errors out.
        """
        try:
            with self.__action_lock:
                if self.__current_action is None:
                    return
                else:
                    self.__current_action.cancel()
        except Exception as e:
            self.__set_error(e)
            raise e

    def get_current_action(self) -> Optional[Action]:
        """
        get_current_action will return the current action being
        executed, if any.
        """
        with self.__action_lock:
            return self.__current_action

    def __set_error(self, e: Exception):
        self.error = e

    def get_error(self) -> Optional[Exception]:
        with self.__error_lock:
            return self.error

    def __str__(self) -> str:
        result = "Action Plan:\n"
        result += f"Status: {self.get_status()}\n"

        if self.get_status() != READY:
            result += f"Progress: {self.__action_index + 1}/{len(self.actions)}\n"

        for action in self.actions:
            action_string = str(action)
            for line in action_string.split("\n"):
                result += f"  |  {line}\n"

        return result


class ActionNotReadyException(Exception):
    def __init__(self, type: str):
        super().__init__(f"Action {type} is not ready to be executed")


class ActionNotCancellableException(Exception):
    def __init__(self, type: str, reason: str):
        super().__init__(f"Action {type} is not executing: {reason}")


class CouldNotParseActionPlanException(Exception):
    def __init__(self, e: Exception):
        super().__init__(f"Could not parse action plan: {e}")


class CancellationTriggeredException(Exception):
    def __init__(self):
        super().__init__(f"Acton execution was cancelled")
