from __future__ import annotations

from abc import ABC, abstractmethod, abstractstaticmethod
from concurrent.futures import Future, ThreadPoolExecutor
from threading import Lock
from typing import Any, Callable, Dict, List, Optional, Tuple

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

    Action will implement status checking, and internal cancellation.
    Additional functionality is on the onus for the implementing class.
    """

    def __init__(
        self,
        type: str,
        llm_description: str,
        llm_examples: str,
        reasoning: str,
        parameters: Dict[str, str],
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

        self.type: str = type
        self.llm_description: str = llm_description
        self.llm_examples: str = llm_examples
        self.parameters: Dict[str, str] = parameters
        self.reasoning: str = reasoning

        self.__status_lock = Lock()
        self.__status: int = 0

        self.error: Optional[Exception] = None

        self._return_lock = Lock()
        self._return_result: Any = None

    @abstractmethod
    def _execute(self) -> Any:
        """
        __execute is the implementing class's method for executing the
        action. It is expected that the implementing class will handle
        status checking for cancellation flags during the process.

        It can return any result; this result is saved to the action
        and can be queried with .get_result().
        """
        pass

    def execute(self):
        """
        execute performs the action; handled within the abstract class
        to handle status control during the action.
        """
        with self.__status_lock:
            if self.__status != READY:
                raise ActionNotReadyException(self.type)
            self.__status = EXECUTING

        try:
            result = self._execute()
            self.set_result(result)
        except Exception as e:
            self._set_error(e)
            raise e

        with self.__status_lock:
            if self.__status == EXECUTING:
                self.__status = COMPLETE

    @abstractmethod
    def _cancel(self):
        """
        __cancel is the implementing class's method for cancelling
        its action in progress.
        """
        pass

    def cancel(self):
        """
        cancel will trigger the action to cancel, if possible. It will
        be blocking until the action is cancelled or errors out.
        Internally it will set the status to CANCELLING, and then call
        the implementing class's __cancel method.
        """
        with self.__status_lock:
            if self.__status != EXECUTING:
                raise ActionNotCancellableException(
                    self.type, "Action not currently executing"
                )
            self.__status = CANCELLING

        try:
            self._cancel()
            self.set_status(CANCELLED)
        except Exception as e:
            self._set_error(e)
            raise e

    @abstractstaticmethod
    def Create(reason: str, parameters: List[str]) -> Action:
        """
        Create is passed the reasoning of the action being created
        and the ordered parameters inserted. The implementing class
        must utilize these to initialize and create an Action object
        for use.
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

    def _set_error(self, e: Exception):
        self.error = str(e)
        self.set_status(ERROR)

    def set_result(self, results: Any):
        """
        set_result will thread-safe set the result of the action
        """
        with self._return_lock:
            self._return_result = results

    def get_result(self) -> Any:
        """
        get_result will thread-safe return the result of the action
        """
        with self._return_lock:
            return self._return_result

    def __str__(self) -> str:
        # Format the parameters
        parameters = ""
        for key, value in self.parameters.items():
            if len(parameters) > 0:
                parameters += ", "
            parameters += f"{key}={value}"

        reasoning_string = ""
        if self.reasoning is not None and len(self.reasoning) > 0:
            reasoning_string = f"# {self.reasoning}\n"

        return f"{reasoning_string}{self.type}({parameters})"

    def __eq__(self, __value: Action) -> bool:
        if __value is None:
            return False
        return (
            self.type == __value.type
            and self.parameters == __value.parameters
            and self.reasoning == __value.reasoning
        )


class ActionPlanParser:
    """
    ActionPlanParser is a parsing client that, given a text blurb from
    an LLM, and a set of possible actions we'd expect from the LLM,
    will extract the set of actions, their reasoning, and parameters,
    and finally return a list of Action objects as specified.
    """

    def __init__(self, action_classes: Dict[str, Callable]):
        """
        Creates a new ActionPlanParser object. The action_classes
        is a dict of the action type to the class object itself
        for parsing calls.
        """
        self.action_classes = action_classes

    def parse_action_lines(
        self, lines: Tuple[str, str]
    ) -> Tuple[str, str, Dict[str, str]]:
        """
        parse_action_lines - Given a set of two lines from the LLM for
        its reasoning and action call, extract out the reasoning, action
        name, and list of passed parameters for additional processing.
        """
        # Extract the reasoning and action call
        reasoning = lines[0]
        action_call = lines[1]

        # The reasoning is preceded by "# " and may have additional spacing;
        # clear this up
        reasoning = reasoning.replace("# ", "").strip()

        # Extract the action name
        action_name = action_call.split("(")[0]

        # Extract the parameters
        parameters = action_call.split("(")[1].split(")")[0].split(",")
        parameters = [parameter.strip() for parameter in parameters]

        return (reasoning, action_name, parameters)

    def parse(self, text: str) -> ActionPlan:
        """
        parse will accept a block of text and parse from it a list
        of ordered actions, their reasoning, and parameters. It will
        then return that ActionPlan for each parsed action if
        no issues occur during parsing.
        """
        # First we will remove white space surrounding the text and
        # then we will remove blank lines that may be spaced between
        # actions
        lines = text.strip().split("\n")
        lines = [line.strip() for line in lines if len(line.strip()) > 0]

        # Now that we've isolated, break our lines into groups. The
        # expected output is:
        # # comment for our reasoning
        # action(parameters)
        # We need to deal with the comment line missing as well
        reason_action_combinations: List[Tuple[str, str]] = []

        while len(lines) > 0:
            reason_action: Tuple[str, str] = ("", "")

            # Check to see if the top line is a commented reason
            # or an action. Basically - does it start with "#"?
            reason = ""
            if lines[0].startswith("#"):
                reason = lines.pop(0)

            # The next line should be an action line, regardless
            # if there was a reasoning line or not
            action = lines.pop(0)

            reason_action = (reason, action)
            reason_action_combinations.append(reason_action)

        # Now that we have our reason/action combinations, we can
        # parse them into actions
        actions: List[Action] = []
        for reason_action in reason_action_combinations:
            # Parse the action lines
            reasoning, action_name, parameters = self.parse_action_lines(reason_action)

            # Check to see if we have an action for this action name
            if action_name not in self.action_classes:
                raise UnknownActionException(action_name)

            # Create the action
            action_class = self.action_classes[action_name]
            action = action_class.Create(reasoning, parameters)

            actions.append(action)

        # Create and return our action plan
        return ActionPlan(actions)


class ActionPlan:
    """
    ActionPlan is a tool for managing the execution of a set of
    ordered actions.
    """

    def __init__(self, actions: List[Action]):
        """
        Creates a new action plan instance with the provided set of
        actions, instantiating to a READY status.
        """
        self.actions = actions
        self.__status = READY
        self.__action_index = -1

        self.__status_lock = Lock()
        self.error: Optional[Exception] = None

    def execute(self):
        """
        execute will begin executing the plan, blocking until the
        plan is complete or errors out.
        """
        with self.__status_lock:
            if self.__status != READY:
                raise ActionPlanNotReadyException()
            self.__status = EXECUTING

        try:
            for index, action in enumerate(self.actions):
                self._set_index(index)
                action.execute()
        except Exception as e:
            self.error(e)
            raise e

        with self.__status_lock:
            if self.__status == EXECUTING:
                self.__status = COMPLETE

    def execute_async(self) -> Future:
        """
        execute_async will begin executing the plan, returning
        immediately. The plan will execute in the background, and
        the caller can check the status of the plan to determine
        when it is complete.
        """
        with ThreadPoolExecutor() as executor:
            return executor.submit(self.execute)

    def cancel(self):
        """
        cancel will trigger the plan to cancel, if possible. It will
        be blocking until the plan is cancelled or errors out.
        """
        with self.__status_lock:
            if self.__status != EXECUTING:
                raise ActionPlanNotCancellableException("Plan is not EXECUTING")
            self.__status = CANCELLING

        try:
            for action in self.actions:
                action.cancel()
            self.set_status(CANCELLED)
        except Exception as e:
            self._set_error(e)
            raise e

    def set_status(self, status: int):
        """
        set_status will thread-safe change the status of the plan
        """
        with self.__status_lock:
            self.__status = status

    def get_status(self) -> int:
        """
        get_status will thread-safe return the status of the plan
        """
        with self.__status_lock:
            return self.__status

    def _set_error(self, e: Exception):
        self.error = e
        self.set_status(ERROR)

    def _set_index(self, index: int):
        """
        _set_index sets the targeted index for status tracking
        """
        with self.__status_lock:
            self.__action_index = index

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


class ActionPlanNotReadyException(Exception):
    def __init__(self):
        super().__init__(f"Action Plan is not ready to be executed")


class ActionPlanNotCancellableException(Exception):
    def __init__(self, reason: str):
        super().__init__(f"Action Plan is not executing: {reason}")


class CouldNotParseActionPlanException(Exception):
    def __init__(self, e: Exception):
        super().__init__(f"Could not parse action plan: {e}")


class UnknownActionException(Exception):
    def __init__(self, type: str):
        super().__init__(f"Unknown action type: {type}")
