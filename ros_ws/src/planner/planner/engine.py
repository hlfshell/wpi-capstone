import io
from contextlib import redirect_stdout
from functools import partial
from threading import Lock, Thread
from typing import Callable, Dict, List, Optional, Union

import rclpy
from litterbug.items import Item
from rclpy.node import Node

from capstone_interfaces.msg import RobotAction
from planner.action import Action, ActionPlanner
from planner.interaction import InteractionModule
from planner.navigation import NavigationModule
from planner.omniscience import OmniscienceModule
from planner.robot_actions import (
    DoISee,
    GiveObject,
    LookAround,
    MoveToHuman,
    MoveToObject,
    MoveToRoom,
    PickUpObject,
)
from planner.state import StateModule
from planner.vision import VisionModule

from time import sleep


class RobotEngine(Node):
    """
    RobotEngine is a tool that converts pythonic code to robotic control.

    It utilizes prepared robotic actions in the form of planner.Actions
    and appropriately handles execution and possible cancellation/
    interruption of actions via new information.
    """

    def __init__(self):
        super().__init__("robot_engine")

        self.__executor = rclpy.executors.MultiThreadedExecutor()

        self.__actions_publisher = self.create_publisher(
            RobotAction, "/objective/action", 10
        )

        # Modules
        self.__omniscience_module = OmniscienceModule()
        self.__interaction_module = InteractionModule()
        self.__navigation_module = NavigationModule()
        self.__vision_module = VisionModule()
        self.__state_module = StateModule()

        # Actions
        actions: Dict[str, Action] = {
            "move_to_object": MoveToObject(
                self.__navigation_module,
                self.__vision_module,
                self.__state_module,
                self.__omniscience_module,
            ),
            "move_to_room": MoveToRoom(self.__navigation_module, self.__state_module),
            "move_to_human": MoveToHuman(self.__navigation_module, self.__state_module),
            "pickup_object": PickUpObject(
                self.__interaction_module, self.__state_module
            ),
            "give_object": GiveObject(self.__interaction_module, self.__state_module),
            "do_i_see": DoISee(self.__vision_module, self.__omniscience_module),
            "look_around_for": LookAround(
                self.__navigation_module,
                self.__vision_module,
                self.__omniscience_module,
            ),
        }

        functions: Dict[str, Callable] = {
            "complete": partial(self.__mark_result, True),
            "fail": partial(self.__mark_result, False),
        }

        self.__action_planner = ActionPlanner(
            actions, functions, on_call_callback=self.__on_call
        )

        self.__output_callback: Callable = None

        self.__result_lock = Lock()
        self.__result: Optional[bool] = None

    def run(self, code: str) -> Union[None, bool]:
        # Reset our result
        with self.__result_lock:
            self.__result = None

        out = StdOutRedirect(self.__broadcast)
        with redirect_stdout(out):
            self.__action_planner.execute(code)
            sleep(0.25)

        # Get the result if possible
        with self.__result_lock:
            result = self.__result

        return result

    def set_output_callback(self, callback: Callable):
        self.__output_callback = callback

    def cancel(self):
        self.__action_planner.cancel()

    def spin(self):
        self.__executor.add_node(self)
        self.__executor.add_node(self.__interaction_module)
        self.__executor.add_node(self.__navigation_module)
        self.__executor.add_node(self.__vision_module)
        self.__executor.add_node(self.__state_module)
        # self.__executor.add_node(self.__omniscience_module)
        self.__omniscience_module.spin()
        Thread(target=self.__executor.spin, daemon=True).start()

    def __mark_result(self, result: bool):
        with self.__result_lock:
            self.__result = result

    def __broadcast(self, msg):
        if self.__output_callback is not None:
            self.__output_callback(msg)

    def __on_call(self, function_name: str, *args, **kwargs):
        # Combine args and kwargs into a single string of comma
        # delimited as if it was being passed into a function
        parameters = ", ".join(
            [str(arg) for arg in args] + [f"{k}={v}" for k, v in kwargs.items()]
        )

        self.__actions_publisher.publish(
            RobotAction(
                function=function_name,
                parameters=parameters,
            )
        )


class StdOutRedirect(io.TextIOBase):
    """
    StdOutRedirect will send all writes to stdout
    to the provided callback.
    """

    def __init__(self, target: Callable):
        self.target = target

    def write(self, s: str):
        self.target(s)
