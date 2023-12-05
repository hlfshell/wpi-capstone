import rclpy
from rclpy.node import Node
from typing import Dict

from threading import Thread

from capstone_interfaces.srv import (
    PickUpObject as PickUpObjectMsg,
    GiveObject as GiveObjectMsg,
)

from planner.action import Action, ActionPlanner
from planner.interaction import InteractionModule
from planner.navigation import NavigationModule
from planner.robot_actions import (
    GiveObject,
    MoveToObject,
    MoveToRoom,
    MoveToHuman,
    PickUpObject,
    DoISeeObject,
)
from planner.state import StateModule
from planner.vision import VisionModule


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

        print("making modules")
        # Modules
        self.interaction_module = InteractionModule()
        self.navigation_module = NavigationModule()
        self.vision_module = VisionModule()
        self.state_module = StateModule()

        # Actions
        print("Creating Actions")
        actions: Dict[str, Action] = {
            "move_to_object": MoveToObject(
                self.navigation_module,
                self.vision_module,
                self.state_module,
            ),
            "move_to_room": MoveToRoom(self.navigation_module, self.state_module),
            "move_to_human": MoveToHuman(self.navigation_module, self.state_module),
            "pickup_object": PickUpObject(self.interaction_module),
            "give_object": GiveObject(self.interaction_module),
            "do_i_see_object": DoISeeObject(self.vision_module),
        }

        print("Creating planner")
        self.__action_planner = ActionPlanner(actions)

    def run(self, code: str):
        self.__action_planner.execute(code)

    def spin(self):
        self.__executor.add_node(self)
        self.__executor.add_node(self.interaction_module)
        self.__executor.add_node(self.navigation_module)
        self.__executor.add_node(self.vision_module)
        self.__executor.add_node(self.state_module)
        Thread(target=self.__executor.spin, daemon=True).start()
