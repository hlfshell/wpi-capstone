from __future__ import annotations

from math import sqrt
from threading import Lock, Thread
from time import sleep
from typing import Dict, List, Tuple, Union

import rclpy
from gazebo_msgs.srv import GetEntityState, GetModelList
from litterbug.items import Item
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OmniscienceModule(Node):
    """
    Omniscience module that stores all information about the world
    items in a way the robot couldn't know to make certain safety
    checks easier for simulation.
    """

    def __init__(self, items: List[Item]):
        super().__init__("omniscience_module")
        self.__executor = rclpy.executors.MultiThreadedExecutor()

        self.__items_lock = Lock()
        # self.__items_by_type is a dictionary of item type to list of items
        # and self.__items_by_id is a dictionary of item id to item
        self.__items: List[Item] = items

        self.__odometry_subscriber = self.create_subscription(
            msg_type=Odometry,
            topic="/odom",
            callback=self.__pose_callback,
            qos_profile=10,  # Keep last
        )
        self.__first_pose_received: bool = False
        self.__pose_lock = Lock()
        self.__position: Tuple[float, float] = (0.0, 0.0)

        self.__get_model_list_client = self.create_client(
            GetModelList, "/get_model_list"
        )
        self.__get_model_list_client.wait_for_service()
        self.__get_entity_state_client = self.create_client(
            GetEntityState, "/gazebo/get_entity_state"
        )
        self.__get_entity_state_client.wait_for_service()

        self.create_timer(1.0 / 5.0, self.__update_items_locations)

    def __get_item(self, item: Union[str, int]) -> List[Item]:
        """
        __get_item returns the item by its name or id
        """
        items: List[Item] = []
        with self.__items_lock:
            for i in self.__items:
                if isinstance(item, str):
                    if i.name == item:
                        items.append(i)
                else:
                    if i.label == item:
                        items.append(i)
        return items

    def am_i_near(self, item: Union[str, int], distance: float) -> bool:
        """
        Check if the robot is near the item.
        """
        robot_position = self.robot_position()
        items = self.__get_item(item)

        for target in items:
            if self.__distance(robot_position, target.origin) < distance:
                return True
        return False

    def robot_position(self) -> Tuple[float, float]:
        """
        robot_position returns the last known position
        of the robot. Will lock until the first odom msg is
        received.
        """
        while True:
            with self.__pose_lock:
                if self.__first_pose_received:
                    break
            sleep(0.1)
        with self.__pose_lock:
            return self.__position

    def __pose_callback(self, msg: Odometry):
        """
        __pose_callback updates our current position of the robot
        """
        with self.__pose_lock:
            self.__first_pose_received = True
            self.__position = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
            )

    def __distance(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """
        __distance returns the distance between two points
        """
        return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def __get_model(
        self, model_name: str
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
        """
        Given a model name, get_model returns the the origin and the
        orientation of the model if it exists
        """
        request = GetEntityState.Request()
        request.name = model_name

        future = self.__get_entity_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response: GetEntityState.Response = future.result()

        if not response.success:
            return None

        return (
            (
                response.state.pose.position.x,
                response.state.pose.position.y,
                response.state.pose.position.z,
            ),
            (
                response.state.pose.orientation.x,
                response.state.pose.orientation.y,
                response.state.pose.orientation.z,
                response.state.pose.orientation.w,
            ),
        )

    def __get_model_list(self) -> List[Item]:
        """
        list_items returns a list of items that exist in the world
        """
        request = GetModelList.Request()
        future = self.__get_model_list_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response: GetModelList.Response = future.result()

        if not response.success:
            self.get_logger().error(f"Could not list models: {response.status_message}")
            return

        items: List[Item] = []
        for model_name in response.model_names:
            result = self.__get_model(model_name)
            if result is None:
                continue
            position, orientation = result

            # Create an item placeholder for the information,
            # even though we're missing label and model
            item = Item(
                name=model_name,
                label="",
                model="",
                origin=position,
                orientation=orientation,
            )
            items.append(item)

        return items

    def __update_items_locations(self):
        """
        Requests from the world gazebo model the list of
        current models and their poses. Then it updates
        the existing catalog of items in its inventory based
        on its associated ID and new position. Thus if we
        bump an object and move it, we can still track its
        presence throughout the simulation. Copied over in a hurry
        from litterbug.
        """
        # We start by requesting from gazebo the list
        # of all current models
        updated_items = self.__get_model_list()
        # updated_items = self.__object_tracking_node.get_model_list()

        # To prevent multiple iterations, we will use a dict for
        # rapid reference of poses (position, orientation)
        updates: Dict[
            str, Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]
        ] = {}
        for item in updated_items:
            updates[item.name] = (item.origin, item.orientation)

        # Now we iterate through known items and update their
        # position and orientation as needed.
        with self.__items_lock:
            for item in self.__items:
                # Ignore humans
                if item.label in HUMAN_LABELS:
                    continue

                if item.name in updates:
                    item.origin = updates[item.name][0]
                    item.orientation = updates[item.name][1]

    def spin(self):
        """
        spin starts the omniscience module
        """
        self.__executor.add_node(self)
        Thread(target=self.__executor.spin, daemon=True).start()


HUMAN_LABELS = ["human", "person", "man", "woman"]
