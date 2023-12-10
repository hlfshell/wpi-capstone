from __future__ import annotations

from math import sqrt
from threading import Lock, Thread
from time import sleep
from typing import Dict, List, Tuple, Optional

import rclpy
from capstone_interfaces.msg import Item as ItemMsg
from capstone_interfaces.msg import ItemsState
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

    def __init__(self):
        super().__init__("omniscience_module")
        self.__executor = None

        self.__items_lock = Lock()
        # self.__items_by_type is a dictionary of item type to list of items
        # and self.__items_by_id is a dictionary of item id to item
        self.__items: List[Item] = []

        self.__odometry_subscriber = self.create_subscription(
            msg_type=Odometry,
            topic="/odom",
            callback=self.__pose_callback,
            qos_profile=10,  # Keep last
        )

        self.__pose_tripwire: bool = True
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

        self.__items_state_subscriber = self.create_subscription(
            msg_type=ItemsState,
            topic="/litterbug/items_state",
            callback=self.__items_state_callback,
            qos_profile=10,  # Keep last
        )

    def __items_state_callback(self, msg: ItemsState):
        """
        __items_state_callback updates the items state
        """
        items: List[Item] = []
        with self.__items_lock:
            for item in msg.items:
                items.append(
                    Item(
                        item.name,
                        item.label,
                        "",  # model
                        (item.x, item.y),
                        (0.0, 0.0, 0.0, 1.0),  # orientation
                    )
                )

            self.__items = items

    def __get_item(self, item: str) -> List[Item]:
        """
        __get_item returns the item by its name or id
        """
        items: List[Item] = []
        with self.__items_lock:
            for i in self.__items:
                if isinstance(item, str):
                    if i.label == item:
                        items.append(i)
                else:
                    raise "No ints supported"
                    if i.name == item:
                        items.append(i)
        return items

    def am_i_near(
        self, item: str, distance: float, location: Optional[Tuple[float, float]] = None
    ) -> bool:
        """
        Check if the robot is near the item.
        """
        self.get_logger().info(f"am_i_near: {item}, {distance}, {location}")
        if location is None:
            location = self.robot_position()

        items = self.__get_item(item)
        self.get_logger().info(f"am_i_near: {items}")

        for target in items:
            target = self.__distance(location, target.origin)
            check = target <= distance
            self.get_logger().info(f"am_i_near: {target}, {check}")
            if check:
                self.get_logger().info(f"am_i_near: {True}")
                return True
        self.get_logger().info(f"am_i_near: {False}")
        return False

    def am_i_near_objects(
        self, item: str, distance: float, location: Optional[Tuple[float, float]] = None
    ) -> List[Tuple[Item, float]]:
        """
        Check if the robot is near the item, returning all matching nearby items
        """
        if location is None:
            location = self.robot_position()

        items = self.__get_item(item)

        found: List[Item] = []

        for target in items:
            distance_from = self.__distance(location, target.origin)
            check = distance_from <= distance
            if check:
                found.append((target, distance_from))

        # Sort found by distance, closest first (the second element of the tuple)
        found.sort(key=lambda x: x[1])

        return found

    def robot_position(self) -> Tuple[float, float]:
        """
        robot_position returns the last known position
        of the robot. Will lock until the first odom msg is
        received.
        """
        with self.__pose_lock:
            self.__pose_tripwire = False
        while True:
            with self.__pose_lock:
                if not self.__pose_tripwire:
                    position = self.__position
                    break
            sleep(0.1)
        # with self.__pose_lock:
        #     return self.__position
        return position

    def __pose_callback(self, msg: Odometry):
        """
        __pose_callback updates our current position of the robot
        """
        with self.__pose_lock:
            if not self.__pose_tripwire:
                self.__pose_tripwire = True
            self.__position = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
            )

    def __distance(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """
        __distance returns the distance between two points
        """
        return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def spin(self):
        """
        spin starts the omniscience module
        """
        if self.__executor is not None:
            raise Exception("Already spinning")
        self.__executor = rclpy.executors.MultiThreadedExecutor()
        self.__executor.add_node(self)
        Thread(target=self.__executor.spin, daemon=True).start()


HUMAN_LABELS = ["human", "person", "man", "woman"]
