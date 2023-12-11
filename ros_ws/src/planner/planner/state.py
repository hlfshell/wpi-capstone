from __future__ import annotations

import math
from typing import List, Tuple, Optional

import rclpy
from nav_msgs.msg import Odometry
from time import sleep
from threading import Lock
from capstone_interfaces.msg import StateObject, Room, HumanSpotted
from capstone_interfaces.srv import (
    ObjectDescriptionQuery,
    ObjectIDQuery,
    GetRooms,
    RoomByCoordinates,
    QueryHumanLocation,
)
from rclpy.node import Node


class StateModule(Node):
    """
    StateModule handles generic state queries
    with our ROS2 state package.  Simplifies
    having to pass around ROS2 clients
    everywhere.
    """

    def __init__(self):
        super().__init__("state_module")

        self.__query_client = self.create_client(
            ObjectDescriptionQuery, "/object_description_query"
        )
        self.__object_id_client = self.create_client(ObjectIDQuery, "/object_id_query")
        self.__get_rooms_client = self.create_client(GetRooms, "/get_rooms")
        self.__room_by_location_client = self.create_client(
            RoomByCoordinates, "/room_by_location"
        )

        # Human tracking
        self.__human_subscription = self.create_subscription(
            HumanSpotted,
            "/human_spotted",
            self.__human_callback,
            qos_profile=10,  # Keep last
        )
        self.__human_lock = Lock()
        self.__human_position = (0.0, 0.0)
        self.__human_times_seen = 0
        self.__human_query_client = self.create_client(
            QueryHumanLocation, "/human_location"
        )

        self.__odometry_subscriber = self.create_subscription(
            msg_type=Odometry,
            topic="/odom",
            callback=self.__pose_callback,
            qos_profile=10,  # Keep last
        )
        self.__first_pose_received: bool = False
        self.__pose_lock = Lock()
        self.__position: Tuple[float, float] = (0.0, 0.0)
        self.__orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)

        self.wait_for_service()

        self.query_human_location()

    def wait_for_service(self):
        self.__query_client.wait_for_service()
        self.__object_id_client.wait_for_service()
        self.__get_rooms_client.wait_for_service()
        self.__room_by_location_client.wait_for_service()
        self.__human_query_client.wait_for_service()

    def query_for_object(self, description: str) -> List[Object]:
        """
        Given a query about an object, see what objects fit
        """
        request = ObjectDescriptionQuery.Request()
        request.object_description = description

        future = self.__query_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response: ObjectDescriptionQuery.Response = future.result()

        # objects = [
        #     Object.FromStateObject(state_object)
        #     for state_object in response.states_of_objects
        # ]
        objects = []
        for object in response.states_of_objects:
            objects.append(Object.FromStateObject(object))
            self.get_logger().info(
                f"Object: {object.id}-{object.description} ({object.location})"
            )

        for object in objects:
            distance = self.distance_from_robot(object.position)
            self.get_logger().info(
                f"Distance to {object.id}:{object.description}: {distance}"
            )

        objects.sort(key=lambda object: self.distance_from_robot(object.position))
        if len(objects) > 0:
            self.get_logger().info(f"Sorted objects: {objects[0].position}")

        return objects

    def query_for_object_id(self, object_id: int) -> Optional[Object]:
        """
        Given an objects ID (not label) return the object if
        we know about it
        """
        request = ObjectIDQuery.Request()
        request.object_id = object_id

        future = self.__object_id_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response: ObjectIDQuery.Response = future.result()

        objects = [
            Object.FromStateObject(state_object)
            for state_object in response.states_of_objects
        ]

        if len(objects) == 0:
            return None
        else:
            return objects[0]

    def get_rooms(self) -> List[Room]:
        """
        get_rooms returns all known rooms
        """
        request = GetRooms.Request()

        future = self.__get_rooms_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response: GetRooms.Response = future.result()

        return response.rooms

    def get_room_by_location(self, location: Tuple[float, float]) -> Optional[Room]:
        """
        get_room_by_location returns the room associated with the given coordinates if any.
        """
        request = RoomByCoordinates.Request()
        request.x = location[0]
        request.y = location[1]

        future = self.__room_by_location_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response: RoomByCoordinates.Response = future.result()

        if response.room.name == "":
            return None
        else:
            return response.room

    def get_human_location(self) -> Tuple[float, float]:
        """
        get_human_location returns the last known location of the human
        """
        with self.__human_lock:
            return self.__human_position

    def query_human_location(self) -> Tuple[float, float]:
        """
        Queries the state service for the known human location, whereas
        get_human_location uses last known location via the module. It's
        a "I wrote the other one first" situation.
        """
        request = QueryHumanLocation.Request()

        future = self.__human_query_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response: QueryHumanLocation.Response = future.result()

        with self.__human_lock:
            self.__human_position = (response.pose.x, response.pose.y)

        return (response.pose.x, response.pose.y)

    def __human_callback(self, msg: HumanSpotted):
        with self.__human_lock:
            if self.__human_times_seen < 100:
                self.__human_times_seen += 1
            # Don't adjust too heavily at any given moment
            # but also don't penalize too heavily new data
            delta = (msg.x - self.__human_position[0], msg.y - self.__human_position[1])
            delta = (
                delta[0] / self.__human_times_seen,
                delta[1] / self.__human_times_seen,
            )

            self.__human_position = (
                self.__human_position[0] + delta[0],
                self.__human_position[1] + delta[1],
            )

    def robot_position(
        self,
    ) -> Tuple[Tuple[float, float], Tuple[float, float, float, float]]:
        """
        robot_position returns the last known position
        """
        while True:
            with self.__pose_lock:
                if self.__first_pose_received:
                    break
            sleep(0.1)
        with self.__pose_lock:
            return self.__position, self.__orientation

    def __pose_callback(self, msg: Odometry):
        with self.__pose_lock:
            self.__first_pose_received = True

            self.__position = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
            )
            self.__orientation = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )

    def distance_from_robot(self, location: Tuple[float, float]) -> float:
        """
        distance_from_robot returns the distance from the robot to a given location
        """
        robot_location, _ = self.robot_position()

        return self.distance(robot_location, location)

    def distance(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """
        __distance returns the distance between two points
        """
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


class Object:
    def __init__(
        self,
        id: str,
        description: str,
        room: str,
        position: Tuple[float, float, float],
        time_seen: float,
    ):
        self.id = id
        self.description = description
        self.room = room
        self.position = position
        self.time_seen = time_seen

    def FromStateObject(state_object: StateObject) -> Object:
        return Object(
            str(state_object.id),
            state_object.description,
            state_object.location,
            (state_object.x, state_object.y, state_object.z),
            state_object.time_seen,
        )

    def distance(self, location: Tuple[float, float]) -> float:
        """
        distance calculates the distance between the object and a given location
        """
        return math.sqrt(
            (self.position[0] - location[0]) ** 2
            + (self.position[1] - location[1]) ** 2
        )
