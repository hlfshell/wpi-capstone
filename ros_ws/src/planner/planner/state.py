from __future__ import annotations

import math
from typing import List, Tuple

import rclpy
from capstone_interfaces.msg import StateObject
from capstone_interfaces.srv import ObjectDescriptionQuery, ObjectIDQuery
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
        self.__query_client.wait_for_service()
        self.__object_id_client.wait_for_service()

    def query_for_object(self, description: str) -> List[Object]:
        request = ObjectDescriptionQuery.Request()
        request.object_description = description

        future = self.__query_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response: ObjectDescriptionQuery.Response = future.result()

        return [
            Object.FromStateObject(state_object)
            for state_object in response.states_of_objects
        ]

    def query_for_object_id(self, object_id: str) -> List[Object]:
        request = ObjectIDQuery.Request()
        request.object_id = object_id

        future = self.__object_id_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response: ObjectIDQuery.Response = future.result()

        return [
            Object.FromStateObject(state_object)
            for state_object in response.states_of_objects
        ]


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
