import math
from threading import Lock
from time import time
from typing import Dict, List, Optional, Tuple, Union

from capstone_interfaces.msg import ObjectSpotted
from rclpy.node import Node


class VisionModule(Node):
    def __init__(self):
        super().__init__("vision_node")

        self.__object_spotted_publisher = self.create_subscription(
            msg_type=ObjectSpotted,
            topic="/object_spotted",
            callback=self.__object_spotted_callback,
            qos_profile=10,  # Keep last
        )

        self.__object_spotted_lock = Lock()
        self.__object_tracking: Dict[str, ObjectTracker] = {}

    def __object_spotted_callback(self, msg: ObjectSpotted):
        object_id = msg.description
        location = (msg.x, msg.y)

        with self.__object_spotted_lock:
            if object_id not in self.__object_tracking:
                self.__object_tracking[object_id] = ObjectTracker(object_id, location)
            else:
                self.__object_tracking[object_id].spotted(location)

    def is_nearby_since(
        self,
        object: str,
        location: Tuple[float, float],
        distance: float,
        time: float,
    ):
        """
        is_nearby_since will determine if the specified object has
        been spotted within a set distance of the given location
        within the specified time limit.
        """
        with self.__object_spotted_lock:
            if object not in self.__object_tracking:
                return False

            target = self.__object_tracking[object]

        if target.last_seen < time:
            return False

        return target.distance(location) < distance


class ObjectTracker:
    """
    ObjectTrack is a class that handles the mathematical
    tracking of objects and associated metadata
    """

    def __init__(self, id: str, location: Tuple[float, float]):
        self.id = id
        self.location = location
        self.last_seen = time()

        self.__times_seen = 1

    def spotted(self, location):
        """
        Mark the location for the object as seen. If the distance
        between the current location and the given location is
        significantly difference, this is not just error but
        rather a new location entirely. Reset the location
        and times seen for now. Otherwise, update the location
        based on its overall contribution to the average location.
        """
        if self.distance(location) > 0.5:
            self.location = location
            self.__times_seen = 1
        else:
            self.__times_seen += 1
            deltax = (location[0] - self.location[0]) / self.__times_seen
            deltay = (location[1] - self.location[1]) / self.__times_seen

            self.location = (
                self.location[0] + deltax,
                self.location[1] + deltay,
            )

    def distance(self, location: Tuple[float, float]) -> float:
        """
        __distance calculates the distance between the
        current location and the given location
        """
        return math.sqrt(
            (self.location[0] - location[0]) ** 2
            + (self.location[1] - location[1]) ** 2
        )
