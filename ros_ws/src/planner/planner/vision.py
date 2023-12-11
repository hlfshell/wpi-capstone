from rclpy.node import Node
from threading import Lock


from capstone_interfaces.msg import ObjectSpotted
from nav_msgs.msg import Odometry

from typing import Dict, Tuple, Union, List, Optional

from time import time
import math


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
        self.__object_tracking: Dict[str, List[ObjectTracker]] = {}

        self.__odometry_subscriber = self.create_subscription(
            msg_type=Odometry,
            topic="/odom",
            callback=self.__pose_callback,
            qos_profile=10,  # Keep last
        )
        self.__position_lock = Lock()
        self.__position: Tuple[float, float] = (0.0, 0.0)

    def __object_spotted_callback(self, msg: ObjectSpotted):
        label = msg.description
        location = (msg.x, msg.y)

        with self.__object_spotted_lock:
            if label not in self.__object_tracking:
                self.__object_tracking[label] = [ObjectTracker(label, location)]
            else:
                # See if any of the objects are close enough to be the same item
                for object in self.__object_tracking[label]:
                    if object.distance(location) < 0.5:
                        object.spotted(location)
                        return

                # If we've reached this point we can assume it's
                # a new object and add it in
                self.__object_tracking[label].append(ObjectTracker(label, location))

    def __pose_callback(self, msg: Odometry):
        with self.__position_lock:
            self.__position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def is_nearby_since(
        self,
        object: Union[str, int],
        distance: float,
        time: float,
        location: Optional[Tuple[float, float]] = None,
    ):
        """
        is_nearby_since will determine if the specified object has
        been spotted within a set distance of the given location
        within the specified time limit.

        Accepts either a label (string) or a unique ID (int) for
        the object
        """
        if location is None:
            with self.__position_lock:
                location = self.__position

        with self.__object_spotted_lock:
            # if isinstance(object, str):
            if object not in self.__object_tracking:
                return False

            for target in self.__object_tracking[object]:
                if target.last_seen < time:
                    continue

                if target.distance(location) < distance:
                    return True

            return False
            # else:
            #     for targets in self.__object_tracking.values():
            #         for target in targets:
            #             if target.id != object:
            #                 continue

            #             if target.last_seen < time:
            #                 return False

            #             return target.distance(location) < distance

            #     return False

    def is_nearby_since_items(
        self,
        object: Union[str, int],
        distance: float,
        time: float,
        location: Optional[Tuple[float, float]] = None,
    ) -> List[Tuple[str, float]]:
        if location is None:
            with self.__position_lock:
                location = self.__position

        items: List[Tuple[str, float]] = []
        with self.__object_spotted_lock:
            if object not in self.__object_tracking:
                return []

            for target in self.__object_tracking[object]:
                if target.last_seen < time:
                    continue

                target_distance = target.distance(location)
                if target_distance < distance:
                    items.append((target.label, target_distance))

        return items


class ObjectTracker:
    """
    ObjectTrack is a class that handles the mathematical
    tracking of objects and associated metadata
    """

    def __init__(self, label: str, location: Tuple[float, float]):
        self.label = label
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
