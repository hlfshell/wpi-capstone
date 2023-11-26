from typing import List, Tuple
from threading import Lock
import math
import numpy as np
from litterbug.map import Map
from litterbug.items import Item
from litterbug.gazebo import Gazebo

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class Litterbug(Node):
    """
    Litterbug is a service that manages the objects within
    a simulated gazebo world. The goal of litterbug is to
    provide a control interface to emulate the robot being
    able to "grasp" or otherwise interact with different
    kinds of objects within the environment without having
    to actually simulate it.

    Litterbug will maintain an occupancy map loaded at the
    start via passing in a pgm file - notable because it
    will *not* contain populated items. We do this to
    prevent the robot from "seeing" through walls or being
    able to interact with things through obstacles. We need
    the pgm saved map instead of the current occupancy map
    as the objects we interact with may be they themselves
    obstacles that trigger during a check.

    Arguments:
    items - a list of known items that the robot can interact
        with in the world
    interaction_range - the range at which the robot can
        reach and interact with objects. It is a radial
        check and does not consider orientation
    """

    def __init__(self, items: List[Item], map: Map, interaction_range: float = 0.5):
        super().__init__("litterbug_service")

        self.__map = map

        self.__gazebo = Gazebo()
        print("before")
        self.__gazebo.wait_for_ready()
        print("ready")
        raise "stop"

        self.__interaction_range = interaction_range

        self.__world_items_lock = Lock()
        self.__world_items = items

        self.__robots_posession_lock = Lock()
        self.__robots_posession: List[Item] = []

        self.__robot_location_pose = Lock()
        self.__robot_location: Tuple[float, float] = (0.0, 0.0)
        self.__robot_orientation: float = 0.0  # radians

    def populate(self):
        """
        populate adds all items to the simulation, ignoring
        errors where they exist already.
        """
        for item in self.__items:
            try:
                self.__gazebo.add_item(item)
            except gazebo.ItemAlreadyExists:
                continue

    def interact(self, item: Item):
        """
        interact handles logic for when the robot interacts
        with the item. There are three possible responses to
        interaction:

        1. Nothing occurs - a message is returned because
            there is no interaction programmed.
        2. Pick up - the robot attempts to pick up the object
            and add it to the robots possession. If it can be
            picked up, then it is removed from the world and
            assumed to be on the robot's person
        3. Give - the robot attempts to give the object to
            our human operator. If it is in the robot's
            possession, it is removed from it and the world.
            Otherwise an error occurs.
        """

        # TODO - implement
        pass

    def __update_robot_pose(self, pose: PoseStamped):
        """
        __update_robot_location updates the robot location
        as per the broadcasted last position.
        """
        with self.__robot_location_lock:
            self.__robot_location = (pose.pose.position.x, pose.pose.position.y)
            _, _, psi = self.__quaternion_to_euler(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            )
            self.__robot_orientation = psi

    def __get_robot_pose(self) -> Tuple[Tuple[float, float], float]:
        """
        __get_robot_pose returns the current robot
        location as a tuple of (x, y) and its orientation
        as an angle in radians
        """
        with self.__robot_location_lock:
            return self.__robot_location, self.__robot_orientation

    def __quaternion_to_euler(self, x, y, z, w):
        """
        __quaternion_to_euler converts a quaternion to
        euler angles (in radians); returns a tuple of
        (phi, theta, psi)
        """
        phi = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))

        theta = 2.0 * (w * y - z * x)
        theta = 1.0 if theta > 1.0 else theta
        theta = -1.0 if theta < -1.0 else theta
        theta = math.asin(theta)

        psi = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        return phi, theta, psi

    def __add_world_item(self, item: Item):
        """
        __add_world_item adds an item to the world
        """
        with self.__world_items_lock:
            self.__world_items.append(item)

    def __remove_world_item(self, item: Item):
        """
        __remove_world_item removes an item from the world
        """
        with self.__world_items_lock:
            self.__world_items.remove(item)

    def __add_robot_item(self, item: Item):
        """
        __add_robot_item adds an item to the robot's possession
        """
        with self.__robots_posession_lock:
            self.__robots_posession.append(item)

    def __remove_robot_item(self, item: Item):
        """
        __remove_robot_item removes an item from the robot's possession
        """
        with self.__robots_posession_lock:
            self.__robots_posession.remove(item)

    def __distance(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """
        __distance returns the distance between two points
        """
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1])) ** 2

    def __distance_from_robot(self, item: Item) -> float:
        """
        __distance_from_robot returns the distance from the
        robot to the item
        """
        return self.__distance(self.__get_robot_pose(), item.origin)

    def __proximity_check(self, item: Item) -> bool:
        """
        __proximity_check returns true if the item is within
        the robot's proximity range
        """
        return self.__distance_from_robot(item) < self.__interaction_range

    def __is_visible(self, item: Item) -> bool:
        """
        __is_visible returns true if the item is visible
        to the robot
        """
        # TODO
        pass

    def interactable_objects(self) -> List[Item]:
        """
        interactable_objects returns a list of items that
        are within the robot's interaction range
        """
        with self.__world_items_lock:
            return list(filter(self.__proximity_check, self.__world_items))

    def interact_with(self, item: Item):
        """
        interact_with will attempt to interact with an object. If
        that object is not within range it will raise a
        CanNotInteractWithObject exception. Otherwise, the interaction
        function of the given item will be handled here.
        """
        if not self.__proximity_check(item):
            raise CanNotInteractWithObject(item)

        # TODO
        pass

    def vision_check(self) -> List[Item]:
        """
        vision_check is an emulated check to see what items the robot
        would see given its position and rotation. In short, we project
        a 2d cone from its origin outwards at a set angle and distance.
        For complexity reasons we ignore possible occlusion. We then,
        based on proximity to the robot, determine how likely it is
        the robot has seen the object. We roll the dice, and determine
        if the robot has seen the object or not, emulating a vision
        model's false negatives.
        """
        # TODO
        pass


class CanNotInteractWithObject(Exception):
    """
    CanNotInteractWithObject is an exception that is raised when
    the robot attempts to interact with an object that it can not
    interact with. This is typically because the object is not
    within range.
    """

    def __init__(self, item: Item):
        self.item = item

    def __str__(self):
        return f"Can not interact with {self.item}"
