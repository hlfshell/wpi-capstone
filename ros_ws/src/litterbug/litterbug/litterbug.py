from typing import List, Tuple
from threading import Lock
import math
import numpy as np
from litterbug.utils import load_map_from_pgm, draw_map


class Litterbug:
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

    def __init__(self, items: List[Item], map_path: str, interaction_range: float = 0.5):
        self.__map = load_map_from_pgm(map_path)
        draw_map(self.__map)

        raise "stop"
        self.__gazebo = Gazebo()
        self.__gazebo.wait_for_ready()

        self.__interaction_range = interaction_range

        self.__world_items_lock = Lock()
        self.__world_items = items

        self.__robots_posession_lock = Lock()
        self.__robots_posession: List[Item] = []

        self.__robot_location_lock = Lock()
        self.__robot_location: Tuple[float, float] = (0.0, 0.0)

        # Load the pgm file and convert it into an array
        self.__map: np.ndarray = 

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

    def __update_robot_location(self, pose: PoseStamped):
        """
        __update_robot_location updates the robot location
        as per the broadcasted last position.
        """
        with self.__robot_location_lock:
            self.__robot_location = (pose.pose.position.x, pose.pose.position.y)

    def __get_robot_location(self) -> Tuple[float, float]:
        """
        __get_robot_location returns the current robot
        location as a tuple of (x, y)
        """
        with self.__robot_location_lock:
            return self.__robot_location

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
        return self.__distance(self.__get_robot_location(), item.origin)

    def __proximity_check(self, item: Item) -> bool:
        """
        __proximity_check returns true if the item is within
        the robot's proximity range
        """
        return self.__distance_from_robot(item) < self.__interaction_range

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
