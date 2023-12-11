from __future__ import annotations
from planner.action import Action

import rclpy

import numpy as np

from typing import Dict, Optional, Tuple, List, Union
from math import pi, degrees
from time import sleep

from capstone_interfaces.srv import ObjectIDQuery
from capstone_interfaces.srv import PickUpObject as PickUpObjectMsg
from capstone_interfaces.srv import GiveObject as GiveObjectMsg
from capstone_interfaces.msg import StateObject, Room

from threading import Lock

from litterbug.items import Item
from planner.omniscience import OmniscienceModule
from planner.interaction import InteractionModule
from planner.navigation import NavigationModule
from planner.vision import VisionModule
from planner.state import StateModule

"""
MoveToObject - move, uninterrupted if possible, to the last known location
    of a given object id. Succeeds if the robot moves close to the object
    and if the object is still there.

MoveToRoom - move, uninterrupted if possible, to the center of a given room.
    Succeeds if you are close to the center of the room.

MoveToHuman - move, uninterrupted if possible, to the last known location
    of a given human. Succeeds if the robot moves close to the human and
    if the human is still there.

LookAroundRobot - SPIN!

SearchAreaForObject - (search area) Given an object ID, will create a small search pattern
    immediately around the robot for a given object. Will stop if the object
    is spotted or if the search area is exhausted. Use only if you are in an
    area where the object is likely to be.

PickupObject - Given an object ID, will attempt to pick up the object if it is
    within reach. Navigate to the object prior to picking it up if possible.

GiveObject - Given an object ID, will attempt to give the object to a human.
    Succeeds only if a human is nearby to take the object and the object is
    within reach. Navigate to the human prior to giving the object if possible.

DoISeeObject - Given an object ID, will return whether or not the robot has
    recently seen a given object with the label provided within a set time
    window and distance of the robot. The time window and distance is coded
    to be "recent".
"""


class MoveToObject(Action):
    """
    MoveToObject is an action that will, given the ID of an object.
    attempt to move to it. If the object is not known to the robot
    or if the robot fails to maneuver to it, then the action will
    report

    Result will be a tuple of (success, message), where message is
    set for the "why" in the event of a unsuccessful move.
    """

    def __init__(
        self,
        navigator: NavigationModule,
        vision: VisionModule,
        state: StateModule,
        omniscience: OmniscienceModule,
    ):
        """
        Create a new MoveToObject action.
        Parameters:
        - navigator: The navigation module to use for moving
        - vision: The vision module to use for object tracking
        - object_query_client: The client to use for querying the
            object database for location/existence
        """
        super().__init__("MoveToObject")
        self.__navigator = navigator
        self.__vision = vision
        self.__state = state
        self.__omniscience = omniscience

        self.__object_id_lock = Lock()
        self.__object_id: str = ""

    def _execute(self, object_to_move_to: Union[str, int]):
        """
        _execute requests the robot to move to a given location.
        """
        # Check to see if the object is a string that can be
        # easily converted to an int; if so, convert it to an
        # int
        try:
            object_to_move_to = int(object_to_move_to)
        except ValueError:
            pass

        if isinstance(object_to_move_to, str):
            # If the object is a string, we need to query the
            # state module to find the object ID
            objects = self.__state.query_for_object(object_to_move_to)
            if objects is None or len(objects) == 0:
                self._set_result((False, "item not known"))
                return
            else:
                # We have multiple objects; find the closest
                # to our robot
                object = objects[0]
                object_to_move_to = objects[0].id
        elif isinstance(object_to_move_to, int):
            object = self.__state.query_for_object_id(object_to_move_to)
            if object is None:
                self._set_result((False, "item not known"))
                return
        else:
            self._set_result((False, "item not known"))

        with self.__object_id_lock:
            self.__object_id = object.id

        location = object.position

        self.__navigator.mover2(location, 0.75)
        sleep(0.25)

        # If we see the object within set distance in the past
        # five seconds, we succeed
        # Check to see if the object is nearby via omniscience
        # for ease of simulation due to issues w/ vision
        robot_position, _ = self.__navigator.get_current_pose()
        if self.__omniscience.am_i_near(
            object.description, 1.25, location=robot_position
        ):
            self._set_result((True, ""))
            return
        else:
            self._set_result((False, "object not seen"))
            return

    def _cancel(self):
        self.__navigator.cancel()

    def clone(self) -> MoveToObject:
        return MoveToObject(
            self.__navigator,
            self.__vision,
            self.__state,
            self.__omniscience,
        )


class MoveToRoom(Action):
    """
    MoveToRoom will move to a room given a name. The target location
    in the room will be its centroid
    """

    def __init__(self, navigator: NavigationModule, state: StateModule):
        super().__init__("MoveToRoom")

        self.__navigator = navigator
        self.__state = state

    def _execute(self, room_to_move_to: str):
        """ """
        room = self.__get_room(room_to_move_to)

        if room is None:
            self._set_result((False, "room not known"))
            return

        location = (room.x, room.y)

        self.__navigator.move_to(location, lambda x: None)

        while True:
            if self._is_cancelled():
                self._set_result((False, "cancelled"))
                return
            position, _ = self.__navigator.get_current_pose()

            distance = self.__navigator.distance(location, position)
            if distance < 1.0:
                self.__navigator.cancel()
                self._set_result((True, ""))
                return

            sleep(0.25)

    def __get_room(self, room_name: str) -> Optional[Room]:
        """
        Get the room from the query_services
        """
        rooms = self.__state.get_rooms()

        for room in rooms:
            if room.name == room_name:
                return room
        return None

    def _cancel(self):
        self.__navigator.cancel()

    def clone(self):
        return MoveToRoom(self.__navigator, self.__state)


class MoveToHuman(Action):
    """
    MoveToHuman will move to our human on request
    """

    def __init__(self, navigator: NavigationModule, state: StateModule):
        super().__init__("MoveToHuman")

        self.__navigator = navigator
        self.__state = state

    def _execute(self):
        """ """
        location = self.__state.get_human_location()

        # result, msg = self.__navigator.move_to_synchronous(
        #     location, distance_for_success=1.0
        # )
        self.__navigator.move_to(location, self.__complete_callback)

        while True:
            if self._is_cancelled():
                self._set_result((False, "cancelled"))
                return
            position, _ = self.__navigator.get_current_pose()

            distance = self.__navigator.distance(location, position)
            if distance < 1.0:
                self.__navigator.cancel()
                self._set_result((True, ""))
                return

            sleep(0.25)

    def __complete_callback(self, results: Tuple[bool, str]):
        pass

    def _cancel(self):
        self.__navigator.cancel()

    def clone(self):
        return MoveToHuman(self.__navigator, self.__state)


class SearchAreaForObject(Action):
    def __init__(self):
        super().__init__("SearchRoomForObject")

    def _execute(self, room_to_search: str, object_to_search_for: str):
        """ """
        pass

    def _cancel(self):
        pass

    def clone(self):
        pass


class PickUpObject(Action):
    """
    PickUpObject will attempt to pick up an item. It returns a tuple of
    (success, message), where message is set for the "why" in the
    event of a unsuccessful pickup.
    """

    def __init__(self, interaction_module: InteractionModule, state: StateModule):
        super().__init__("PickUpObject")
        self.__interaction = interaction_module
        self.__state = state

    def _execute(self, object_to_pickup: Union[str, int]):
        """
        Send a pickup object request
        """
        if isinstance(object_to_pickup, int):
            object = self.__state.query_for_object_id(object_to_pickup)
            if object is None:
                self._set_result((False, "object not known"))
                return
            else:
                object_to_pickup = object.description

        result = self.__interaction.pickup_object(object_to_pickup)
        self._set_result(result)

    def _cancel(self):
        """
        There is no real cancellation of this action as is
        """
        pass

    def clone(self):
        return PickUpObject(self.__interaction, self.__state)


class GiveObject(Action):
    """
    GiveObject will attempt to give an item to a human. It returns a tuple of
    (success, message), where message is set for the "why" in the
    event of a unsuccessful pickup.
    """

    def __init__(self, interacton_module: InteractionModule, state: StateModule):
        super().__init__("GiveObject")
        self.__interaction = interacton_module
        self.__state = state

    def _execute(self, object_to_give: Union[str, int]):
        """
        Send a give object request
        """
        if isinstance(object_to_give, int):
            object = self.__state.query_for_object_id(object_to_give)
            if object is None:
                self._set_result((False, "object not known"))
                return
            else:
                object_to_give = object.description

        result = self.__interaction.give_object(object_to_give)
        self._set_result(result)

    def _cancel(self):
        """
        There is no real cancellation of this action as is
        """
        pass

    def clone(self):
        return GiveObject(self.__interaction, self.__state)


class DoISee(Action):
    """
    DoISee will return whether or not the robot has recently seen
    a given object with the label provided within a set time window
    and distance of the robot. It returns a tuple of (success, message),
    where message is set for the "why" in the event of not seeing it.
    """

    def __init__(self, vision: VisionModule, omnisicence: OmniscienceModule):
        super().__init__("DoISeeObject")
        self.__vision = vision
        self.__omniscience = omnisicence

    def _execute(self, object_to_look_for: Union[str, int]):
        """
        Send a give object request
        """
        # If we are within a set distance assume we see it due to issues
        # with close range vision on our simulator
        if self.__omniscience.am_i_near(object_to_look_for, 1.0):
            self._set_result(True)
            return

        if self.__vision.is_nearby_since(object_to_look_for, 8.0, 5.0):
            self._set_result(True)
        else:
            self._set_result(False)

    def _cancel(self):
        """
        There is no real cancellation of this action as is
        """
        pass

    def clone(self):
        return DoISee(self.__vision, self.__omniscience)


class LookAround(Action):
    """
    LookAround will spin the robot around in place
    """

    def __init__(
        self,
        navigation: NavigationModule,
        vision: VisionModule,
        omnisciense: OmniscienceModule,
    ):
        super().__init__("LookAround")
        self.__vision = vision
        self.__navigation = navigation
        self.__omniscience = omnisciense
        self.__cancel_flag = False
        self.__cancel_lock = Lock()

    def __items_check(self, objects_to_look_for: List[str]) -> List[Tuple[str, float]]:
        vision_distance = 8.0
        vision_since = 5.0

        stuff_found: List[Tuple[str, float]] = []
        for object in objects_to_look_for:
            # If we are next to an item we "auto-see" it
            objects = self.__omniscience.am_i_near_objects(object, 1.0)
            for object in objects:
                item: Item = object[0]
                distance = object[1]
                stuff_found.append((item.label, distance))
            objects = self.__vision.is_nearby_since_items(
                object, vision_distance, vision_since
            )
            for object in objects:
                label: str = object[0]
                distance = object[1]
                stuff_found.append((label, distance))
        return stuff_found

    def filter_items(self, items: List[Tuple[str, float]]) -> List[Tuple[str, float]]:
        """
        Filter the items to identify likely duplicates and throw
        them out
        """
        # Move through each item and see if we essentially "duplicated"
        # objects by determining a match on label and a "too close"
        # distance

        objects: Dict[str, List[float]] = {}

        for item in items:
            if item[0] not in objects:
                objects[item[0]] = [item[1]]
            else:
                for distance in objects[item[0]]:
                    if abs(distance - item[1]) < 0.1:
                        # Too close, throw it out
                        continue

                    objects[item[0]].append(item[1])

        # Convert objects to a list again
        items = []
        for key in objects:
            for distance in objects[key]:
                items.append((key, distance))

        return items

    def _execute(self, objects_to_look_for: Union[str, int, List[str], List[int]]):
        """
        Send a give object request
        """

        if isinstance(objects_to_look_for, int):
            objects_to_look_for = [objects_to_look_for]
        elif isinstance(objects_to_look_for, str):
            objects_to_look_for = [objects_to_look_for]

        stuff_found: List[Tuple[str, float]] = []

        # Check before spinning
        stuff_found = self.__items_check(objects_to_look_for)
        if len(stuff_found) > 0:
            self._set_result(self.filter_items(stuff_found))
            return

        for spin in range(4):
            # We want to spin pi / 2, but we add a bit as a buffer
            # as nav2 allows error room, and tends to undershoot in
            # practice. Plus the important thing is room coverage,
            # not precision here.
            # rotation = (pi / 2) + (pi / 8)
            rotation = pi / 2
            self.__navigation.spin_synchronous(rotation)
            # self.__navigation.spinner(rotation)

            # Check to see if we canceled the rotation since rotating
            with self.__cancel_lock:
                if self.__cancel_flag:
                    self._set_result([])
                    return

            stuff_found = self.__items_check(objects_to_look_for)
            if len(stuff_found) > 0:
                self._set_result(self.filter_items(stuff_found))
                return

        # If we've made it here, we didn't see it
        self._set_result([])

    def _cancel(self):
        """
        There is no real cancellation of this action as is
        """
        with self.__cancel_lock:
            self.__cancel_flag = True
        self.__navigation.cancel()

    def clone(self):
        return LookAround(self.__navigation, self.__vision, self.__omniscience)
