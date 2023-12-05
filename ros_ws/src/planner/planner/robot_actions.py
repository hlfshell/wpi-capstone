from __future__ import annotations
from planner.action import Action

import rclpy
from rclpy.node import Client

from typing import Optional, Tuple, List, Union

from capstone_interfaces.srv import ObjectIDQuery
from capstone_interfaces.srv import PickUpObject as PickUpObjectMsg
from capstone_interfaces.srv import GiveObject as GiveObjectMsg
from capstone_interfaces.msg import StateObject, Room

from threading import Lock

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
        self, navigator: NavigationModule, vision: VisionModule, state: StateModule
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

        self.__object_id_lock = Lock()
        self.__object_id: str = ""

    def _execute(self, object_to_move_to: str):
        """
        _execute requests the robot to move to a given location.
        """
        # location = self.__get_item_location(object_to_move_to)
        object = self.__state.query_for_object_id(object_to_move_to)
        if object is None:
            self._set_result(False, "item not known")
            return

        with self.__object_id_lock:
            self.__object_id = object_to_move_to

        location = object.position

        # self.__navigator.move_to(location, self.__movement_complete_callback)

        result, msg = self.__navigator.move_to_synchronous(location)

        # If the result is a failure, we either cancelled or
        # failed to reach the spot for some reason.
        if not result:
            self._set_result((False, msg))
            return

        # Confirm if we've seen the item, which we should since
        # we just moved to it. Basically see if it's in front
        # of us and it hasn't moved.
        with self.__object_id_lock:
            object_id = self.__object_id

        # If we see the object within half a meter in the past
        # five seconds, we succeed
        if self.__vision.is_nearby_since(
            object_id,
            0.5,
            5.0,
        ):
            return self._set_result((True, ""))
        else:
            return self._set_result((False, "object not seen"))

    def _cancel(self):
        self.__navigator.cancel()

    def clone(self) -> MoveToObject:
        return MoveToObject(
            self.__navigator,
            self.__vision,
            self.__state,
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

        result, msg = self.__navigator.move_to_synchronous(
            location, distance_for_success=1.0
        )
        self._set_result((result, msg))

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

        result, msg = self.__navigator.move_to_synchronous(
            location, distance_for_success=0.5
        )
        self._set_result((result, msg))

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

    def __init__(self, pickup_client: Client):
        super().__init__("PickUpObject")
        self.__pickup_client = pickup_client

    def _execute(self, object_to_pickup: int):
        """
        Send a pickup object request
        """
        print("executing", object_to_pickup)
        object_to_pickup = str(object_to_pickup)
        print("calling")
        result = self.__pickup_client(object_to_pickup)
        print("result", result)
        # request: PickUpObjectMsg.Request = PickUpObjectMsg.Request()
        # request.object = object_to_pickup
        # print("making call")
        # future = self.__pickup_client.call_async(request)
        # print("spin time")
        # rclpy.spin_until_future_complete(self, future)
        # response: PickUpObjectMsg.Response = future.result()
        # print("response", response)

        # if not response.success:
        #     self._set_result(False, response.status_message)
        # else:
        #     self._set_result(True)

        # return

    def _cancel(self):
        """
        There is no real cancellation of this action as is
        """
        pass

    def clone(self):
        return PickUpObject(self.__pickup_client)


class GiveObject(Action):
    """
    GiveObject will attempt to give an item to a human. It returns a tuple of
    (success, message), where message is set for the "why" in the
    event of a unsuccessful pickup.
    """

    def __init__(self, give_client: Client):
        super().__init__("GiveObject")
        self.__give_client = give_client

    def _execute(self, object_to_give: str):
        """
        Send a give object request
        """
        request: GiveObjectMsg.Request = GiveObjectMsg.Request()
        request.object = object_to_give
        future = self.__give_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response: GiveObjectMsg.Response = future.result()

        if not response.success:
            self._set_result(False, response.status_message)
        else:
            self._set_result(True)

        return

    def _cancel(self):
        """
        There is no real cancellation of this action as is
        """
        pass

    def clone(self):
        return GiveObject(self.__give_client)


class DoISeeObject(Action):
    """
    DoISeeObject will return whether or not the robot has recently seen
    a given object with the label provided within a set time window
    and distance of the robot. It returns a tuple of (success, message),
    where message is set for the "why" in the event of not seeing it.
    """

    def __init__(self, vision: VisionModule):
        super().__init__("DoISeeObject")
        self.__vision = vision

    def _execute(self, object_to_look_for: Union[str, int]):
        """
        Send a give object request
        """
        if self.__vision.is_nearby_since(object_to_look_for, 0.5, 2.0):
            self._set_result(True)
        else:
            self._set_result(False)

    def _cancel(self):
        """
        There is no real cancellation of this action as is
        """
        pass

    def clone(self):
        return DoISeeObject(self.__vision)
