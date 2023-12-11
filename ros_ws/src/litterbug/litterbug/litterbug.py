from sys import argv
import argparse
import math
from os import path
from threading import Lock, Thread
from time import sleep
from typing import Callable, Dict, List, Tuple

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from capstone_interfaces.msg import ObjectSpotted, HumanSpotted, ItemsState
from capstone_interfaces.msg import Item as ItemMsg
from capstone_interfaces.srv import GiveObject, PickUpObject
from nav_msgs.msg import Odometry
from rclpy.node import Node

from litterbug.gazebo import Gazebo, ItemAlreadyExists
from litterbug.items import Item
from litterbug.map import Map


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
    vision_range - the range at which the robot can see
        objects.
    fov - the vision cone angle in radians
    vision_fps - the max frequency at which the robot will
        perform a vision check in frames per second
    """

    def __init__(
        self,
        items: List[Item],
        map: Map,
        interaction_range: float = 1.5,
        vision_range: float = 8.0,
        fov: float = math.radians(40.0),
        vision_fps: int = 12,
        models_directory: str = "./models",
        enable_vision_simulation: bool = True,
    ):
        super().__init__("litterbug_service")

        self.__map = map

        self.__gazebo = Gazebo(models_dir=models_directory)

        self.__interaction_range = interaction_range
        self.__vision_range = vision_range
        self.__fov = fov
        self.__enable_vision_simulation = enable_vision_simulation

        # World and robot inventory management
        self.__world_items_lock = Lock()
        self.__world_items = items

        self.__robots_possession_lock = Lock()
        self.__robots_possession: List[Item] = []

        # Tracking the robot pose
        self.__odometry_subscriber = self.create_subscription(
            msg_type=Odometry,
            topic="/odom",
            callback=self.__update_robot_pose,
            qos_profile=10,  # Keep last
        )
        self.__robot_pose_lock = Lock()
        self.__robot_location: Tuple[float, float] = (0.0, 0.0)
        self.__robot_orientation: float = 0.0  # radians

        # Track the human (We're assuming one, unmoving... physically,
        # not emotionally)
        self.__human_position_lock = Lock()
        self.__human_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.__human_spotted_times: int = 0

        # Vision and handling
        self.__human_spotted_publisher = self.create_publisher(
            msg_type=HumanSpotted,
            topic="/human_spotted",
            qos_profile=10,  # Keep last
        )
        if self.__enable_vision_simulation:
            self.__object_spotted_publisher = self.create_publisher(
                msg_type=ObjectSpotted,
                topic="/object_spotted",
                qos_profile=10,  # Keep last
            )

            # Emulate our camera at the set fps
            self.create_timer(1.0 / vision_fps, self.vision_scan)
        else:
            # If we aren't the source of human detection, we
            # need to subscribe to it
            self.__human_subscription = self.create_subscription(
                msg_type=HumanSpotted,
                topic="/human_spotted",
                callback=self.__human_spotted_callback,
            )

        # Track the changes in the world's objects at 5Hz
        # so we react to them as we'd expect w/ interactions
        self.create_timer(1.0 / 5.0, self.update_items_locations)
        # We also broadcast the "omniscient" state of all items
        # for management reasons
        self.__item_state_publisher = self.create_publisher(
            msg_type=ItemsState,
            topic="/litterbug/items_state",
            qos_profile=10,  # Keep last
        )
        self.create_timer(1.0 / 5.0, self.__publish_items_state)

        # Services for interaction requests
        self.__pickup_service = self.create_service(
            srv_type=PickUpObject,
            srv_name="pickup_object",
            callback=self.__pickup_object,
        )
        self.__place_service = self.create_service(
            srv_type=GiveObject,
            srv_name="place_object",
            callback=self.__give_object,
        )
        self.__pickup_announcement = self.create_publisher(
            msg_type=ItemMsg,
            topic="/litterbug/pickup",
            qos_profile=10,  # Keep last
        )

    def wait_for_ready(self):
        self.__gazebo.wait_for_ready()

    def populate(self):
        """
        populate adds all items to the simulation, ignoring
        errors where they exist already.
        """
        self.get_logger().info("Populating world")
        with self.__world_items_lock:
            for item in self.__world_items:
                self.get_logger().info(
                    f"Loading {item.name} - {item.label} - {item.spawn}"
                )
                if not item.spawn:
                    continue
                if item.label in HUMAN_LABELS:
                    with self.__human_position_lock:
                        self.__human_position = item.origin
                        self.__human_spotted_times = 1
                        # Broadcast the human_spotted on init
                        # to prep the query service and other
                        # modules
                        self.__human_spotted_publisher.publish(
                            HumanSpotted(
                                x=item.origin[0],
                                y=item.origin[1],
                            )
                        )
                try:
                    self.__gazebo.spawn_item(item, self.get_logger())
                except ItemAlreadyExists:
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

    def pickup_item(self, item: Item):
        """
        pickup_item will attempt to pick up an item from the
        world if it exists and is within range/pickupable.
        If so, it gets added to the robot's inventory and
        removed from the world otherwise. If not, it will
        raise a CanNotPickUpObject exception
        """
        # Double check item proximity
        if not self.__proximity_check(item):
            raise CanNotPickUpObject(item, "Item not within proximity")

        with self.__world_items_lock:
            with self.__robots_possession_lock:
                # Remove the item from the world items
                self.__world_items.remove(item)
                try:
                    self.__gazebo.delete_item(item)
                except Exception as e:
                    self.get_logger().error(str(e))
                    raise CanNotPickUpObject(item, "Item was not present")

                # Add it to our robot posessions
                self.__robots_possession.append(item)

    def __human_spotted_callback(self, human: HumanSpotted):
        """
        __human_spotted_callback is the callback for when
        the human is spotted by the vision system. It will
        update the human's position.
        """
        with self.__human_position_lock:
            if self.__human_spotted_times < 100:
                self.__human_spotted_times += 1

            # The delta is based no how many times we've seen the
            # human. Over time, we limit the minimization
            # of how much we adjust based on the delta
            # as we don't want to minimize future updates
            # from having a real effect.
            delta = (
                human.x - self.__human_position[0],
                human.y - self.__human_position[1],
                human.z - self.__human_position[2],
            )

            delta = (
                delta[0] / self.__human_spotted_times,
                delta[1] / self.__human_spotted_times,
                delta[2] / self.__human_spotted_times,
            )

            self.__human_position = (
                self.__human_position[0] + delta[0],
                self.__human_position[1] + delta[1],
                self.__human_position[2] + delta[2],
            )

    def __pickup_object(
        self, request: PickUpObject.Request, response: PickUpObject.Response
    ):
        """
        __pickup_object is the ROS service callback to pick up
        and object
        """
        target = request.object.lower()

        # First confirm that we're not trying to break the first law
        # of robotics and pick up our dear human
        if target == "human":
            response.success = False
            response.status_message = "Cannot pick up a human - that's rude"
            return response

        # Get a list of all items we can interact with
        interactable = self.interactable_objects()

        # Find the first item that matches what we wish
        # to interact with
        item: Item = None
        for i in interactable:
            if i.name.lower() == target or i.label.lower() == target:
                item = i
                break
        if item is None:
            response.success = False
            response.status_message = (
                f"Could not find item {target} within interaction range"
            )
            return response

        try:
            self.pickup_item(item)
        except CanNotPickUpObject as e:
            response.success = False
            response.status_message = str(e)
            return response

        response.success = True

        # Announce that we picked up the object so state can handle
        # it
        self.__pickup_announcement.publish(
            ItemMsg(
                name=item.name,
                label=item.label,
                x=item.origin[0],
                y=item.origin[1],
            )
        )

        return response

    def give_item(self, item: Item):
        """
        give_item handles giving the human the object
        that the robot is carrying
        """

        # Are we close to the human to give them the object?
        with self.__human_position_lock:
            if (
                self.__distance_from_robot(self.__human_position)
                > self.__interaction_range
            ):
                raise CanNotGiveObject(
                    item,
                    "Human is not within interaction range",
                )

        # Simply remove the object from the robot's possession for now;
        # perhaps in the future there could be actual placement logic,
        # but we'll assume it returns back to the human and hands it to
        # them.
        with self.__robots_possession_lock:
            self.__robots_possession.remove(item)

    def __give_object(self, request: GiveObject.Request, response: GiveObject.Response):
        """
        __give_object is the service callback for ROS requests to
        give an object to the human
        """
        target = request.object.lower()

        # Do we have the item in our posession?
        with self.__robots_possession_lock:
            item = None
            for i in self.__robots_possession:
                if i.name.lower() == target or i.label.lower() == target:
                    item = i
                    break
        if item is None:
            response.success = False
            response.status_message = (
                f"Could not find item {target} within robot's possession"
            )
            return response

        try:
            self.give_item(item)
        except CanNotGiveObject as e:
            response.success = False
            response.status_message = str(e)
            return response

        response.success = True

        return response

    def __update_robot_pose(self, odometry: Odometry):
        """
        __update_robot_location updates the robot location
        as per the broadcasted last position.
        """
        pose = odometry.pose.pose
        position = pose.position
        quaternion = pose.orientation

        with self.__robot_pose_lock:
            self.__robot_location = (
                position.x,
                position.y,
            )
            _, _, psi = self.__quaternion_to_euler(
                quaternion.x,
                quaternion.y,
                quaternion.z,
                quaternion.w,
            )
            self.__robot_orientation = psi

    def __get_robot_pose(self) -> Tuple[Tuple[float, float], float]:
        """
        __get_robot_pose returns the current robot
        location as a tuple of (x, y) and its orientation
        as an angle in radians
        """
        with self.__robot_pose_lock:
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

    def __get_world_items(self) -> List[Item]:
        """
        __get_world_items returns a list of items in the world
        """
        with self.__world_items_lock:
            return self.__world_items

    def __distance(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """
        __distance returns the distance between two points
        """
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def __distance_from_robot(self, location: Tuple[float, float]) -> float:
        """
        __distance_from_robot returns the distance from the
        robot to the item
        """
        pose, _ = self.__get_robot_pose()

        if isinstance(location, Item):
            x, y, _ = location.origin
        elif len(location) == 3:
            x, y, _ = location
        else:
            x, y = location
        return self.__distance(pose, (x, y))

    def __proximity_check(self, item: Item) -> bool:
        """
        __proximity_check returns true if the item is within
        the robot's proximity range
        """
        x, y, _ = item.origin
        return self.__distance_from_robot((x, y)) < self.__interaction_range

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

    def __within_cone(
        self,
        origin: Tuple[float, float],
        target: Tuple[float, float],
        radius: float,
        heading: float,
        fov: float,
    ) -> bool:
        """
        __within_cone checks whether a cone projected of radius size and
        heading direction at width of the fov angle contains the given
        target location. The heading is the expected direction that the cone
        is projected at, and the fov is the total arc width of the cone
        (so +/- 1/2 fov). All angles are assumed to be in radians.
        """
        # First we confirm whether or not the distance between the points
        # are within the given radius; if not, we can't be within the cone
        if self.__distance(origin, target) > radius:
            return False

        # Next we confirm that the angle between the origin and the
        # target falls within the +/- fov range from the directed angle
        # of the cone
        angle_to_target = math.atan2(target[1] - origin[1], target[0] - origin[0])

        # We wish to then find the difference between the two angles. But
        # note that we have to deal with wrapping around the unit circle
        # if the cone overlaps the 0/2pi boundary. Thus we need to take the
        # minimum of the two possible angles
        angle_diff = min(
            abs(heading - angle_to_target),
            abs(2 * math.pi - heading - angle_to_target),
        )

        # Finally, our fov is a total arc width, so we need to divide by 2
        # and see if our angle falls within that distance from our heading
        # angle
        return angle_diff < fov / 2

    def __fuzzy_coordinates(self, target: Tuple[float, float]) -> Tuple[float, float]:
        """
        __fuzzy_coordinates returns a tuple of coordinates that are
        within a small range of the given origin coordinates with
        some random error introduced to emulate the robot's vision
        model being imperfect.
        """
        error_range = 0.05  # 5 centimeters of difference.
        error_x = np.random.normal(-error_range, error_range)
        error_y = np.random.normal(-error_range, error_range)
        error_z = np.random.normal(-error_range, error_range)
        return (target[0] + error_x, target[1] + error_y, target[2] + error_z)

    def __vision_detection_probability(self, location: Tuple[float, float]) -> bool:
        """
        __vision_detection_probability returns a boolean value on whether
        the robot sees it or not. The detection is based on a probability
        determine by how close it is to the robot.

        0-40% vision range - 95% chance
        40-60% vision range - 75% chance
        60-80% vision range - 50% chance
        80-100% vision range - 25% chance
        """
        distance = self.__distance_from_robot(location)
        if distance < self.__vision_range * 0.4:
            return np.random.choice([True, False], p=[0.95, 0.05])
        elif distance < self.__vision_range * 0.6:
            return np.random.choice([True, False], p=[0.75, 0.25])
        elif distance < self.__vision_range * 0.8:
            return np.random.choice([True, False], p=[0.5, 0.5])
        else:
            return np.random.choice([True, False], p=[0.25, 0.75])

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
        # First, cut down the possible Items down to ones within the
        # vision range for the robot. If that check works, then
        # also perform a line of site check for the two spots.
        pose, heading = self.__get_robot_pose()
        items = []
        for item in self.__get_world_items():
            x, y, _ = item.origin

            if self.__within_cone(
                pose,
                (x, y),
                self.__vision_range,
                heading,
                self.__fov,
            ) and self.__map.line_of_sight(pose, (x, y)):
                items.append(item)

        return items

    def vision_scan(self):
        """
        vision_scan is an emulated scan of the robot's vision. It
        will return a list of items that the robot has seen, and
        will also publish a message to the /object_spotted topic
        for each item.

        Each item that is scanned will have its coordinates fuzzed
        to emulate imperfect localization, and then published to
        ObjectSpotted.
        """
        items = self.vision_check()
        for item in items:
            if self.__vision_detection_probability(item.origin):
                # We are in fact spotting the object, so let's
                # publish its fuzzed coordinates
                x, y, z = self.__fuzzy_coordinates(item.origin)
                if item.label in HUMAN_LABELS:
                    self.__human_spotted_publisher.publish(
                        HumanSpotted(
                            x=x,
                            y=y,
                        )
                    )
                else:
                    self.__object_spotted_publisher.publish(
                        ObjectSpotted(
                            description=item.label,
                            x=x,
                            y=y,
                            z=z,
                        )
                    )

        return items

    def __publish_items_state(self):
        """
        __publish_items_state publishes the current state of all items
        in the world to the /litterbug/items_state topic
        """
        items = []
        with self.__world_items_lock:
            for item in self.__world_items:
                items.append(
                    ItemMsg(
                        name=item.name,
                        label=item.label,
                        x=item.origin[0],
                        y=item.origin[1],
                    )
                )

        self.__item_state_publisher.publish(
            ItemsState(
                items=items,
            )
        )

    def update_items_locations(self):
        """
        Requests from the world gazebo model the list of
        current models and their poses. Then it updates
        the existing catalog of items in its inventory based
        on its associated ID and new position. Thus if we
        bump an object and move it, we can still track its
        presence throughout the simulation.
        """
        # We start by requesting from gazebo the list
        # of all current models
        updated_items = self.__gazebo.get_model_list()
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
        with self.__world_items_lock:
            for item in self.__world_items:
                # If it's a human, we update the human's position
                # and not an item in the world inventory since we
                # can't interact with it beyond giving stuff
                if item.label in HUMAN_LABELS:
                    with self.__human_position_lock:
                        self.__human_position = item.origin

                    continue

                if item.name in updates:
                    item.origin = updates[item.name][0]
                    item.orientation = updates[item.name][1]
                else:
                    # If the item is not in the updates, then
                    # it has been removed and we need to perform
                    # a clean up action.
                    self.__world_items.remove(item)


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


class IntervalEvent:
    def __init__(self, interval: float, func: Callable, *args, **kwargs):
        self.interval = interval
        self.func = func
        self.args = args
        self.kwargs = kwargs

        self.__running_lock = Lock()
        self.__thread: Thread = None

    def start(self):
        with self.__running_lock:
            self.__running = True
            self.__thread = Thread(target=self.__start)
            self.__thread.start()

    def stop(self):
        with self.__running_lock:
            self.__running = False
            self.__thread = None

    def __start(self):
        while True:
            with self.__running_lock:
                if not self.__running:
                    break
            self.func(*self.args, **self.kwargs)
            sleep(self.interval)


class CanNotPickUpObject(Exception):
    """
    CanNotPickUp is an exception that is raised when the robot
    attempts to pick up an object that it can not pick up. This
    is typically because the object is not within range.
    """

    def __init__(self, item: Item, reason: str):
        self.item = item
        self.reason = reason

    def __str__(self):
        return f"Can not pick up {self.item} - {self.reason}"


class CanNotGiveObject(Exception):
    """
    CanNotGiveObject is an exception that is raised when the robot
    attempts to place an object that it can not place. This is
    typically because the object is not within range of the human.
    """

    def __init__(self, item: Item, reason: str):
        self.item = item
        self.reason = reason

    def __str__(self):
        return f"Can not place {self.item} - {self.reason}"


HUMAN_LABELS = ["human", "person", "man", "woman"]


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--enable-vision-simulation", type=str, default="True")
    args, _ = parser.parse_known_args(argv)
    enable_vision_simulation = args.enable_vision_simulation == "True"

    rclpy.init()

    models_dir = path.join(get_package_share_directory("litterbug"), "models")
    maps_dir = path.join(get_package_share_directory("litterbug"), "maps")
    items_dir = path.join(get_package_share_directory("litterbug"), "items")

    map = Map.FromMapFile(f"{maps_dir}/house")

    items = Item.from_csv(f"{items_dir}/items.csv")

    litterbug = Litterbug(
        items,
        map,
        models_directory=models_dir,
        enable_vision_simulation=enable_vision_simulation,
    )
    litterbug.wait_for_ready()
    litterbug.populate()

    rclpy.spin(litterbug)


if __name__ == "__main__":
    main()
