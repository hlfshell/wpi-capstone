from typing import List, Tuple
from threading import Lock
import math
import numpy as np
from litterbug.map import Map
from litterbug.items import Item
from litterbug.gazebo import Gazebo, ItemAlreadyExists

from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.clock import ClockType

# from capstone_interfaces.msg import ObjectSpotted


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

    def __init__(
        self,
        items: List[Item],
        map: Map,
        interaction_range: float = 0.5,
        vision_range: float = 5.0,
        fov: float = math.radians(40.0),
    ):
        super().__init__("litterbug_service")

        self.__map = map

        self.__gazebo = Gazebo()
        # self.__gazebo.wait_for_ready()

        self.__interaction_range = interaction_range
        self.__vision_range = vision_range
        self.__fov = fov

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

        # Object broadcaster
        # self.__object_spotted_publisher = self.create_publisher(
        #     msg_type=ObjectSpotted,
        #     topic="/object_spotted",
        #     qos_profile=10,  # Keep last
        # )

    def populate(self):
        """
        populate adds all items to the simulation, ignoring
        errors where they exist already.
        """
        for item in self.__items:
            try:
                self.__gazebo.add_item(item)
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
            # self.__robot_orientation = -psi
            self.__robot_orientation = psi
        print("firing test")
        self.test()

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

    def __get_world_items(self) -> List[Item]:
        """
        __get_world_items returns a list of items in the world
        """
        with self.__world_items_lock:
            return self.__world_items

    def __add_robot_item(self, item: Item):
        """
        __add_robot_item adds an item to the robot's possession
        """
        with self.__robots_possession_lock:
            self.__robots_possession.append(item)

    def __remove_robot_item(self, item: Item):
        """
        __remove_robot_item removes an item from the robot's possession
        """
        with self.__robots_possession_lock:
            self.__robots_possession.remove(item)

    def __distance(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """
        __distance returns the distance between two points
        """
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def __distance_from_robot(self, item: Item) -> float:
        """
        __distance_from_robot returns the distance from the
        robot to the item
        """
        pose, _ = self.__get_robot_pose()
        return self.__distance(pose, item.origin)

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
        # angle_to_target = math.atan2(origin[1] - target[1], origin[0] - target[0])
        # angle_to_target = math.atan2(origin[0] - target[0], origin[1] - target[1])

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
        return target
        # error_range = 0.05  # 5 centimeters of difference.
        # error_x = np.random.normal(-error_range, error_range)
        # error_y = np.random.normal(-error_range, error_range)
        # error_z = np.random.normal(-error_range, error_range)
        # return (target[0] + error_x, target[1] + error_y, target[2] + error_z)

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
        return True
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

            img = self.__map.to_rgb()

            # draw the item
            pixel_x, pixel_y = (
                int((x - self.__map.origin[0]) / 0.05),
                img.shape[0] - int((y - self.__map.origin[1]) / 0.05),
            )

            # print(
            #     self.__within_cone(
            #         pose,
            #         (x, y),
            #         self.__vision_range,
            #         heading,
            #         self.__fov,
            #     )
            # )

            # cv2.circle(img, (pixel_x, pixel_y), 5, (0, 255, 0), -1)
            # img = self.__map.resize_img(img, size=800)
            # cv2.imshow("Vision", img)
            # cv2.waitKey()

            # raise "stop"

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
                self.__object_spotted_publisher.publish(
                    ObjectSpotted(
                        name=item.name,
                        x=x,
                        y=y,
                        z=z,
                    )
                )

        return items

    def test(self):
        img = self.__map.to_rgb()

        # now move through each (x,y) and if it is in the cone mark it
        # as blue
        pose, heading = self.__get_robot_pose()
        # print(">>", pose, heading)

        # raise "stop"

        pixel_count = 0
        cone_count = 0

        center = (0, 0)
        center_pixel = (
            int((center[0] - self.__map.origin[0]) / 0.05),
            int((center[1] - self.__map.origin[1]) / 0.05),
        )

        # print("center", center_pixel, self.__map.origin)
        cv2.circle(img, center_pixel, 5, (0, 0, 255), -1)

        # print("img shape late", img.shape)

        # cv2.circle(img, (10, 0), 5, (0, 0, 255), -1)
        # cv2.circle(img, (0, 10), 5, (0, 255, 0), -1)

        # print("map shape", self.__map.map.shape)

        # for y in range(self.__map.map.shape[0]):
        #     for x in range(self.__map.map.shape[1]):
        #         x_meters = (x * 0.05) + self.__map.origin[0]
        #         y_meters = (y * 0.05) + self.__map.origin[1]
        #         pixel_count += 1
        #         if self.__within_cone(
        #             pose,
        #             (x_meters, y_meters),
        #             self.__vision_range,
        #             heading,
        #             self.__fov,
        #         ):
        #             cone_count += 1

        #             # img[y, x] = (255, 0, 0)
        #             img[x, y] = (255, 0, 0)

        # result = self.__map.line_of_sight(pose, (0.026827, 1.776700))
        # print("line of site check", result)

        # for x in range(img.shape[0]):
        #     for y in range(img.shape[1]):
        #         x_meter = (x * 0.05) + self.__map.origin[0]
        #         y_meter = (y * 0.05) + self.__map.origin[1]
        #         pixel_count += 1

        # for pixel_x in range(img.shape[0]):
        #     for pixel_y in range(img.shape[1]):
        # for pixel_y in range(img.shape[0]):
        #     for pixel_x in range(img.shape[1]):
        # x = (pixel_x * 0.05) + self.__map.origin[0]
        # y = (pixel_y * 0.05) + self.__map.origin[1]
        # pixel_count += 1
        # if self.__within_cone(
        #     pose,
        #     (x, y),
        #     self.__vision_range,
        #     heading,
        #     self.__fov,
        # ):
        #     cone_count += 1
        #     img[pixel_x, pixel_y] = (255, 0, 0)
        #     # img[pixel_y, pixel_x] = (255, 0, 0)

        cone_count = 0
        for pixel_y in range(img.shape[0]):
            for pixel_x in range(img.shape[1]):
                # Calculate real coords
                x = (pixel_x * 0.05) + self.__map.origin[0]
                y = (pixel_y * 0.05) + self.__map.origin[1]

                # print(
                #     "checking",
                #     pose,
                #     (x, y),
                #     self.__distance(pose, (x, y)),
                #     self.__vision_range,
                # )
                if self.__within_cone(
                    pose,
                    (x, y),
                    # (y, x),
                    self.__vision_range,
                    heading,
                    self.__fov,
                ):
                    cone_count += 1
                    # img[pixel_y, pixel_x] = (255, 0, 0)
                    y_adjusted = img.shape[0] - pixel_y
                    try:
                        img[y_adjusted, pixel_x] = (255, 0, 0)
                        # img[pixel_x, y_adjusted] = (255, 0, 0)
                    except:
                        # print("except triggered")
                        pass

        # print("cone count", cone_count)
        robot_pixel_x = int((pose[0] - self.__map.origin[0]) / 0.05)
        robot_pixel_y = img.shape[0] - int((pose[1] - self.__map.origin[1]) / 0.05)
        # robot_pixel_x = int((pose[0] - self.__map.origin[1]) / 0.05)
        # robot_pixel_y = int((pose[1] - self.__map.origin[0]) / 0.05)
        # print("robot", robot_pixel_x, robot_pixel_y)
        cv2.circle(img, (robot_pixel_x, robot_pixel_y), 1, (255, 0, 0), -1)
        # cv2.circle(img, (robot_pixel_y, robot_pixel_x), 1, (255, 0, 0), -1)

        for item in self.__get_world_items():
            print(item.name, item.origin)
            # x = int((item.origin[0] - self.__map.origin[0]) / 0.05)
            # y = int((item.origin[1] - self.__map.origin[1]) / 0.05)
            x = int((item.origin[0] - self.__map.origin[0]) / 0.05)
            y = img.shape[0] - int((item.origin[1] - self.__map.origin[1]) / 0.05)
            # print(x, y)
            cv2.circle(img, (x, y), 5, (0, 255, 0), -1)
            # cv2.circle(img, (x, y), 5, (0, 255, 0), -1)

            # Draw the line
            # cells = self.__map.line((robot_pixel_x, robot_pixel_y), (x, y))
            cells = self.__map.line((robot_pixel_y, robot_pixel_x), (y, x))
            # print("Cells found", len(cells))
            for cell in cells:
                # img[cell[1], cell[0]] = (0, 255, 0)
                img[cell[0], cell[1]] = (0, 0, 255)

            # Does the robot see it?
            # if self.__vision_detection_probability(item):
            #     print("Robot sees item")
            # else:
            #     print("Robot does not see item")

        # cv2.circle(img, (robot_pixel_y, robot_pixel_x), 1, (0, 0, 255), -1)
        # cv2.circle(img, (robot_pixel_x, robot_pixel_y), 1, (0, 0, 255), -1)

        # img = self.__map.resize_img(img, size=800)

        # print("pixel counts", pixel_count, cone_count)

        seen = self.vision_check()
        if len(seen) > 0:
            for item in seen:
                print("Robot sees", item.name)
        else:
            print("ROBOT DOES NOT SEE NUTTIN")

        # cv2.imshow("Vision", img)
        # cv2.waitKey()
        # raise "stop"

        # img = self.__map.resize_img(img, size=800)
        # cv2.imshow("Vision", img)
        # cv2.waitKey()


import cv2


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
