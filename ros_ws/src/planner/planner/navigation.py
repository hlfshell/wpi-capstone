from rclpy.node import Node
from threading import Lock

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future as ActionTaskFuture
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from ament_index_python.packages import get_package_share_directory
import netpbmfile
from os import path
from planner.map import Map

from typing import Optional, Callable, Tuple, Union

from math import asin, sin, cos, atan2, sqrt, pi
from time import sleep, time


class NavigationModule(Node):
    """
    NavigationModule handles generic navigation tasks for robot,
    as well as tracking current position to judge the success of those
    movements.
    """

    def __init__(self):
        super().__init__("navigation_module")

        self.__distance_for_success = 1.0
        self.__acceptable_angle_difference = pi / 8

        self.__navigate = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.__navigate.wait_for_server()

        self.__goal_pose_lock = Lock()
        self.__goal_position: Optional[Tuple[float, float]] = None
        self.__goal_orientation: Optional[Tuple[float, float, float, float]] = None

        self.__goal_handle_lock = Lock()
        self.__goal_handle: Optional[ClientGoalHandle] = None
        self.__cancelled: bool = False

        self.__result_callback_lock = Lock()
        self.__result_callback: Optional[Callable[[bool, str]]] = None

        maps_dir = path.join(get_package_share_directory("planner"), "maps")
        self.__map = Map.FromMapFile(path.join(maps_dir, "house"))

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

        self.__sync_lock = Lock()
        self.__sync_complete: bool = False
        self.__last_result: Tuple[bool, str] = (False, "")

        self.__navigator = BasicNavigator()

    def cancel(self):
        """
        cancel will, if possible, cancel the current goal if one exists. If
        one doesn't, it simply returns
        """
        with self.__goal_handle_lock:
            self.__cancelled = True
            if self.__goal_handle is None:
                return
            else:
                self.__goal_handle.cancel_goal_async()

    def get_current_pose(
        self,
    ) -> Tuple[Tuple[float, float], Tuple[float, float, float, float]]:
        """
        get_current_position returns the last known position
        of the robot. Will lock until the first odom msg is
        received.
        """
        while True:
            with self.__pose_lock:
                if self.__first_pose_received:
                    break
            sleep(0.1)
        with self.__pose_lock:
            return self.__position, self.__orientation

    def mover(
        self,
        location: Tuple[float, float],
        distance_for_success: float = 0.75,
    ):
        if len(location) > 2:
            location = (location[0], location[1])
        spot = self.__map.closest_known_point(location, distance_for_success)
        if len(spot) > 2:
            self.get_logger().info(f"Errored {spot}")
            raise "TODO"

        return self.move_to_synchronous(spot, distance_for_success)

    def mover2(
        self,
        location: Tuple[float, float],
        distance_for_success: float = 0.75,
    ) -> Tuple[bool, str]:
        if len(location) > 2:
            location = (location[0], location[1])
        spot = self.__map.closest_known_point(location, distance_for_success)
        if len(spot) > 2:
            self.get_logger().info(f"Errored {spot}")
            raise "TODO"

        # hacky hacky hacky
        lock = Lock()
        self.__end = False
        with self.__goal_handle_lock:
            self.__cancelled = False

        def callback(msg):
            with lock:
                self.__end = True

        # end hacky

        self.move_to(spot, callback)

        while True:
            with lock:
                if self.__end:
                    break
            with self.__goal_handle_lock:
                if self.__cancelled:
                    break

            position, _ = self.get_current_pose()

            distance = self.distance(location, position)
            if distance < distance_for_success:
                self.cancel()
                return (True, "")

            sleep(0.25)

        position, _ = self.get_current_pose()
        distance = self.distance(location, position)

        # If we've reached this point we aren't close enough
        # to succeed
        return (False, f"{distance:.2f}m away from goal")

    def move_to(
        self,
        location: Tuple[float, float],
        result_callback: Optional[Callable],
        distance_for_success: float = 0.75,
        acceptable_angle_difference: float = pi / 8,
        orientation: Optional[
            Union[Tuple[float, float, float, float], Tuple[float, float, float]]
        ] = None,
    ):
        """
        move_to will move attempt to move the robot to a given x,y location.
        The result_callback is a function with parameters (success, message)
        where success is a boolean and message is a string that explains failure
        if one exists. A success is considered being within a set distance of
        the goal (the navigation module's distance_for_success parameter). Note
        that a cancellation is considered a failure.
        """
        self.get_logger().info(f"move_to: {location}")
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = location[0]
        goal.pose.pose.position.y = location[1]

        if orientation is not None:
            if len(orientation) == 4:
                goal.pose.pose.orientation.x = orientation[0]
                goal.pose.pose.orientation.y = orientation[1]
                goal.pose.pose.orientation.z = orientation[2]
                goal.pose.pose.orientation.w = orientation[3]
            elif len(orientation) == 3:
                x, y, z, w = self.__euler_to_quaternion(*orientation)
                goal.pose.pose.orientation.x = x
                goal.pose.pose.orientation.y = y
                goal.pose.pose.orientation.z = z
                goal.pose.pose.orientation.w = w
            else:
                raise ValueError("orientation must be length 3 or 4")

        self.__distance_for_success = distance_for_success
        self.__acceptable_angle_difference = acceptable_angle_difference

        with self.__goal_pose_lock:
            self.__goal_position = location
            self.__goal_orientation = orientation
        with self.__result_callback_lock:
            self.__result_callback = result_callback

        future: ActionTaskFuture = self.__navigate.send_goal_async(goal)
        future.add_done_callback(self.__navigate_acceptance_callback)

    def move_to_synchronous(
        self,
        location: Tuple[float, float],
        distance_for_success: float = 0.75,
        orientation: Optional[
            Union[Tuple[float, float, float, float], Tuple[float, float, float]]
        ] = None,
    ) -> Tuple[bool, str]:
        """
        move_to_synchronous will move the robot to a given x,y location
        and return whether or not it succeeded. A success is considered
        being within a set distance of the goal (the navigation module's
        distance_for_success parameter). Note that a cancellation is
        considered a failure.
        """
        with self.__sync_lock:
            self.sync_complete = False

        self.move_to(
            location, lambda result: None, distance_for_success, orientation=orientation
        )

        while True:
            with self.__sync_lock:
                if self.__sync_complete:
                    self.__sync_complete = False
                    return self.__last_result
            sleep(0.1)

    def spin(
        self,
        rotation: float,
        result_callback: Callable,
        angle_difference_acceptable: float = pi / 8,
    ) -> Tuple[bool, str]:
        """
        spin will spin the robot "in place" (mostly) by given radians.
        """
        # Reset the pose lock and then query for the current
        # pose - we do this because a spin requires its current
        # pose accurately, and odom updates at an unknown tick
        with self.__pose_lock:
            self.__first_pose_received = False
        current_position, current_orientation = self.get_current_pose()

        self.__distance_for_success = 0.25
        self.__acceptable_angle_difference = angle_difference_acceptable

        roll, pitch, yaw = self.__quaternion_to_euler(*current_orientation)
        yaw += rotation
        x, y, z, w = self.__euler_to_quaternion(roll, pitch, yaw)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = current_position[0]
        goal.pose.pose.position.y = current_position[1]
        goal.pose.pose.orientation.x = x
        goal.pose.pose.orientation.y = y
        goal.pose.pose.orientation.z = z
        goal.pose.pose.orientation.w = w

        with self.__goal_pose_lock:
            self.__goal_position = current_position
            self.__goal_orientation = (x, y, z, w)

        with self.__result_callback_lock:
            self.__result_callback = result_callback

        future: ActionTaskFuture = self.__navigate.send_goal_async(goal)
        future.add_done_callback(self.__navigate_acceptance_callback)

    def spin_synchronous(
        self,
        rotation: float,
        angle_difference_acceptable: float = pi / 8,
    ) -> Tuple[bool, str]:
        """
        spin_synchronous will spin the robot "in place" (mostly) by
        given radians and return whether or not it succeeded.
        """
        with self.__sync_lock:
            self.sync_complete = False

        def callback(msg):
            with self.__sync_lock:
                self.__sync_complete = True
                with self.__goal_pose_lock:
                    self.__goal_position = None
                    self.__goal_orientation = None

        self.spin(rotation, callback, angle_difference_acceptable)

        while True:
            with self.__sync_lock:
                if self.__sync_complete:
                    self.__sync_complete = False
                    return self.__last_result
            sleep(0.1)

    def spinner(self, rotation: float, angle_difference_acceptable: float = pi / 8):
        """
        spinner will spin the robot "in place" (mostly) by given radians.
        """
        self.__navigator.spin(rotation)

    def __navigate_acceptance_callback(self, future: ActionTaskFuture):
        """
        __navigate_acceptance_callback is called when the robot accepts
        or rejects the given goal.
        """
        goal_handle: ClientGoalHandle = future.result()
        with self.__goal_handle_lock:
            self.__goal_handle = goal_handle

            if not goal_handle.accepted:
                with self.__result_callback_lock:
                    callback = self.__result_callback
                callback(False, "Robot rejected navigation action")
                return

            future: ActionTaskFuture = goal_handle.get_result_async()
            future.add_done_callback(self.__navigate_result_callback)

    def __navigate_result_callback(self, future: ActionTaskFuture):
        """
        __navigate_result_callback is called when the robot has finished
        with the given task, success or fail
        """
        current_position, current_pose = self.get_current_pose()

        with self.__goal_pose_lock:
            goal_position = self.__goal_position
            goal_orientation = self.__goal_orientation

        if goal_position is None:
            # There seems to be an errant race condition where goal
            # position can be None due to a cancellation or double
            # completion. In any event, we abort here if that's the
            # case and assume we're ok
            with self.__result_callback_lock:
                callback = self.__result_callback
            return

        distance = self.distance(current_position, goal_position)

        result = distance < self.__distance_for_success

        if goal_orientation is not None and result:
            _, _, yaw = self.__quaternion_to_euler(*goal_orientation)
            _, _, current_yaw = self.__quaternion_to_euler(*current_pose)
            yaw_diff = abs(yaw - current_yaw)
            if yaw > pi:
                yaw_diff = 2 * pi - yaw_diff

            if yaw_diff > self.__acceptable_angle_difference:
                result = False

        # We do assignment prior to calling to try to prevent long
        # running locks
        with self.__result_callback_lock:
            callback = self.__result_callback

        if goal_orientation is None and not result:
            msg = "Success" if result else f"{distance:.2f} away from goal"
        elif not result:
            msg = (
                "Success"
                if result
                else f"{distance:.2f} away from goal and {yaw_diff:.2f} away from orientation"
            )
        result = (result, "Success" if result else msg)

        self.__last_result = result
        with self.__sync_lock:
            self.__sync_complete = True
            with self.__goal_pose_lock:
                self.__goal_position = None
                self.__goal_orientation = None
        callback(result)

    def __pose_callback(self, msg: Odometry):
        """
        __pose_callback updates our current position of the robot
        """
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

    def distance_to_robot(self, location: Tuple[float, float]) -> float:
        """
        distance_to_robot calculates the distance between the robot
        and the given location
        """
        current_position, _ = self.get_current_pose()
        return self.distance(current_position, location)

    def distance(self, a: Tuple[float, float], b: [float, float]) -> float:
        """
        __distance calculates the distance between two points
        """
        return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def __quaternion_to_euler(
        self, x: float, y: float, z: float, w: float
    ) -> Tuple[float, float, float]:
        """
        __quaternion_to_euler converts a quaternion to
        euler angles (in radians); returns a tuple of
        (phi, theta, psi)
        """
        phi = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))

        theta = 2.0 * (w * y - z * x)
        theta = 1.0 if theta > 1.0 else theta
        theta = -1.0 if theta < -1.0 else theta
        theta = asin(theta)

        psi = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        return phi, theta, psi

    def __euler_to_quaternion(
        self, phi: float, theta: float, psi: float
    ) -> Tuple[float, float, float, float]:
        """
        __euler_to_quaternion converts euler angles (in radians)
        to a quaternion; returns a tuple of (x, y, z, w)
        """
        x = sin(phi / 2) * cos(theta / 2) * cos(psi / 2) - cos(phi / 2) * sin(
            theta / 2
        ) * sin(psi / 2)
        y = cos(phi / 2) * sin(theta / 2) * cos(psi / 2) + sin(phi / 2) * cos(
            theta / 2
        ) * sin(psi / 2)
        z = cos(phi / 2) * cos(theta / 2) * sin(psi / 2) - sin(phi / 2) * sin(
            theta / 2
        ) * cos(psi / 2)
        w = cos(phi / 2) * cos(theta / 2) * cos(psi / 2) + sin(phi / 2) * sin(
            theta / 2
        ) * sin(psi / 2)

        return x, y, z, w
