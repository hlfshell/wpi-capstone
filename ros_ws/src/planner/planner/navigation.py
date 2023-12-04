from rclpy.node import Node
from threading import Lock

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future as ActionTaskFuture
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry

from typing import Optional, Callable, Tuple

import math
from time import sleep


class NavigationModule(Node):
    """
    NavigationModule handles generic navigation tasks for robot,
    as well as tracking current position to judge the success of those
    movements.
    """

    def __init__(self, distance_for_success: float = 0.5):
        super().__init__("navigation_module")

        self.__distance_for_success = distance_for_success

        self.__navigate = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.__navigate.wait_for_server()

        self.__goal_position_lock = Lock()
        self.__goal_position: Optional[Tuple[float, float]] = None

        self.__goal_handle_lock = Lock()
        self.__goal_handle: Optional[ClientGoalHandle] = None

        self.__result_callback_lock = Lock()
        self.__result_callback: Optional[Callable[[bool, str]]] = None

        self.__odometry_subscriber = self.create_subscription(
            msg_type=Odometry,
            topic="/odom",
            callback=self.__pose_callback,
            qos_profile=10,  # Keep last
        )
        self.__position_lock = Lock()
        self.__position: Tuple[float, float] = (0.0, 0.0)

        self.__sync_lock = Lock()
        self.__sync_complete: bool = False
        self.__last_result: Tuple[bool, str] = (False, "")

    def cancel(self):
        """
        cancel will, if possible, cancel the current goal if one exists. If
        one doesn't, it simply returns
        """
        with self.__goal_handle_lock:
            if self.__goal_handle is None:
                return
            else:
                self.__goal_handle.cancel_goal_async()

    def get_current_position(self) -> Tuple[float, float]:
        """
        get_current_position returns the last known position
        of the robot.
        """
        with self.__position_lock:
            return self.__position

    def move_to(
        self,
        location: Tuple[float, float],
        result_callback: Callable,
        distance_for_success: float = 0.5,
    ):
        """
        move_to will move attempt to move the robot to a given x,y location.
        The result_callback is a function with parameters (success, message)
        where success is a boolean and message is a string that explains failure
        if one exists. A success is considered being within a set distance of
        the goal (the navigation module's distance_for_success parameter). Note
        that a cancellation is considered a failure.
        """
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = location[0]
        goal.pose.pose.position.y = location[1]
        self.__distance_for_success = distance_for_success

        with self.__goal_position_lock:
            self.__goal_position = location
        with self.__result_callback_lock:
            self.__result_callback = result_callback

        future: ActionTaskFuture = self.__navigate.send_goal_async(goal)
        future.add_done_callback(self.__navigate_acceptance_callback)

    def move_to_synchronous(
        self,
        location: Tuple[float, float],
        distance_for_success: float = 0.5,
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

        self.move_to(location, lambda result: None, distance_for_success)

        while True:
            with self.__sync_lock:
                if self.__sync_complete:
                    self.__sync_complete = False
                    return self.__last_result
            sleep(0.1)

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
        current_position = self.get_current_position()

        with self.__goal_position_lock:
            goal_position = self.__goal_position

        distance = self.__distance(current_position, goal_position)

        result = distance < self.__distance_for_success

        # We do assignment prior to calling to try to prevent long
        # running locks
        with self.__result_callback_lock:
            callback = self.__result_callback

        result = (result, "Success" if result else f"{distance:.2f} away from goal")

        self.__last_result = result
        with self.__sync_lock:
            self.__sync_complete = True
        callback(result)

    def __pose_callback(self, msg: Odometry):
        """
        __pose_callback updates our current position of the robot
        """
        with self.__position_lock:
            self.__position = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
            )

    def __distance(self, a: Tuple[float, float], b: [float, float]) -> float:
        """
        __distance calculates the distance between two points
        """
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
