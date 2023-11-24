from threading import Lock, Thread
from time import sleep
from typing import Optional, Tuple

import cv2
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node

from explorer.search import Explorer


class SearchService(Node):
    """
    SearchService's goal is to explore a new area until
    it is sufficiently covered by mapping. It does this
    using the Explorer node, feeding it current
    OccupancyGrid messages and the robot's location,
    then commanding the robot to move to a set position
    as defined by the Explorer node.

    It subscribes to /map and /robot_pose, publishing to
    /goal_pose
    """

    def __init__(self):
        super().__init__("search_service")

        # Track the latest robot pose
        self.__pose_lock = Lock()
        self.__latest_pose: Optional[PoseStamped] = None
        self.__odometry_subscriber = self.create_subscription(
            msg_type=Odometry,
            topic="/odom",
            callback=self.__pose_callback,
            qos_profile=10,  # Keep last
        )

        # Track the latest OccupancyGrid
        self.__map_lock = Lock()
        self.__latest_map: Optional[OccupancyGrid] = None
        self.__map_subscriber = self.create_subscription(
            msg_type=OccupancyGrid,
            topic="/map",
            callback=self.__map_callback,
            qos_profile=10,  # Keep last
        )

        # Publisher for goal poses
        self.__goal_publisher = self.create_publisher(
            msg_type=PoseStamped,
            topic="/goal_pose",
            qos_profile=10,  # Keep last
        )

        self.__ignore_list: Dict[Tuple[float, float], int] = {}
        self.__ignore_at_n_attempts = 5
        self.__ignore_list_lock = Lock()

        self.index = 0

        self.__explore_thread = Thread(target=self.explore)
        self.__explore_thread.start()

    def __map_callback(self, msg: OccupancyGrid):
        """
        Callback for /map topic. Updates the latest map
        """
        with self.__map_lock:
            self.__latest_map = msg

    def __pose_callback(self, msg: Odometry):
        """
        Callback for /robot_pose topic. Updates the latest pose
        """
        with self.__pose_lock:
            pose = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
            )
            self.__latest_pose = pose

    def __send_goal(self, goal: Tuple[float, float]):
        """
        Sends a goal to the robot
        """
        # Convert our (x,y) coordinates to a PoseStamped
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = goal[0]
        pose.pose.position.y = goal[1]
        pose.pose.orientation.w = 0.0

        self.__goal_publisher.publish(pose)

    def __generate_ignore_list(self) -> [Tuple[float, float]]:
        """
        Generates a list of coordinates to ignore based on
        repeated failed attempts to reach.
        """
        with self.__ignore_list_lock:
            ignore_list = [
                k
                for k, v in self.__ignore_list.items()
                if v >= self.__ignore_at_n_attempts
            ]

            return ignore_list

    def get_next_goal(self) -> Optional[PoseStamped]:
        """
        Returns the next goal pose for the robot to move to,
        or None if there is no goal
        """
        self.index += 1

        with self.__pose_lock:
            pose = self.__latest_pose
        with self.__map_lock:
            map = self.__latest_map

        if pose is None or map is None:
            if pose is None:
                print("No pose!")
            else:
                print("No map!")
            return None

        ignore_list = self.__generate_ignore_list()

        explorer = Explorer(map, pose, ignore_list=ignore_list, debug=True)
        goal = explorer.explore()

        img = explorer.generate_debug_map_image(size=800)
        cv2.imwrite(f"logs/debug_map_{self.index}.png", img)

        return goal

    def explore(self):
        self.index = 0
        while True:
            with self.__pose_lock:
                pose = self.__latest_pose
            with self.__map_lock:
                map = self.__latest_map
            if pose is None or map is None:
                continue

            # Delay a bit between checks
            sleep(5)

            goal = self.get_next_goal()
            if goal is None:
                print("Complete!")
                break

            if goal in self.__ignore_list:
                self.__ignore_list[goal] += 1
            else:
                self.__ignore_list[goal] = 1

            print("Sending to", goal)
            self.__send_goal(goal)


def main(args=None):
    rclpy.init(args=args)

    node = SearchService()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
