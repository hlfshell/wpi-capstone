import rclpy
from rclpy.node import Node
from threading import Lock, Thread
from rclpy.action import ActionClient

from typing import Optional, Tuple

from explorer.search import Explorer

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import cv2
from time import sleep

# from geometry_msgs.srv import GetCurrentPose

# from nav2_util import getCurrentPose


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
        # self.__goal_pose_client = ActionClient(self, NavigateToPose, "NavigateToPose")
        # self.__last_goal: Optional[Tuple(float, float)] = None
        # self.__last_action = None

        self.__explore_thread = Thread(target=self.explore)
        self.__explore_thread.start()

        self.index = 0

    def __map_callback(self, msg: OccupancyGrid):
        """
        Callback for /map topic. Updates the latest map
        """
        with self.__map_lock:
            # print("Got map")
            self.__latest_map = msg

        # self.get_next_goal()

    def __pose_callback(self, msg: Odometry):
        """
        Callback for /robot_pose topic. Updates the latest pose
        """
        with self.__pose_lock:
            pose = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
            )
            # print("Got pose", pose)
            self.__latest_pose = pose
        # self.get_next_goal()

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
        # goal.pose.orientation.w = 1.0

        self.__goal_publisher.publish(pose)

    # def __send_goal(self, goal: Tuple[float, float]):
    #     """
    #     Sends a goal to the robot
    #     """
    #     # Convert our (x,y) coordinates to a PoseStamped
    #     pose = PoseStamped()
    #     pose.header.stamp = self.get_clock().now().to_msg()
    #     pose.header.frame_id = "map"
    #     pose.pose.position.x = goal[0]
    #     pose.pose.position.y = goal[1]
    #     pose.pose.orientation.w = 0.0
    #     # goal.pose.orientation.w = 1.0

    #     goal_message = NavigateToPose.Goal()
    #     goal_message.pose = pose

    #     future = self.__goal_pose_client.send_goal_async(goal_message)
    #     self.__last_goal = goal
    #     self.__last_action = future

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

        # save the map to a pickle
        # with open("map.pickle", "wb") as f:
        #     pickle.dump(map, f)

        # raise "done"

        explorer = Explorer(map, pose, debug=True)
        goal = explorer.explore()

        img = explorer.get_debug_map_img(size=800)
        cv2.imwrite(f"logs/debug_map_{self.index}.png", img)
        # explorer.show_map(size=800)
        # explorer.show_debug_map(size=800)

        return goal

    def explore(self):
        self.index = 0
        while True:
            # Delay a bit between checks
            sleep(3)

            with self.__pose_lock:
                pose = self.__latest_pose
            with self.__map_lock:
                map = self.__latest_map
            if pose is None or map is None:
                continue

            # If we have an active action, kill it
            # if self.__last_action is not None:
            #     print("cancelling goal...")
            #     self.__goal_pose_client._cancel_goal()
            #     print("cancelled")
            #     break

            goal = self.get_next_goal()
            if goal is None:
                print("Complete!")
                break

            print("Sending to", goal)
            self.__send_goal(goal)

    # def test(self):
    #     # load the file from the pickle
    #     map = None
    #     with open("map.pickle", "rb") as f:
    #         map = pickle.load(f)

    #     # What is map?
    #     print(map)
    #     print(isinstance(map, OccupancyGrid))
    #     origin = (-1.7001501162482318, -0.46967678386795436)
    #     ex = Explorer(map, origin, debug=True)
    #     ex.explore()
    #     ex.show_map(size=800)
    #     ex.show_debug_map(size=800)
