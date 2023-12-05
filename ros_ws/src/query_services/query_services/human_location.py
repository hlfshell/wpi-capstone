import rclpy
from rclpy.node import Node

from capstone_interfaces.srv import QueryHumanLocation
from capstone_interfaces.msg import HumanPose, HumanSpotted

from typing import Tuple, List

from threading import Lock


class HumanService(Node):
    """
    HumanService tracks the last known location of our human user
    user and informs other services of where they might be.
    """

    def __init__(self):
        super().__init__("human_service")

        self.__human_position_lock = Lock()
        self.__human_position: Tuple[float, float] = (0.0, 0.0)
        self.__human_last_seen_list: List[Tuple[float, float]] = []

        self.__human_spotted_subscription = self.create_subscription(
            HumanSpotted,
            "/human_spotted",
            self.__human_spotted_callback,
            qos_profile=10,  # Keep last
        )

        self.__human_query_service = self.create_service(
            QueryHumanLocation, "/human_location", self.__human_query_callback
        )

    def __human_spotted_callback(self, msg: HumanSpotted):
        with self.__human_position_lock:
            location = (msg.x, msg.y)
            self.__human_last_seen_list.append(location)
            if len(self.__human_last_seen_list) > 100:
                self.__human_last_seen_list = self.__human_last_seen_list[-100:]

            # The location is the average of all in the list
            self.__human_position = (
                sum([x[0] for x in self.__human_last_seen_list])
                / len(self.__human_last_seen_list),
                sum([x[1] for x in self.__human_last_seen_list])
                / len(self.__human_last_seen_list),
            )

    def __human_query_callback(
        self, request: QueryHumanLocation.Request, response: QueryHumanLocation.Response
    ):
        with self.__human_position_lock:
            response.pose.x = self.__human_position[0]
            response.pose.y = self.__human_position[1]
        return response


def main(args=None):
    rclpy.init(args=args)

    node = HumanService()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
