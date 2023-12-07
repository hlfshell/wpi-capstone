import rclpy
from rclpy.node import Node

from capstone_interfaces.msg import AIPrintStatement


class Printer(Node):
    def __init__(self):
        super().__init__("printer")
        self.__sub = self.create_subscription(
            AIPrintStatement,
            "/objective/ai/out",
            self.__print,
            10,
        )

    def __print(self, msg: AIPrintStatement):
        print(msg.out)


def main():
    rclpy.init()
    printer = Printer()
    rclpy.spin(printer)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
