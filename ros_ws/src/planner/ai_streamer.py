import rclpy
from rclpy.node import Node

from capstone_interfaces.msg import (
    AIPrintStatement,
    ObjectiveStatus,
    Objective,
    RobotAction,
)


class Printer(Node):
    def __init__(self):
        super().__init__("printer")
        self.__ai_out_sub = self.create_subscription(
            AIPrintStatement,
            "/objective/ai/out",
            self.__ai_out,
            10,
        )

        self.__objective_status_sub = self.create_subscription(
            ObjectiveStatus,
            "/objective/status",
            self.__objective_status,
            10,
        )

        self.__robot_action_sub = self.create_subscription(
            RobotAction,
            "/objective/action",
            self.__robot_action,
            10,
        )

        self.__objective_sub = self.create_subscription(
            Objective,
            "/objective",
            self.__objective,
            10,
        )

    def __ai_out(self, msg: AIPrintStatement):
        out = msg.out.strip()
        if out != "":
            print(f"[AI] {out}")

    def __objective_status(self, msg: ObjectiveStatus):
        print(f"[Objective] {msg.id} - {msg.status}")

    def __robot_action(self, msg: RobotAction):
        print(f"[Robot] {msg.function}({msg.parameters})")

    def __objective(self, msg: Objective):
        print(f"***** [New Objective] {msg.id}: {msg.objective} *****")


def main():
    rclpy.init()
    printer = Printer()
    rclpy.spin(printer)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
