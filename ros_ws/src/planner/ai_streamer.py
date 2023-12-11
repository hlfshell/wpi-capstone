import os
import rclpy
from rclpy.node import Node

from elevenlabs import generate, play, set_api_key
from random import choice

from capstone_interfaces.msg import (
    AIPrintStatement,
    ObjectiveStatus,
    Objective,
    RobotAction,
)


class Printer(Node):
    def __init__(self):
        super().__init__("printer")

        if "ELEVENLABS_API_KEY" in os.environ:
            set_api_key(os.environ["ELEVENLABS_API_KEY"])
            self.__enable_voice = True
        else:
            self.__enable_voice = False

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
            self.__voice(out)

    def __objective_status(self, msg: ObjectiveStatus):
        print(f"[Objective] {msg.id} - {msg.status}")

    def __robot_action(self, msg: RobotAction):
        print(f"[Robot] {msg.function}({msg.parameters})")

    def __objective(self, msg: Objective):
        print(f"***** [New Objective] {msg.id}: {msg.objective} *****")
        phrases = [
            "Okay, let's go!",
            "Sounds good. Let me create a plan.",
            "Cool - let's get on that.",
            "Beep boop, let's do it.",
            "Alright, let's see what I can do.",
        ]
        phrase = choice(phrases)
        self.__voice(f"New objective received: {msg.objective}. {phrase}")

    def __voice(self, out: str):
        if not self.__enable_voice:
            return
        try:
            if len(out) > 100:
                return
            # Replace "_" with " "
            out = out.replace("_", " ")
            audio = generate(
                text=out,
                voice="fuTxl7jIp6JGHlH2L9DW",
                model="eleven_monolingual_v1",
            )
            play(audio)
        except:
            pass


def main():
    rclpy.init()
    printer = Printer()
    rclpy.spin(printer)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
