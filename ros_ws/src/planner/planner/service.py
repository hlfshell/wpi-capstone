import rclpy
from rclpy.node import Node

from planner.llm import LLM
from planner.engine import RobotEngine

from capstone_interfaces.msg import Objective, Plan, ObjectiveStatus

from threading import Lock
from typing import Optional, Dict, List


# Statuses
"""
The expected trajectory of an objective is as follows:

1. CREATED - the objective has been reported and is waiting
        to be processed.
2. PLANNING - the plan is being generated by an LLM
3. INITIALIZING - the plan is being passed to the robot engine
        and being spot checked for errors. This should be quick
4. RUNNING - the plan is being executed by the robot engine
    and the robot in the real world
5. COMPLETED - the plan has been completed successfully without
    errors. Note that this does not mean that the objective
    has been completed, but rather that the plan has been ran.

Additional states can be:
1. CANCELLED - the plan has been cancelled by some element
    of the system.
2. ERROR - the plan has encountered an error and cannot be
    completed.

"""
CREATED = "CREATED"
PLANNING = "PLANNING"
INIT = "INITIALIZING"
RUNNING = "RUNNING"
COMPLETED = "COMPLETED"
CANCELLED = "CANCELLED"
ERROR = "ERROR"


class Service(Node):
    """
    Service creates a planner service - it accepts an objective
    and generates the resulting plan. This plan is then
    broadcasted and fed into the RobotPlanner to create the
    resulting actions.
    """

    def __init__(self, llm: LLM, engine: RobotEngine):
        super().__init__("planner")
        self.__llm = llm
        self.__engine = engine

        self.__plan_publisher = self.create_publisher(Plan, "plan", 10)
        self.__objective_status_publisher = self.create_publisher(
            Objective, "objective/status", 10
        )

        # State memory
        self.__lock = Lock()
        self.__plan: str = ""
        self.__objective: str = ""
        self.__objective_id: str = ""
        self.__status: ObjectiveStatus = ObjectiveStatus()
        self.__history: Dict[str, List[ObjectiveStatus]] = {}

        self.__objective_subscription = self.create_subscription(
            Objective, "objective", self.__objective_callback, 10
        )

        self.__plan_processor = self.create_timer(0.5, self.__process_plan)

    def __objective_callback(self, msg: Objective):
        """
        Assign a new objective, cancelling existing ones first
        """

        # First, if there is an existing objective, we need to
        # cancel the plan
        with self.__lock:
            cancel = self.__objective_id != ""

        if cancel:
            self.cancel()

        status = ObjectiveStatus(
            status=CREATED,
            plan_id=self.__objective_id,
            message="new objective received",
        )

        with self.__lock:
            self.__objective_id = msg.id
            self.__objective = msg.objective
            self.__history[self.__objective_id] = [status]
            self.__plan = ""
            self.__status = status

        self.__log_objective_status(status)
        self.__objective_status_publisher.publish(status)

    def cancel(self, msg: Optional[str] = None):
        """
        cancel will pass a cancellation signal to the robot engine
        and announce the cancellation publicly, if an objective
        is running.
        """
        if msg is None:
            msg = "cancellation requested"

        status = ObjectiveStatus(
            status=CANCELLED, plan_id=self.__objective_id, message=msg
        )
        self.__engine.cancel()

        with self.__lock:
            self.__history[self.__objective_id].append(status)
            self.__status = status
            self.__objective = ""
            self.__objective_id = ""
            self.__plan = ""

        self.__log_objective_status(status)
        self.__objective_status_publisher.publish(status)

    def __log_objective_status(self, status: ObjectiveStatus):
        """
        Log the objective status to the history
        """
        self.get_logger().info(
            f"objective {status.plan_id} status: {status.status} - {status.message}"
        )

    def __process_plan(self):
        """
        process_plan sees if there is an objective in the CREATED
        stage. If there is, it will begin the process of generating
        a plan and passing it to the robot engine.
        """
        with self.__lock:
            if self.__objective_id == "" or self.__status.status != CREATED:
                return

            status = ObjectiveStatus(
                status=PLANNING,
                plan_id=self.__objective_id,
                message="AI is now planning what to do",
            )
            self.__history[self.__objective_id].append(status)
            self.__status = status

        self.__log_objective_status(status)
        self.__objective_status_publisher.publish(status)


def main(args=None):
    rclpy.init(args=args)

    llm = LLM()
    service = Service(llm)

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
