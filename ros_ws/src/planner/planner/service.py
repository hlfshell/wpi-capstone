from threading import Lock
from typing import Dict, List, Optional, Union
from uuid import uuid4
from os import path

import rclpy
from capstone_interfaces.msg import AIPrintStatement, Objective, ObjectiveStatus, Plan
from rclpy.node import Node

from planner.ai import AI
from planner.engine import RobotEngine
from planner.llm import OpenAI, PaLM

from time import sleep
from threading import Thread

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
3. FAILED - the plan completed, but was unable to succeed at
    what it set out to do
4. REPLANNING - the plan has failed, and the AI is requesting
    a replan

"""
CREATED = "CREATED"
PLANNING = "PLANNING"
INIT = "INITIALIZING"
RUNNING = "RUNNING"
COMPLETED = "COMPLETED"
CANCELLED = "CANCELLED"
FAILED = "FAILED"
ERROR = "ERROR"
REPLAN = "REPLAN"


class Service(Node):
    """
    Service creates a planner service - it accepts an objective
    and generates the resulting plan. This plan is then
    broadcasted and fed into the RobotPlanner to create the
    resulting actions.
    """

    def __init__(self, ai: AI, engine: RobotEngine):
        super().__init__("planner")
        self.__ai = ai
        self.__engine = engine
        self.__engine.set_output_callback(self.__broadcast_output)

        self.__plan_publisher = self.create_publisher(Plan, "objective/plan", 10)
        self.__objective_status_publisher = self.create_publisher(
            ObjectiveStatus, "objective/status", 10
        )
        self.__ai_print_publisher = self.create_publisher(
            AIPrintStatement, "objective/ai/out", 10
        )

        # State memory
        self.__lock = Lock()
        self.__plan: str = ""
        self.__plan_id: str = ""
        self.__objective: str = ""
        self.__objective_id: str = ""
        self.__status: ObjectiveStatus = ObjectiveStatus()
        self.__history: Dict[str, List[ObjectiveStatus]] = {}

        self.__objective_subscription = self.create_subscription(
            Objective, "objective", self.__objective_callback, 10
        )

        self.__plan_processor = self.create_timer(0.5, self.__process_plan)
        self.__plan_executor = self.create_timer(0.5, self.__execute_plan)

        Thread(target=self.test).start()

    def test(self):
        sleep(5.0)
        with self.__lock:
            self.get_logger().info("Triggered test")
            id = "test"

            plan = open("./runme.py", "r").read()

            status = ObjectiveStatus(
                status=INIT,
                id=id,
                message="plan generated, initializing robot",
            )
            self.__objective_id = id
            self.__history[id] = [status]
            self.__status = status
            self.__plan_id = str(uuid4())
            self.__plan = plan
        self.__log_objective_status(status)
        self.__objective_status_publisher.publish(status)

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
            id=self.__objective_id,
            message="new objective received",
        )

        with self.__lock:
            self.__objective_id = msg.id
            self.__objective = msg.objective
            if self.__objective_id not in self.__history:
                self.__history[self.__objective_id] = [status]
            else:
                self.__history[self.__objective_id].append(status)
            self.__plan = ""
            self.__plan_id = ""
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
            self.__plan_id = ""

        self.__log_objective_status(status)
        self.__objective_status_publisher.publish(status)

    def __log_objective_status(self, status: ObjectiveStatus):
        """
        Log the objective status to the history
        """
        self.get_logger().info(
            f"objective {status.id} status: {status.status} - {status.message}"
        )

    def __broadcast_output(self, msg: str):
        """
        Broadcast an AI print statement
        """
        self.__ai_print_publisher.publish(AIPrintStatement(out=msg))

    def __process_plan(self):
        """
        process_plan sees if there is an objective in the CREATED
        stage. If there is, it will begin the process of generating
        a plan and passing it to the robot engine.
        """
        with self.__lock:
            if self.__objective_id == "" or (
                self.__status.status != CREATED and self.__status.status != REPLAN
            ):
                return

            status = ObjectiveStatus(
                status=PLANNING,
                id=self.__objective_id,
                message="AI is now planning what to do",
            )
            self.__history[self.__objective_id].append(status)
            self.__status = status

            objective_id = self.__objective_id
            objective = self.__objective

        self.__log_objective_status(status)
        self.__objective_status_publisher.publish(status)

        self.get_logger().info("Jumping into while")
        attempts = 0
        while True:
            try:
                plan_id = str(uuid4())
                self.get_logger().info("Generating plans")
                plans: List[str] = self.__ai.generate_plans(objective)
                self.get_logger().info("Getting best plan")
                plan = self.__ai.get_best_plan(objective, plans)
                self.get_logger().info("Got best plan!!!!!!!!!")
                break
            except Exception as e:
                raise e
                self.get_logger().error(e)
                attempts += 1
                if attempts > 3:
                    self.get_logger().error("failed to generate plan within 3 attempts")
                    # set our status and broadcast our failure
                    with self.__lock:
                        status = ObjectiveStatus(
                            status=ERROR,
                            id=self.__objective_id,
                            message="failed to generate plan after 3 attempts",
                        )
                        self.__history[self.__objective_id].append(status)
                        self.__status = status
                        self.__objective = ""
                        self.__objective_id = ""
                        self.__plan = ""
                        self.__plan_id = ""
                    self.__log_objective_status(status)
                    self.__objective_status_publisher.publish(status)
                    return

        # Set our plan and announce its creation
        self.get_logger().info("Setting plan")
        with self.__lock:
            self.__plan = plan
            self.__plan_id = plan_id
            status = ObjectiveStatus(
                status=INIT,
                id=self.__objective_id,
                message="plan generated, initializing robot",
            )
            self.__history[self.__objective_id].append(status)
            self.__status = status
            self.__plan = plan
        self.__log_objective_status(status)
        self.__objective_status_publisher.publish(status)
        self.__plan_publisher.publish(
            Plan(id=plan_id, objective=objective_id, plan=plan)
        )

    def __execute_plan(self):
        """
        __execute_plan checks to see if a plan is marked in the
        INIT status. If so, it will go ahead and attempt to execute
        the plan. Success or failure, this function will announce
        state changes appropriately.
        """
        with self.__lock:
            if self.__plan_id == "" or self.__status.status != INIT:
                return

            plan = self.__plan

            status = ObjectiveStatus(
                status=RUNNING,
                id=self.__objective_id,
                message="plan is now running",
            )
            self.__history[self.__objective_id].append(status)
            self.__status = status
            # Write the plan to a file
            with open("./plan.out", "w") as f:
                f.write(plan)

        self.__log_objective_status(status)
        self.__objective_status_publisher.publish(status)

        try:
            result: Union[None, bool] = self.__engine.run(plan)
            # Three possible results - our plan was successful (True),
            # our plan failed (False), or our plan failed for some
            # reason.
            if result is None:
                # Check to see if we were cancelled prior to declaring
                # an issue
                with self.__lock:
                    if self.__status.status == CANCELLED:
                        # For now, just abort
                        return
            elif result:
                # Success!
                with self.__lock:
                    status = ObjectiveStatus(
                        status=COMPLETED,
                        id=self.__objective_id,
                        message="plan has completed executing",
                    )
                    self.__history[self.__objective_id].append(status)
                    self.__status = status
                    self.__objective = ""
                    self.__objective_id = ""
                    self.__plan = ""
                    self.__plan_id = ""
                self.__log_objective_status(status)
                self.__objective_status_publisher.publish(status)
            else:
                # Check to see if we were cancelled prior to declaring
                # an issue
                with self.__lock:
                    if self.__status.status == CANCELLED:
                        # For now, just abort
                        return
                # If we reached this part, we have failed and must
                # attempt to replan
                with self.__lock:
                    status = ObjectiveStatus(
                        status=FAILED,
                        id=self.__objective_id,
                        message="plan failed to execute",
                    )
                    self.__history[self.__objective_id].append(status)
                    self.__status = status
                    self.__plan = ""
                    self.__plan_id = ""
                self.__log_objective_status(status)
                self.__objective_status_publisher.publish(status)
                with self.__lock:
                    status = ObjectiveStatus(
                        status=REPLAN,
                        id=self.__objective_id,
                        message="requesting replan",
                    )
                    self.__status = status
                    self.__history[self.__objective_id].append(status)
                self.__log_objective_status(status)
                self.__objective_status_publisher.publish(status)

        except Exception as e:
            raise e
            self.get_logger().error(str(e))
            with self.__lock:
                status = ObjectiveStatus(
                    status=ERROR,
                    id=self.__objective_id,
                    message="plan failed to execute",
                )
                self.__history[self.__objective_id].append(status)
                self.__status = status
                self.__objective = ""
                self.__objective_id = ""
                self.__plan = ""
                self.__plan_id = ""
            self.__log_objective_status(status)
            self.__objective_status_publisher.publish(status)
            return


def main(args=None):
    rclpy.init(args=args)

    # llm = OpenAI(model="gpt-4-1106-preview")
    llm = OpenAI()
    # llm = PaLM()
    ai = AI(llm)

    engine = RobotEngine()

    ai.spin()
    engine.spin()

    service = Service(ai, engine)

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
