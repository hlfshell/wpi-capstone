import rclpy
from rclpy.node import Node

import ast
from planner.llm import LLM

from typing import List, Optional, Dict, Tuple

from capstone_interfaces.msg import StateObject, Room
from capstone_interfaces.srv import PlannerQuery, GetRooms
from nav_msgs.msg import Odometry

from ament_index_python.packages import get_package_share_directory
from os import path

from threading import Lock, Thread
from concurrent.futures import Future, wait, ThreadPoolExecutor

from math import sqrt


class AI(Node):
    """
    AI is a class that utilizes an LLM to generate pythonic
    code to solve for objectives, determine the state of
    the world and its progress, and other complicated tasks
    requiring contextual processing.
    """

    def __init__(
        self,
        llm: LLM,
        planning_prompt: Optional[str] = None,
        functions_prompt: Optional[str] = None,
        rating_prompt: Optional[str] = None,
    ):
        super().__init__("ai")
        self.__llm = llm

        if planning_prompt is None or functions_prompt is None or rating_prompt is None:
            package_dir = get_package_share_directory("planner")
            prompt_dir = path.join(package_dir, "prompts")

        if planning_prompt is None:
            planning_prompt_path = path.join(prompt_dir, "planning.prompt")
            self.__planning_prompt = open(planning_prompt_path, "r").read()
        else:
            self.__planning_prompt = planning_prompt

        if functions_prompt is None:
            functions_prompt_path = path.join(prompt_dir, "functions.prompt")
            self.__functions_prompt = open(functions_prompt_path, "r").read()
        else:
            self.__functions_prompt = functions_prompt

        if rating_prompt is None:
            rating_prompt_path = path.join(prompt_dir, "rating.prompt")
            self.__rating_prompt = open(rating_prompt_path, "r").read()
        else:
            self.__rating_prompt = rating_prompt

        self.__robot_position_lock = Lock()
        self.__robot_position: Tuple[float, float] = (0.0, 0.0)
        self.__robot_position_client = self.create_subscription(
            Odometry,
            "/odom",
            self.__pose_callback,
            qos_profile=10,  # Keep last
        )

        self.__state_query = self.create_client(PlannerQuery, "/general_query")
        self.__room_query = self.create_client(GetRooms, "/get_rooms")
        self.__state_query.wait_for_service()
        self.__room_query.wait_for_service()

        self.__executor = None

    def generate_plan(self, objective: str) -> str:
        """
        Generate a plan for the given objective
        """
        state_prompt = self.generate_state_prompt(objective)
        instructions_prompt = "Remember, do not reply with anything but python code to accomplish your goal."
        objective_str = f"Your objective is to: {objective}"

        prompts = [
            self.__planning_prompt,
            self.__functions_prompt,
            state_prompt,
            instructions_prompt,
            objective_str,
        ]

        # Write out out the planning prompt to a file
        with open("./planning.prompt", "w") as f:
            f.write("\n".join(prompts))

        output = self.__llm.prompt()

        try:
            return self.__llm.clean_response(output)
        except:  # noqa
            # If we blew up there, the code was likely
            # unrunnable. We can't return an empty string,
            # so we will instead return a line of code
            # that will cause the syntax checker to fail
            return "a ="

    def generate_plans(self, objective: str, count: int = 5) -> List[str]:
        """
        generate_plans simultaneously generates count plans
        at once and returns all that succeed. Failure on any
        one is ignored for speed.
        """
        state_prompt = self.generate_state_prompt(objective)
        instructions_prompt = "Remember, do not reply with anything but python code to accomplish your goal."
        objective_str = f"Your objective is to: {objective}"

        prompts = [
            self.__planning_prompt,
            self.__functions_prompt,
            state_prompt,
            instructions_prompt,
            objective_str,
        ]

        # Write out out the planning prompt to a file
        with open("./planning.prompt", "w") as f:
            f.write("\n".join(prompts))

        # Create multiple simultaneous threads at once to
        # generate plans
        self.get_logger().info("Generating futures")
        futures: List[Future] = []
        with ThreadPoolExecutor(max_workers=count) as executor:
            for _ in range(count):
                future = executor.submit(self.__llm.prompt, prompts)
                futures.append(future)

        self.get_logger().info("Waiting for plans to generate")
        wait(futures)
        self.get_logger().info("Plans generated")
        outputs: List[str] = []
        for future in futures:
            try:
                outputs.append(self.__llm.clean_response(future.result()))
            except:  # noqa
                pass

        self.get_logger().info(f"Generated {len(outputs)} plans")
        return outputs

    def generate_state_prompt(self, objective: str) -> str:
        """
        When called, this function queries the state management
        system for all known objects. It then builds a
        comprehensive string describing the state of the world
        as we know it.
        """
        with self.__robot_position_lock:
            robot_position = self.__robot_position

        # Get the list of all known rooms
        rooms = self.__get_rooms()

        prompt = "You are aware of the following rooms:\n"
        for room in rooms:
            distance = self.__distance(robot_position, (room.x, room.y))
            prompt += f"\t{room.name} - {distance:.2f}m away\n"

        items = self.__get_related_items(objective)
        items_by_room: Dict[str, List[StateObject]] = {}
        for item in items:
            if item.location not in items_by_room:
                items_by_room[item.location] = []
            items_by_room[item.location].append(item)

        prompt += "You are aware of the following items (ID - label - distance) related to your mission:\n"
        for room, items in items_by_room.items():
            prompt += f"\t{room}:\n"
            for item in items:
                distance = self.__distance(robot_position, (item.x, item.y))
                prompt += f"\t\t{item.id} - {item.description} - {distance:.2f}m away\n"
        return prompt

    def rate_plans(
        self, objective: str, plans: List[str], raters: int = 5
    ) -> Tuple[Dict[int, int], Dict[int, List[str]]]:
        """
        rate_plans takes a list of plans and returns a list of
        integers representing the index of each plan per its
        rating. The first plan is ranked best, and so on. If
        a plan's index is not listed it is because it was
        determined invalid and should not be considered at all.
        """
        # First we eliminate plans that can not be executed
        # via syntax check
        potential_plans: List[str] = []
        for index, plan in enumerate(plans):
            try:
                self.code_check(plan)
                # If we've reached this our code is fine
                potential_plans.append(plan)
            except Exception as e:  # noqa
                pass

        # If we have only one or no plans, remaining, return
        # them.
        if len(potential_plans) == 0:
            return {}, {}
        elif len(potential_plans) == 1:
            surviving_index = plans.index(potential_plans[0])
            return {surviving_index: 5}, {
                surviving_index: ["this is the only plan that worked"]
            }

        # Compile a prompt for asking the LLM to rate the plans
        prompts = [
            self.__functions_prompt,
            self.__rating_prompt,
            f"Your objective is to: {objective}",
        ]

        for index, plan in enumerate(plans):
            if plan in potential_plans:
                prompts.append(self.__plan_to_rating_prompt(plan, index))

        prompts.append(
            "Rank the prompts according to how well they accomplish the objective and in the format requested"
        )

        # Write out out the rating prompt to a file
        with open("./rating.prompt", "w") as f:
            f.write("\n".join(prompts))

        futures: List[Future] = []
        with ThreadPoolExecutor(max_workers=raters) as executor:
            for _ in range(raters):
                future = executor.submit(self.__llm.prompt, prompts, json=True)
                futures.append(future)

        wait(futures)

        # Scores are a dict with key being the plan
        # index, and the value being its rating. Note
        # that the index is the original index, since
        # some plans may have been eliminated due to
        # outright syntax errors.
        scores: Dict[int, int] = {}
        reasons: Dict[int, List[str]] = {}
        for future in futures:
            try:
                response = future.result()
                # Convert the text response into a dict
                # for easier processing
                response = ast.literal_eval(response)
                response = response["results"]
                for item in response:
                    plan = item["plan"]
                    rating = item["score"]
                    reason = item["reason"]

                    if plan not in scores:
                        scores[plan] = 0
                        reasons[plan] = []
                    # Add in the weighted score
                    scores[plan] += rating / len(potential_plans)
                    reasons[plan].append(f"Plan: {plan}\nScore: {rating}\n{reason}")
            except:  # noqa
                pass

        return scores, reasons

    def get_best_plan(self, objective: str, plans: List[str]) -> Optional[str]:
        """
        get_best_plan takes a list of plans and returns the
        best one according to the LLM's rating system. If
        no plans are valid, None is returned.
        """
        scores, reasons = self.rate_plans(objective, plans)
        scores: Dict[int, int]
        reasons: Dict[int, List[str]]
        with open("./ratings.out", "w") as f:
            out = []
            for plan, score in scores.items():
                out.append(f"***** Plan: {plan + 1} *****")
                out.append(f"Score: {score}")
                out.append("Raters' reasoning:")
                for reason in reasons[plan]:
                    out.append(f"\t-{reason}")
            f.write("\n".join(out))

        if len(scores) == 0:
            return None

        best_plan = max(scores, key=scores.get)
        return plans[best_plan]

    def code_check(self, code: str) -> bool:
        """
        code_check returns whether or not the code as provided
        is valid python. There may still be issues with the code,
        but it will run in a python interpreter as written.
        """
        ast.parse(code)

    def __plan_to_rating_prompt(self, plan: str, index: int) -> str:
        """
        Given a code plan, wrap it with some stuff
        """
        return f"Plan {index}:\n```python\n{plan}\n```"

    def __pose_callback(self, msg: Odometry):
        """
        Updates our knowledge of where the robot is located
        in the real world
        """
        with self.__robot_position_lock:
            self.__robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def __get_rooms(self) -> List[Room]:
        """
        get_rooms returns all known rooms
        """
        request = GetRooms.Request()

        future = self.__room_query.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response: GetRooms.Response = future.result()

        return response.rooms

    def __get_related_items(self, objective: str) -> List[StateObject]:
        """
        Given an objective, return all objects that are
        related to it for our state prompts
        """
        request = PlannerQuery.Request()
        request.question = objective

        future = self.__state_query.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response: PlannerQuery.Response = future.result()

        return response.objects

    def __distance(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """
        Given two locations, return the euclidean distance between them
        """
        return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def spin(self):
        """
        Thread safe non blocking spin function
        """
        if self.__executor is not None:
            raise Exception("already spinning")

        self.__executor = rclpy.executors.MultiThreadedExecutor()
        self.__executor.add_node(self)
        Thread(target=self.__executor.spin, daemon=True).start()
