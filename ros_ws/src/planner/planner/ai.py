import ast
from planner.llm import LLM

from typing import List, Optional, Dict

from capstone_interfaces.msg import StateObject

# from capstone_interfaces.srv import GetAllObjects

# Get package directory for file loading ROS2
from ament_index_python.packages import get_package_share_directory
from os import path

from threading import Thread, Lock
from concurrent.futures import Future, wait, ThreadPoolExecutor


class AI:
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

    def generate_plan(self, objective: str) -> str:
        """
        Generate a plan for the given objective
        """
        state_prompt = self.generate_state_prompt()
        instructions_prompt = "Remember, do not reply with anything but python code to accomplish your goal."
        objective_str = f"Your objective is to: {objective}"

        output = self.__llm.prompt(
            [
                self.__planning_prompt,
                self.__functions_prompt,
                state_prompt,
                instructions_prompt,
                objective_str,
            ]
        )

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
        state_prompt = self.generate_state_prompt()
        instructions_prompt = "Remember, do not reply with anything but python code to accomplish your goal."
        objective_str = f"Your objective is to: {objective}"

        prompts = [
            self.__planning_prompt,
            self.__functions_prompt,
            state_prompt,
            instructions_prompt,
            self.generate_state_prompt(),
            objective_str,
        ]

        # Create multiple simultaneous threads at once to
        # generate plans
        futures: List[Future] = []
        with ThreadPoolExecutor(max_workers=count) as executor:
            for _ in range(count):
                future = executor.submit(self.__llm.prompt, prompts)
                futures.append(future)

        wait(futures)
        outputs: List[str] = []
        for future in futures:
            try:
                outputs.append(self.__llm.clean_response(future.result()))
            except:  # noqa
                pass

        return outputs

    def generate_state_prompt(self) -> str:
        """
        When called, this function queries the state management
        system for all known objects. It then builds a
        comprehensive string describing the state of the world
        as we know it.
        """
        return """
You know about the following rooms and objects. The objects are presented in the format of id# - label

kitchen:
    1 - apple
    22 - coke_can
    31 - orange
    12 - wine
    9 - book
living_room:
    14 - tv
    21 - book
    19 - coke_can
    11 - remote_control
bathroom:
    13 - toothbrush
    27 - medicine
    3 - hairbrush
    """

    def rate_plans(
        self, objective: str, plans: List[str], raters: int = 5
    ) -> List[int]:
        """
        rate_plans takes a list of plans and returns a list of
        integers representing the index of each plan per its
        rating. The first plan is ranked best, and so on. If
        a plan's index is not listed it is because it was
        determined invalid and should not be considered at all.
        """
        rating: List[int] = []

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
            return []
        elif len(potential_plans) == 1:
            return [
                {
                    "plan": plans.index(potential_plans[0]),
                    "rating": 5,
                    "reason": "Only one that worked",
                }
            ]

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
        scores, _ = self.rate_plans(objective, plans)

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

    def __get_all_objects(self) -> List[StateObject]:
        pass
