from planner.llm import LLM

from typing import List, Optional, Dict

from capstone_interfaces.msg import StateObject
from capstone_interfaces.srv import GetAllObjects

# Get package directory for file loading ROS2
from ament_index_python.packages import get_package_share_directory
from os import path

PACKAGE_DIR = get_package_share_directory("planner")
PROMPT_DIR = path.join(PACKAGE_DIR, "prompts")


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
    ):
        self.__llm = llm

        if planning_prompt is None:
            planning_prompt_path = path.join(PROMPT_DIR, "planning.prompt")
            self.__planning_prompt = open(planning_prompt_path, "r").read()
        else:
            self.__planning_prompt = planning_prompt

        if functions_prompt is None:
            functions_prompt_path = path.join(PROMPT_DIR, "functions.prompt")
            self.__functions_prompt = open(functions_prompt_path, "r").read()
        else:
            self.__functions_prompt = functions_prompt

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

    def generate_state_prompt(self) -> str:
        """
        When called, this function queries the state management
        system for all known objects. It then builds a
        comprehensive string describing the state of the world
        as we know it.
        """
        pass

    def __get_all_objects(self) -> List[StateObject]:
        pass
