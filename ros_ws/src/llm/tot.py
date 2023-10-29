from __future__ import annotations

from abc import ABC, abstractmethod
from concurrent.futures import Future, ThreadPoolExecutor, wait
from time import time
from typing import List, Optional, Union
from uuid import uuid4

from providers import Provider
from retry import retry


class Step:
    def __init__(
        self,
        prompt: str,
        result: Optional[str] = None,
        children: List[Step] = [],
        rating: Optional[float] = None,
        completed: bool = False,
    ):
        self._id = uuid4()
        self.prompt = prompt
        self.result = result
        self.children = children
        self.rating = rating
        self.completed = completed

    def clone(self) -> Step:
        clone = Step(
            self.prompt,
            self.result,
            children=[thought.clone() for thought in self.children],
            rating=self.rating,
            completed=self.completed,
        )
        clone._id = self._id
        return clone

    def isolate_chain(self, step: Union[str, Step]) -> Step:
        """
        isolate_chain will return the chain of steps to the desired
        step/step id, with all branches pruned
        """
        # check to see if step is a Step
        if isinstance(step, Step):
            id = step._id
        else:
            id = step

        if self._id == id:
            clone = self.clone()
            clone.children = []
            return clone
        for child in self.children:
            chain = child.isolate_chain(id)
            if chain is not None and len(chain) > 0:
                clone = self.clone()
                clone.children = [chain]
                return clone
        return None

    def is_complete(self) -> bool:
        """
        is_complete checks to see if this node or one
        of its children is "completed" and thus a possible
        answer to the query.
        """
        if self.completed:
            return True
        for child in self.children:
            if child.is_complete():
                return True
        return False

    def __eq__(self, __value: object) -> bool:
        if __value is None:
            return False
        return self._id == __value._id

    def generate_string_to(step: Step) -> str:
        pass

    def html(self, wrap_in_html: bool = True) -> str:
        """
        html creates an increasingly more nested
        collapsible html representation of a set of
        thoughts.

        If wrap_in_html is True, then the html is wrapped
        in a full html document with a style tag that
        cleans up the css for the html generated.
        """

        children_html = ""
        for child in self.children:
            children_html += child.html(wrap_in_html=False)

        html = f"""
            <details class="tot">
                <summary class="summary-step{" complete" if self.is_complete()
                    else ""}">{self.result} | {f'Rating: {self.rating}' if
                        self.rating is not None else ""}
                </summary>
                {children_html}
            </details>
        """

        if wrap_in_html:
            html = f"""
                <html>
                    <head>
                        <style>
                            {CSS}
                        </style>
                    </head>
                    <body>
                        <div id="wrapper">
                            {html}
                        </div>
                    </body>
                </html>
            """

        return html


class TreeOfThoughts(ABC):
    def __init__(
        self,
        provider: Provider,
        evaluation_categories: List[str],
        evaluation_category_scores: Optional[List[float]] = None,
        temperature: float = 0.7,
        children_fan_out: int = 0,
        max_time: Optional[float] = None,
        max_steps: Optional[int] = None,
    ) -> None:
        """
        TreeOfThoughts is an object that is prepared to act as an reusable ToT
        prompting engine for a particular problem set. It is configured with a
        set of prompts, evaluation categories, and configuration settings to
        try to derive the desired output from the given LLM provider.



        """
        self.provider = provider

        self.evaluation_categories = evaluation_categories
        self.temperature = temperature
        self.width = children_fan_out
        self.max_time = max_time
        self.max_steps = max_steps
        self.per_step_timeout = 10.0
        self.total_timeout = 60.0

        # If the evaluation_category_scores is None, make it
        # a set of floats evenly spaced from 0 to 1 based on the
        # number of descriptor labels provided. IE if 3 labels
        # are provided, then the scores will be [0.0, 0.5, 1.0]
        if evaluation_category_scores is None:
            evaluation_category_scores = [
                i / (len(evaluation_categories) - 1)
                for i in range(0, len(evaluation_categories))
            ]
        if len(evaluation_categories) != len(evaluation_category_scores):
            raise ValueError(
                "evaluation_categories and evaluation_category_scores must "
                + "be the same length"
            )

        self.llm_threadpool = ThreadPoolExecutor(max_workers=5)

    @abstractmethod
    def generate_step_prompt(self, steps: List[Step]) -> str:
        """
        generate_step_prompt takes the existing produced steps and
        returns a new prompt that is used to ask the LLM for the
        next step.

        Must be defined by the implementing class.
        """
        pass

    @abstractmethod
    def parse_generation_response(self, response: str) -> Step:
        """
        parse_step_response takes the response from the LLM provider
        and parses it into a string that is the next step and
        its reasoning in the form of a Step

        Must be defined by the implementing class.
        """
        pass

    @abstractmethod
    def evaluate_step_prompt(self, step: Step) -> str:
        """
        evaluate_step_prompt takes a generated step and returns
        a new prompt that is used to ask the LLM to rate the step
        based on a set of predetermined categorical labels.

        Must be defined by the implementing class.
        """
        pass

    @abstractmethod
    def parse_evaluation_response(self, response: str) -> str:
        """
        parse_evaluation_response takes the response from the LLM's
        evaluation of a given step and returns a string that is the
        assigned categorical label for a given step.

        Must be defined by the implementing class.
        """
        pass

    def rate(self, step: Step) -> float:
        """
        rate will request a prompt for a given step and request
        a labeled rating from the LLM provider. It will finally
        return the float.

        It can raise an error if the prompt fails after 3 retries
        or if the parsed label is not in the evaluation_categories
        """
        rating_prompt = self.evaluate_step_prompt(step.prompt)
        rating_response = self.prompt(rating_prompt)
        label = self.parse_evaluation_response(rating_response)

        # Find the label if it exists within the category store
        label_index = self.evaluation_categories.index(label)
        if label_index == -1:
            raise ValueError(f"Unknown label {label}")

        # Map the score to the label
        score = self.evaluation_category_scores[label_index]

        return score

    def multi_prompt(
        self,
        prompt: str,
        times: int = -1,
        per_step_timeout: float = 0,
    ) -> List[Step]:
        """
        multi_prompt triggers the given prompt multiple times (the times
        parameter) at once utilizing max_workers workers in a threadpool.
        If times is -1, then the tree of thoughts default of self.width
        is utilized. If per_step_timeout is <= 0, then the default of
        self.per_step_timeout is utilized.
        """
        if times == -1:
            times = self.width
        if per_step_timeout <= 0:
            per_step_timeout = self.per_step_timeout

        with self.llm_threadpool as executor:
            futures: List[Future] = []
            for _ in range(0, times):
                future = executor.submit(self.prompt, prompt)
                futures.append(future)

            # Wait for each future to finish - we can't
            # continue on a given thread until we have
            # scores for each anyway.
            wait(futures)

            branches: List[Step] = []

            for future in futures:
                # Skip over any exceptions; if it failed even
                # with retry we just skip it for now
                if future.exception() is not None:
                    continue

                response = future.result()
                branches.append(Step(prompt, response))

            return branches

    @retry(tries=3)
    def prompt(self, prompt: str) -> str:
        """
        prompt will trigger a network call to the llm provider with a
        built in retry mechanism
        """
        return self.provider.prompt(prompt, self.temperature)

    def execute(self, task: str) -> Step:
        """
        execute runs the tree of thoughts engine
        as configured, producing (hopefully) a
        chain of thoughts in the form of a ThoughtTree
        and an answer.
        """
        stop = False
        started_at = time()

        root_thought = Step()
        parent_thought = root_thought
        current_thought = None

        # Until we trigger a stop condition, we keep expanding
        # upon the tree of thoughts
        while not stop:
            if time() - started_at > self.total_timeout:
                stop = True
                break
            if current_thought is not None:
                parent_thought.children.append(current_thought)
                parent_thought = current_thought

            # Generate a number of possible branches
            # from the current thought
            branches = self.multi_prompt(
                parent_thought.prompt,
                self.temperature,
                per_step_timeout=self.per_step_timeout,
            )

            parent_thought.children = branches

            # Evaluate each thought
            for thought in branches:
                thought.rating = self.rate(thought.result)

        return root_thought


CSS = """
#wrapper {
    margin: 0 auto;
    width: 80%;
}

details[open] details {
  animation: animateDown 0.2s linear forwards;
}

@keyframes animateDown {
  0% {
    opacity: 0;
    transform: translatey(-15px);
  }
  100% {
    opacity: 1;
    transform: translatey(0);
  }
}

details details {
    margin-left: 20px;
    border-left: 3px rgba(128, 128, 128, 0.5) solid;
    padding-left: 5px;
}

summary {
    display: block;
}

summary::after {
    margin-left: 1ch;
    display: inline-block;
    transition: 0.2s;
}

details summary::after {
    content: '➕';
}

details[open] summary::after {
    content: '➖';
}

details:not([open]) summary::after {
    content: '➕';
}

.complete {
    background-color: rgba(19, 227, 42, 0.5);
}
"""
