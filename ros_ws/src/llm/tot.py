from __future__ import annotations
from concurrent.futures import ThreadPoolExecutor, Future, wait
from providers import Provider
from retry import retry
from time import time

from typing import Optional, List

UNKNOWN = 0
COMPLETE = 1
BAD = 2


class Step:
    def __init__(
        self,
        prompt: str,
        result: Optional[str] = None,
        children: List[Step] = [],
        rating: Optional[float] = None,
        completed: bool = False,
    ):
        self.prompt = prompt
        self.result = result
        self.children = children
        self.rating = rating
        self.completed = completed

    def clone(self) -> Step:
        return Step(
            self.prompt,
            self.result,
            children=[thought.clone() for thought in self.children],
            rating=self.rating,
            completed=self.completed,
        )

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


class TreeOfThoughts:
    def __init__(
        self,
        provider: Provider,
        control_prompt: str,
        evaluate_prompt: str,
        evaluation_categories: List[str],
        temperature: float = 0.7,
        width: int = 4,
        max_time: Optional[float] = None,
        max_steps: Optional[int] = None,
    ) -> None:
        """
        TreeOfThoughts is an object that is prepared to act as an reusable ToT
        prompting engine for a particular problem set. It is configured with a
        set of prompts, evaluation categories, and configuration settings to
        try to derive the desired output from the given LLM provider.

        provider: Provider - the LLM provider to use for prompting
        control_prompt: str - the prompt to use for generating each initial
            thought.  It utilizes placeholder templates that are replaced at
            prompt generation time to input the next set of the prompt. Thus
            you must at instantiation of the ToT engine provide a prompt that
            can have the engine generate the next possible step. The control
            prompt always assumes that the desired output for each step is:
            1 - Reason - the reasoning for the next step in the process
            2 - Step: the next step in the process.
            For instance, if the goal of the engine is to solve algebraic
            equations, we would expect a given next step for the equation
            (x + 2 = 4) to be something along the lines of:

            Reason: Subtract 2 from both sides
            Step: x = 2

            ...in that format.

            The placeholder templates within the prompt are as follows:
            {{state_all}} - the latest entirety of generate steps alone
            {{state_last}} - the latest step only, without its reasoning
            {{reason_state_all}} - the latest entirety of generated
                reasoning and their steps, alternating to now
            {{reason_state_last}} - the latest generated reasoning and
                step

            Thus using our previous example, a prompt could be:

            ===
            Given an the equation in its current step, generate an explanation
            of the next step of algebra to perform to solve for x, and then
            perform it. Only perform a single set of Reason and step. The
            following examples show how to format your response:

            Equation: x^2 - 3x + 4 = 0
            Reason: Factor the equation
            Step: (x - 1)(x - 4) = 0
            Equation: (x - 1)(x - 4) = 0
            Reason: We can see x is either 1 or 4
            Step: x = 1 or x = 4

            Equation: 3x - 12 = 24
            Reason: Add 12 to both sides
            Step: 3x = 36
            Equation: 3x = 36
            Reason: Divide both sides by 3
            Step: x = 12

            Equation: {{state_last}}

            ===
            ...where LLM generates the singular next step

        evaluate_prompt: str - the prompt to use for evaluating the quality of
            a given step. This prompt is asking the LLM to produce a set of
            possible categorical labels which we'll convert to values (see
            evaluation_categories). Reasoning for that labels is also expected
            to be requested to improve performance. The placeholder templates
            within this prompt are as follows:


            {{state_last}} - the last generated step and its reason
            {{evaluation_categories}} - a generated ordered list of possible
                labels for the LLM to apply in response.

            Thus using our previous example, a prompt could be:

            ===
            The following is a generated step for an algebraic equation.
            Rate the reasoning and last step performance by assigning
            one of the following categories:

            {{evaluation_categories}}

            Examples:

            Equation: 3x - 12 = 24
            Reason: Add 12 to both sides
            Step: 3x = 36
            Reason: This is a good step because it reduces us to one term on
                each side
            Rating: Good

            Equation: 3x - 12 = 24
            Reason: Divide each side by 3 to isolate x in its term
            Step: x - 4 = 8
            Reason: This is an okay step because it isolates X but doesn't
                reduce the equation to a simpler form
            Rating: Okay

            Equation: e^(x+3) = 4
            Reason: Divide by 13 to simplify
            Step: e^(x+3)/13 = 4/13
            Reason: This is a bad step because it doesn't simplify the equation
                nor does it isolate x
            Rating: Bad

            {{reason_state_all}}

            ===


        """
        self.provider = provider

        self.temperature = temperature
        self.width = width
        self.max_time = max_time
        self.max_steps = max_steps
        self.per_step_timeout = 10.0
        self.total_timeout = 60.0

        self.llm_threadpool = ThreadPoolExecutor(max_workers=5)

        self.generate_prompt = ""
        self.evaluation_prompt = ""

    def rate(self, prompt: str) -> float:
        pass

    def multi_prompt(
        self,
        prompt: str,
        temperature: float,
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

    @retry(tries=3)
    def evaluate(self, branch: Step) -> float:
        pass

    def execute(self, prompt: str) -> Step:
        """
        execute runs the tree of thoughts engine
        as configured, producing (hopefully) a
        chain of thoughts in the form of a ThoughtTree
        and an answer.
        """
        stop = False
        started_at = time()

        root_thought = Step(prompt)
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


# Execute process

# 1 - for targe prompt, generate N possible answers
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
