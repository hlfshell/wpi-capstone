import os
from abc import ABC, abstractmethod
from typing import List, Optional, Union

import openai
import google.generativeai as palm


class LLM(ABC):
    @abstractmethod
    def prompt(self, prompt: Union[str, List[str]], temperature: float) -> str:
        pass


class OpenAI(LLM):
    def __init__(self, key: Optional[str] = None, model: Optional[str] = None):
        super().__init__()

        if key is None:
            env_var = os.getenv("OPENAI_API_KEY")
            if env_var is None or env_var == "":
                raise ValueError("No API key provided")

            openai.api_key = env_var
        else:
            openai.api_key = key

        if model is None or model == "":
            self.__model = "gpt-3.5-turbo-1106"
            # self.__model = "gpt-4-1106-preview"
        else:
            self.__model = model

    def prompt(
        self,
        prompt: Union[str, List[str]],
        temperature: float = 0.7,
        model: Optional[str] = None,
        json: bool = False,
    ) -> str:
        if model is None:
            model = self.__model

        messages = []

        if isinstance(prompt, str):
            messages.append({"role": "system", "content": prompt})
        else:
            for p in prompt:
                messages.append({"role": "system", "content": p})

        response = openai.ChatCompletion.create(
            model=model,
            temperature=temperature,
            response_format={"type": "json_object" if json else "text"},
            max_tokens=4096,
            top_p=1,
            messages=messages,
        )

        return response.choices[0].message.content

    def clean_response(self, output: str) -> str:
        """
        Removes common things the AI puts around the python code we
        want
        """
        # Check to see if '''python or ``` is in the output
        if "```python" in output:
            output = output.split("```python")[1]
            output = output.split("```")[0]
        elif "```" in output:
            output = output.split("```")[1]

        # Strip useless ending/start whitespace
        output = output.strip()

        return output


class PaLM(LLM):
    def __init__(self, key: Optional[str] = None, model: Optional[str] = None):
        if key is None:
            env_var = os.getenv("GOOGLE_API_KEY")
            if env_var is None or env_var == "":
                raise ValueError("No API key provided")

            api_key = env_var
        else:
            api_key = key
        palm.configure(api_key=api_key)

        if model is None or model == "":
            self.model = "models/text-bison-001"
        else:
            self.model = model

    def prompt(self, prompts: List[str], temperature: float = 0.7) -> str:
        # Join the prompts into one string delimited by new lines
        prompt = "\n".join(prompts)
        completion = palm.generate_text(
            model=self.model,
            prompt=prompt,
            temperature=temperature,
        )

        return completion.result.strip()

    def clean_response(self, output: str) -> str:
        """
        Removes common things the AI puts around the python code we
        want
        """
        # Check to see if '''python or ``` is in the output
        if "```python" in output:
            output = output.split("```python")[1]
            output = output.split("```")[0]
        elif "```" in output:
            output = output.split("```")[1]

        # Strip useless ending/start whitespace
        output = output.strip()

        return output
