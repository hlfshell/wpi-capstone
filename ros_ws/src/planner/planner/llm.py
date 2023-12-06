import os
from abc import ABC, abstractmethod
from typing import List, Optional, Union

import openai


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

    def prompt(self, prompt: Union[str, List[str]], temperature: float = 0.7) -> str:
        messages = []

        if isinstance(prompt, str):
            messages.append({"role": "system", "content": prompt})
        else:
            for p in prompt:
                messages.append({"role": "system", "content": p})

        response = openai.ChatCompletion.create(
            model=self.__model,
            temperature=temperature,
            max_tokens=4096,
            top_p=1,
            messages=messages,
        )
        return response.choices[0].message.content
