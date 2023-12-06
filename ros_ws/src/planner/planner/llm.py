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

        # for index, message in enumerate(messages):
        #     print("===")
        #     print(index + 1)
        #     print(type(message["content"]))
        #     print(len(message))
        #     print(len(message["content"]))
        #     print(message)
        #     print("===")

        try:
            response = openai.ChatCompletion.create(
                model=model,
                temperature=temperature,
                response_format={"type": "json_object" if json else "text"},
                max_tokens=4096,
                top_p=1,
                messages=messages,
            )
        except Exception as e:
            print("Error with prompt:")
            print(e)
            raise e

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
