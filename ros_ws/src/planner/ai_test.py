from planner.ai import AI
from planner.llm import OpenAI


planning_prompt = open("./prompts/planning.prompt", "r").read()
functions_prompt = open("./prompts/functions.prompt", "r").read()

# llm = OpenAI(model="gpt-4-1106-preview")
llm = OpenAI()
# ai = AI(llm)

# print(llm.prompt("Hi there!"))


prompts = [
    planning_prompt,
    functions_prompt,
    """
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
    """,
    """
Remember, do not reply with anything but python code to accomplish your goal.
Your objective is to: Get the user a drink
    """,
]

response = llm.prompt(prompts, temperature=0.7)
print("***")
print(response)
print("***")
print(llm.clean_response(response))
