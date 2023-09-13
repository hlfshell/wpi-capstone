import sys

from dotenv import load_dotenv
from langchain.llms.openai import OpenAI

# Load env vars
load_dotenv()

# See if we've specified a prompt file to read from
# If not then use the default prompt file
if len(sys.argv) > 1:
    prompt_file = sys.argv[1]
else:
    prompt_file = "experiments/llm/prompt"

# Read prompt file
with open(prompt_file, "r") as f:
    prompt = f.read()

# Create OpenAI instance
openai = OpenAI()

result = openai.predict(prompt)
print(result)