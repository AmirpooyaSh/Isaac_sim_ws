# sample_usage.py
import os
import sys
from openai import OpenAI

class UFLOpenAI:
    def __init__(
        self,
        api_key: str | None = None,
        model: str = "gpt-4o",
        base_url: str = "https://api.ai.it.ufl.edu",
    ):
        self.api_key = os.getenv("UFL_API_KEY")
        if not self.api_key:
            print("UFL_API_KEY not found; set it or pass api_key='…'")
            sys.exit(1)

        self.client = OpenAI(api_key=self.api_key, base_url=base_url)
        self.model = model

# --- usage ------------------------------------------------------------------
if __name__ == "__main__":
    # instantiate the wrapper (replace with your actual key or set UFL_API_KEY)
    ufl_openai = UFLOpenAI()

    # send a simple text-only prompt
    response = ufl_openai.client.responses.create(
        model=ufl_openai.model,
        input=[{
            "role": "user",
            "content": [
                {"type": "input_text", "text": "Hello, GPT-4o! Briefly introduce yourself."},
            ],
        }],
    )

    response = ufl_openai.client.responses.create(
        model= ufl_openai.model,
        input=[{
            "role": "user",
            "content": [
                {"type": "input_text", "text": "Hello, GPT-4o! Briefly introduce yourself."},
            ],
        }],
    )

    # print the assistant’s reply
    print(response)
