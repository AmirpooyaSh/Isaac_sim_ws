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
    ufl_openai.client.chat.completions.create
    response = ufl_openai.client.chat.completions.create(
        model= ufl_openai.model,
        messages=[{
                    "role": "user",
                    "content": [
                        # --- your prompt & image ---
                        {"type": "text",  "text": "Tell me a Joke"},
                    ],
                }]
    )

    # print the assistant’s reply
    print(response.choices[0].message)
