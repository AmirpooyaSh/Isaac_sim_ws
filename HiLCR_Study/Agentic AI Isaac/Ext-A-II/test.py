import os
from openai import OpenAI

# 1. Initialize the client (Make sure to export your key first, or replace the string below)
# To export in terminal: export OPENAI_API_KEY="your-actual-api-key"
client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY", "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"))

# Using o3-mini (which belongs to the o3 reasoning series)
MODEL_NAME = "o3" 

def test_sampling_parameters(temp_val, top_p_val):
    print(f"\n--- Testing with Temperature={temp_val} and Top_p={top_p_val} ---")
    
    try:
        # Standard API payload format
        response = client.chat.completions.create(
            model=MODEL_NAME,
            messages=[
                {"role": "user", "content": "Give me a one-sentence random creative fact."}
            ],
            temperature=temp_val,
            top_p=top_p_val
        )
        # If it succeeds, print the text output
        print("✅ SUCCESS!")
        print("Response:", response.choices[0].message.content)
        
    except Exception as e:
        # If OpenAI rejects the custom parameters, catch and show the exact API error
        print("❌ FAILED / REJECTED BY API")
        print("Error Details:", str(e))

# Run the test
# Try changing these values (e.g., to 0.0 or 0.5) to see the API block the request
test_sampling_parameters(temp_val=0.0, top_p_val=0.2)
