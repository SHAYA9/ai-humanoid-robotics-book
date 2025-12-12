import os
import google.generativeai as genai
from dotenv import load_dotenv
import asyncio

load_dotenv()

# --- Gemini Client Initialization ---
try:
    GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
    if not GEMINI_API_KEY:
        raise ValueError("GEMINI_API_KEY not found in environment variables.")

    genai.configure(api_key=GEMINI_API_KEY)

    # Using stable Gemini 1.5 Flash (lower quota usage)
    embedding_model = "models/text-embedding-004"
    generative_model = genai.GenerativeModel("gemini-1.5-flash")
    print("✓ Successfully initialized Gemini client.")

except Exception as e:
    print(f"✗ Error initializing Gemini client: {e}")
    generative_model = None


async def get_embedding(text: str):
    """
    Generates an embedding for the given text using Gemini.
    Runs in async context.
    """
    try:
        # Gemini API doesn't have true async, so we run it in thread pool
        loop = asyncio.get_event_loop()
        result = await loop.run_in_executor(
            None,
            lambda: genai.embed_content(model=embedding_model, content=text)
        )
        return result['embedding']
    except Exception as e:
        print(f"Error generating embedding: {e}")
        return None


async def generate_answer(context: str, question: str) -> str:
    """
    Generates an answer based on the given context and question using Gemini.
    Supports both sync and async operations.
    """
    if not generative_model:
        raise RuntimeError("Gemini generative model is not initialized.")
    
    try:
        # Run the Gemini API call in executor since it's synchronous
        loop = asyncio.get_event_loop()
        response = await loop.run_in_executor(
            None,
            lambda: generative_model.generate_content(context)
        )
        
        if response and response.text:
            return response.text
        else:
            return "I couldn't generate a response. Please try again."
            
    except Exception as e:
        print(f"Error generating answer: {e}")
        return f"Sorry, I encountered an error while generating a response: {str(e)}"


def generate_answer_sync(context: str, question: str) -> str:
    """
    Synchronous version of generate_answer for use outside async context.
    """
    if not generative_model:
        raise RuntimeError("Gemini generative model is not initialized.")
    
    try:
        response = generative_model.generate_content(context)
        if response and response.text:
            return response.text
        else:
            return "I couldn't generate a response. Please try again."
    except Exception as e:
        print(f"Error generating answer: {e}")
        return f"Sorry, I encountered an error: {str(e)}"

