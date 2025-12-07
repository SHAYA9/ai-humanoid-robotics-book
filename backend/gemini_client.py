import os
import google.generativeai as genai
from dotenv import load_dotenv

load_dotenv()

# --- Gemini Client Initialization ---
try:
    GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
    if not GEMINI_API_KEY:
        raise ValueError("GEMINI_API_KEY not found in environment variables.")

    genai.configure(api_key=GEMINI_API_KEY)

    embedding_model = "models/text-embedding-004"
    generative_model = genai.GenerativeModel("gemini-1.5-flash")
    print("Successfully initialized Gemini client.")

except Exception as e:
    print(f"Error initializing Gemini client: {e}")
    generative_model = None

async def get_embedding(text: str):
    """Generates an embedding for the given text."""
    try:
        result = await genai.embed_content_async(model=embedding_model, content=text)
        return result['embedding']
    except Exception as e:
        print(f"Error generating embedding: {e}")
        return None

async def generate_answer(context: str, question: str):
    """Generates an answer based on the given context and question."""
    if not generative_model:
        raise RuntimeError("Gemini generative model is not initialized.")
    try:
        prompt = f"""
        You are a helpful assistant for the AI Humanoid Robotics Book.
        Based ONLY on the following selected text from the book, answer the user's question.
        Do not use any other information. If the answer is not in the text, say so.
        Keep your answer concise and directly related to the text.

        Selected Text:
        ---
        {context}
        ---

        Question: {question}
        """
        response = await generative_model.generate_content_async(prompt)
        return response.text
    except Exception as e:
        print(f"Error generating answer: {e}")
        return "Sorry, I encountered an error while generating a response."
