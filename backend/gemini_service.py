import os
import google.generativeai as genai
from dotenv import load_dotenv
import asyncio
import time
from collections import deque

load_dotenv()

# Rate limiting: Track requests to avoid quota issues
class RateLimiter:
    def __init__(self, max_requests_per_minute=15):
        self.max_requests = max_requests_per_minute
        self.requests = deque()
    
    async def wait_if_needed(self):
        """Wait if we've exceeded rate limit"""
        now = time.time()
        # Remove requests older than 60 seconds
        while self.requests and self.requests[0] < now - 60:
            self.requests.popleft()
        
        if len(self.requests) >= self.max_requests:
            # Wait until oldest request is 60 seconds old
            wait_time = 60 - (now - self.requests[0])
            if wait_time > 0:
                print(f"Rate limit reached. Waiting {wait_time:.1f}s...")
                await asyncio.sleep(wait_time)
                # Clean up old requests after waiting
                now = time.time()
                while self.requests and self.requests[0] < now - 60:
                    self.requests.popleft()
        
        self.requests.append(now)

rate_limiter = RateLimiter(max_requests_per_minute=15)

# --- Gemini Client Initialization ---
try:
    GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
    if not GEMINI_API_KEY:
        raise ValueError("GEMINI_API_KEY not found in environment variables.")

    genai.configure(api_key=GEMINI_API_KEY)

    # Model selection based on availability
    # Embedding model needs the "models/" prefix
    embedding_model = "models/text-embedding-004"
    
    # You can change this to one of:
    # - "gemini-1.5-flash-latest" (stable, recommended)
    # - "gemini-1.5-flash-8b-latest" (faster, lower cost)
    # - "gemini-1.5-pro-latest" (more capable)
    # - "gemini-2.0-flash-exp" (experimental)
    # Note: Use -latest suffix for stable versions
    model_name = os.getenv("GEMINI_MODEL", "gemini-2.0-flaash")
    
    generative_model = genai.GenerativeModel(model_name)
    print(f"✓ Successfully initialized Gemini client with {model_name}")

except Exception as e:
    print(f"✗ Error initializing Gemini client: {e}")
    generative_model = None


async def get_embedding(text: str):
    """
    Generates an embedding for the given text using Gemini.
    Runs in async context with rate limiting.
    """
    try:
        await rate_limiter.wait_if_needed()
        
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
    Supports both sync and async operations with rate limiting.
    """
    if not generative_model:
        raise RuntimeError("Gemini generative model is not initialized.")
    
    try:
        await rate_limiter.wait_if_needed()
        
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
        error_msg = str(e)
        print(f"Error generating answer: {error_msg}")
        
        # Handle quota errors gracefully
        if "429" in error_msg or "quota" in error_msg.lower():
            return "I'm currently experiencing high demand. Please try again in a few moments."
        
        return f"Sorry, I encountered an error while generating a response: {error_msg}"


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

