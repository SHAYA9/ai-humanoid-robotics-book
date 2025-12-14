import os
import asyncio
import time
from collections import deque
from dotenv import load_dotenv
import aiohttp
import json

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

# --- Qwen Client Initialization ---
try:
    QWEN_API_KEY = os.getenv("QWEN_API_KEY")
    QWEN_API_BASE = os.getenv("QWEN_API_BASE", "https://dashscope.aliyuncs.com/api/v1")
    
    if not QWEN_API_KEY:
        raise ValueError("QWEN_API_KEY not found in environment variables.")
    
    # Model selection
    # Available models:
    # - "qwen-turbo" (fast, cost-effective)
    # - "qwen-plus" (balanced performance)
    # - "qwen-max" (most capable)
    qwen_model = os.getenv("QWEN_MODEL", "qwen-turbo")
    
    # Embedding model
    qwen_embedding_model = os.getenv("QWEN_EMBEDDING_MODEL", "text-embedding-v2")
    
    print(f"✓ Successfully initialized Qwen client with {qwen_model}")
    qwen_initialized = True

except Exception as e:
    print(f"✗ Error initializing Qwen client: {e}")
    qwen_initialized = False


async def get_embedding(text: str):
    """
    Generates an embedding for the given text using Qwen.
    """
    if not qwen_initialized:
        raise RuntimeError("Qwen client is not initialized.")
    
    try:
        await rate_limiter.wait_if_needed()
        
        url = f"{QWEN_API_BASE}/services/embeddings/text-embedding/text-embedding"
        headers = {
            "Authorization": f"Bearer {QWEN_API_KEY}",
            "Content-Type": "application/json"
        }
        
        payload = {
            "model": qwen_embedding_model,
            "input": {
                "texts": [text]
            }
        }
        
        async with aiohttp.ClientSession() as session:
            async with session.post(url, headers=headers, json=payload) as response:
                if response.status == 200:
                    result = await response.json()
                    if result.get("output") and result["output"].get("embeddings"):
                        return result["output"]["embeddings"][0]["embedding"]
                else:
                    error_text = await response.text()
                    print(f"Error generating embedding: {response.status} - {error_text}")
                    return None
                    
    except Exception as e:
        print(f"Error generating embedding: {e}")
        return None


async def generate_answer(context: str, question: str) -> str:
    """
    Generates an answer based on the given context and question using Qwen.
    Uses DashScope API format.
    """
    if not qwen_initialized:
        raise RuntimeError("Qwen client is not initialized.")
    
    try:
        await rate_limiter.wait_if_needed()
        
        # Use DashScope text generation endpoint
        url = f"{QWEN_API_BASE}/services/aigc/text-generation/generation"
        headers = {
            "Authorization": f"Bearer {QWEN_API_KEY}",
            "Content-Type": "application/json"
        }
        
        # Format messages according to DashScope API
        messages = [
            {
                "role": "system",
                "content": "You are a helpful AI assistant for the AI Humanoid Robotics Book."
            },
            {
                "role": "user",
                "content": context
            }
        ]
        
        payload = {
            "model": qwen_model,
            "input": {
                "messages": messages
            },
            "parameters": {
                "result_format": "message"
            }
        }
        
        async with aiohttp.ClientSession() as session:
            async with session.post(url, headers=headers, json=payload) as response:
                if response.status == 200:
                    result = await response.json()
                    # Extract answer from DashScope response format
                    if result.get("output") and result["output"].get("choices"):
                        return result["output"]["choices"][0]["message"]["content"]
                    else:
                        return "I couldn't generate a response. Please try again."
                else:
                    error_text = await response.text()
                    print(f"Error generating answer: {response.status} - {error_text}")
                    
                    # Handle quota errors gracefully
                    if response.status == 429:
                        return "I'm currently experiencing high demand. Please try again in a few moments."
                    
                    return f"Sorry, I encountered an error while generating a response."
                    
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
    if not qwen_initialized:
        raise RuntimeError("Qwen client is not initialized.")
    
    try:
        # Run async function in new event loop
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        result = loop.run_until_complete(generate_answer(context, question))
        loop.close()
        return result
    except Exception as e:
        print(f"Error generating answer: {e}")
        return f"Sorry, I encountered an error: {str(e)}"