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
    # OpenRouter or DashScope endpoint (both are OpenAI-compatible)
    QWEN_API_BASE = os.getenv("QWEN_API_BASE", "https://openrouter.ai/api/v1")
    
    if not QWEN_API_KEY:
        raise ValueError("QWEN_API_KEY not found in environment variables.")
    
    # Model selection
    # OpenRouter free models: qwen/qwen-2.5-7b-instruct:free, qwen/qwen-2-7b-instruct:free
    # DashScope models: qwen-turbo, qwen-plus, qwen-max
    qwen_model = os.getenv("QWEN_MODEL", "qwen/qwen-2.5-7b-instruct:free")
    
    # Embedding model (not used with OpenRouter)
    qwen_embedding_model = os.getenv("QWEN_EMBEDDING_MODEL", "text-embedding-v3")
    
    # Detect provider from API base
    provider = "OpenRouter" if "openrouter.ai" in QWEN_API_BASE else "DashScope"
    
    print(f"✓ Successfully initialized Qwen API client via {provider}")
    print(f"✓ Model: {qwen_model}")
    print(f"✓ API Base: {QWEN_API_BASE}")
    qwen_initialized = True

except Exception as e:
    print(f"✗ Error initializing Qwen client: {e}")
    qwen_initialized = False


async def get_embedding(text: str):
    """
    Generates an embedding for the given text using Qwen FREE API (OpenAI-compatible format).
    """
    if not qwen_initialized:
        raise RuntimeError("Qwen client is not initialized.")
    
    try:
        await rate_limiter.wait_if_needed()
        
        # Free API uses OpenAI-compatible endpoint
        url = f"{QWEN_API_BASE}/embeddings"
        headers = {
            "Authorization": f"Bearer {QWEN_API_KEY}",
            "Content-Type": "application/json"
        }
        
        # OpenAI-compatible format for free API
        payload = {
            "model": qwen_embedding_model,
            "input": text
        }
        
        async with aiohttp.ClientSession() as session:
            async with session.post(url, headers=headers, json=payload) as response:
                if response.status == 200:
                    result = await response.json()
                    # OpenAI format returns data array with embedding
                    if result.get("data") and len(result["data"]) > 0:
                        return result["data"][0]["embedding"]
                else:
                    error_text = await response.text()
                    print(f"Error generating embedding: {response.status} - {error_text}")
                    return None
                    
    except Exception as e:
        print(f"Error generating embedding: {e}")
        return None


async def generate_answer(context: str, question: str) -> str:
    """
    Generates an answer using Qwen FREE API (OpenAI-compatible format).
    """
    if not qwen_initialized:
        raise RuntimeError("Qwen client is not initialized.")
    
    try:
        await rate_limiter.wait_if_needed()
        
        # OpenAI-compatible chat completions endpoint
        url = f"{QWEN_API_BASE}/chat/completions"
        headers = {
            "Authorization": f"Bearer {QWEN_API_KEY}",
            "Content-Type": "application/json"
        }
        
        # Add OpenRouter-specific headers if using OpenRouter
        if "openrouter.ai" in QWEN_API_BASE:
            headers["HTTP-Referer"] = "https://shaya9.github.io/ai-humanoid-robotics-book"
            headers["X-Title"] = "AI Humanoid Robotics Book"
        
        # OpenAI-compatible message format
        messages = [
            {
                "role": "system",
                "content": "You are a helpful AI assistant for the AI Humanoid Robotics Book. Provide accurate, detailed answers based on the context provided."
            },
            {
                "role": "user",
                "content": f"{context}\n\nQuestion: {question}"
            }
        ]
        
        # OpenAI-compatible payload
        payload = {
            "model": qwen_model,
            "messages": messages,
            "max_tokens": 1500,
            "temperature": 0.7,
            "top_p": 0.8
        }
        
        print(f"🔍 Calling Qwen API: {url}")
        print(f"🔍 Using model: {qwen_model}")
        
        async with aiohttp.ClientSession() as session:
            async with session.post(url, headers=headers, json=payload) as response:
                response_text = await response.text()
                print(f"📥 Response status: {response.status}")
                print(f"📥 Response body: {response_text[:500]}...")  # Log first 500 chars
                
                if response.status == 200:
                    result = json.loads(response_text)
                    # OpenAI format returns choices array
                    if result.get("choices") and len(result["choices"]) > 0:
                        return result["choices"][0]["message"]["content"]
                    else:
                        return "I couldn't generate a response. Please try again."
                else:
                    print(f"❌ Error generating answer: {response.status} - {response_text}")
                    
                    # Handle quota errors gracefully
                    if response.status == 429:
                        return "I'm currently experiencing high demand. Please try again in a few moments."
                    elif response.status == 404:
                        return "API endpoint not found. Please check your QWEN_API_BASE configuration."
                    
                    return f"Sorry, I encountered an error while generating a response."
                    
    except Exception as e:
        error_msg = str(e)
        print(f"❌ Exception in generate_answer: {error_msg}")
        
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