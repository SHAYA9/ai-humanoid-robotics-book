import os
import asyncio
import time
from collections import deque
from dotenv import load_dotenv
import aiohttp
import json
import google.generativeai as genai

load_dotenv()

# Rate limiting for both services
class RateLimiter:
    def __init__(self, max_requests_per_minute=15):
        self.max_requests = max_requests_per_minute
        self.requests = deque()
    
    async def wait_if_needed(self):
        """Wait if we've exceeded rate limit"""
        now = time.time()
        while self.requests and self.requests[0] < now - 60:
            self.requests.popleft()
        
        if len(self.requests) >= self.max_requests:
            wait_time = 60 - (now - self.requests[0])
            if wait_time > 0:
                print(f"Rate limit reached. Waiting {wait_time:.1f}s...")
                await asyncio.sleep(wait_time)
                now = time.time()
                while self.requests and self.requests[0] < now - 60:
                    self.requests.popleft()
        
        self.requests.append(now)

rate_limiter = RateLimiter(max_requests_per_minute=15)

# --- Initialize Gemini for Embeddings ONLY ---
try:
    GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
    if not GEMINI_API_KEY:
        raise ValueError("GEMINI_API_KEY not found in environment variables.")
    
    genai.configure(api_key=GEMINI_API_KEY)
    embedding_model = "models/text-embedding-004"
    print(f"✓ Gemini initialized for embeddings only")
    gemini_initialized = True
except Exception as e:
    print(f"✗ Error initializing Gemini: {e}")
    gemini_initialized = False

# --- Initialize Qwen/OpenRouter for Text Generation ONLY ---
try:
    QWEN_API_KEY = os.getenv("QWEN_API_KEY")
    QWEN_API_BASE = os.getenv("QWEN_API_BASE", "https://openrouter.ai/api/v1").rstrip('/')
    
    if QWEN_API_BASE.endswith('/chat/completions'):
        QWEN_API_BASE = QWEN_API_BASE.replace('/chat/completions', '')
    
    if not QWEN_API_KEY:
        raise ValueError("QWEN_API_KEY not found in environment variables.")
    
    qwen_model = os.getenv("QWEN_MODEL", "meta-llama/llama-3.2-3b-instruct:free")
    
    print(f"✓ Qwen/OpenRouter initialized for text generation")
    print(f"✓ Model: {qwen_model}")
    print(f"✓ API Base: {QWEN_API_BASE}")
    qwen_initialized = True
except Exception as e:
    print(f"✗ Error initializing Qwen: {e}")
    qwen_initialized = False


async def get_embedding(text: str):
    """
    Uses GEMINI for embeddings (to match ingestion).
    This ensures compatibility with your Qdrant database.
    """
    if not gemini_initialized:
        raise RuntimeError("Gemini client is not initialized for embeddings.")
    
    try:
        await rate_limiter.wait_if_needed()
        
        # Run Gemini embedding in thread pool (it's synchronous)
        loop = asyncio.get_event_loop()
        result = await loop.run_in_executor(
            None,
            lambda: genai.embed_content(model=embedding_model, content=text)
        )
        return result['embedding']
    except Exception as e:
        print(f"Error generating embedding with Gemini: {e}")
        return None


async def generate_answer(context: str, question: str) -> str:
    """
    Uses QWEN/OpenRouter for text generation (to avoid Gemini quota).
    This generates the actual answer to user questions.
    """
    if not qwen_initialized:
        raise RuntimeError("Qwen client is not initialized for text generation.")
    
    try:
        await rate_limiter.wait_if_needed()
        
        url = f"{QWEN_API_BASE}/chat/completions"
        
        headers = {
            "Authorization": f"Bearer {QWEN_API_KEY}",
            "Content-Type": "application/json"
        }
        
        # Add OpenRouter-specific headers
        if "openrouter.ai" in QWEN_API_BASE:
            headers["HTTP-Referer"] = "https://shaya9.github.io/ai-humanoid-robotics-book"
            headers["X-Title"] = "AI Humanoid Robotics Book"
        
        messages = [
            {
                "role": "system",
                "content": "You are a helpful AI assistant for the AI Humanoid Robotics Book. Provide accurate, detailed answers based on the context provided. Be concise but thorough."
            },
            {
                "role": "user",
                "content": f"Context from the book:\n{context}\n\nQuestion: {question}\n\nPlease provide a clear and helpful answer based on the context above."
            }
        ]
        
        payload = {
            "model": qwen_model,
            "messages": messages,
            "max_tokens": 1500,
            "temperature": 0.7,
            "top_p": 0.8
        }
        
        print(f"🔍 Generating answer with {qwen_model}")
        
        async with aiohttp.ClientSession() as session:
            async with session.post(url, headers=headers, json=payload) as response:
                response_text = await response.text()
                print(f"📥 Response status: {response.status}")
                
                if response.status == 200:
                    result = json.loads(response_text)
                    if result.get("choices") and len(result["choices"]) > 0:
                        return result["choices"][0]["message"]["content"]
                    else:
                        return "I couldn't generate a response. Please try again."
                else:
                    print(f"❌ Error: {response.status} - {response_text}")
                    
                    if response.status == 429:
                        return "I'm currently experiencing high demand. Please try again in a few moments."
                    elif response.status == 404:
                        return "API endpoint not found. Please check your configuration."
                    
                    return "Sorry, I encountered an error while generating a response."
                    
    except Exception as e:
        error_msg = str(e)
        print(f"❌ Exception in generate_answer: {error_msg}")
        
        if "429" in error_msg or "quota" in error_msg.lower():
            return "I'm currently experiencing high demand. Please try again in a few moments."
        
        return f"Sorry, I encountered an error: {error_msg}"


def generate_answer_sync(context: str, question: str) -> str:
    """
    Synchronous version of generate_answer.
    """
    if not qwen_initialized:
        raise RuntimeError("Qwen client is not initialized.")
    
    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        result = loop.run_until_complete(generate_answer(context, question))
        loop.close()
        return result
    except Exception as e:
        print(f"Error generating answer: {e}")
        return f"Sorry, I encountered an error: {str(e)}"