import os
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from dotenv import load_dotenv
import asyncio

# Load environment variables
load_dotenv()

# Import backend modules with error handling
try:
    import qdrant_service as qc
    import gemini_service as gc
    print("✓ Successfully imported backend modules")
except ImportError as e:
    print(f"✗ Error importing backend modules: {e}")
    import traceback
    traceback.print_exc()
    raise

# -----------------------------------------------------------------------
# FastAPI App Initialization
# -----------------------------------------------------------------------

app = FastAPI(
    title="AI Humanoid Robotics Book - RAG Chatbot API",
    description="A RAG chatbot backend using FastAPI with Gemini API, Qdrant, and Neon Postgres.",
    version="1.0.0"
)

# -----------------------------------------------------------------------
# CORS Configuration
# -----------------------------------------------------------------------

ALLOWED_ORIGINS = [
    "http://localhost:3000",
    "http://localhost:3001",
    "http://localhost:8000",
    "https://shaya9.github.io",
    "https://shaya9.github.io/ai-humanoid-robotics-book",
    os.getenv("FRONTEND_URL", "").strip() or None
]

# Remove None if FRONTEND_URL is empty
ALLOWED_ORIGINS = [origin for origin in ALLOWED_ORIGINS if origin]

print(f"CORS Allowed Origins: {ALLOWED_ORIGINS}")

app.add_middleware(
    CORSMiddleware,
    allow_origins=ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# -----------------------------------------------------------------------
# Pydantic Models
# -----------------------------------------------------------------------

class ChatRequest(BaseModel):
    question: str
    context: str  # The user-selected text from the frontend

class GeneralChatRequest(BaseModel):
    question: str

class ChatResponse(BaseModel):
    answer: str
    sources: str

class TranslateRequest(BaseModel):
    content: str
    target_language: str = "urdu"

class TranslateResponse(BaseModel):
    translated_content: str

# -----------------------------------------------------------------------
# API Endpoints
# -----------------------------------------------------------------------

@app.get("/")
def read_root():
    # Try to reinitialize Qdrant if it failed on startup
    if not qc.qdrant:
        qc.initialize_qdrant()
    
    return {
        "status": "ok",
        "message": "Welcome to the RAG Chatbot API!",
        "version": "1.0.2",
        "qdrant_status": "initialized" if qc.qdrant else "not_initialized",
        "gemini_status": "initialized" if gc.generative_model else "not_initialized"
    }

@app.get("/health")
def health_check():
    return {"status": "healthy"}

@app.post("/api/chat/general", response_model=ChatResponse)
async def chat_general(request: GeneralChatRequest):
    """General non-RAG chat using Gemini."""
    try:
        prompt = f"""
You are a helpful AI assistant for the AI Humanoid Robotics Book.

Question: {request.question}

Provide a clear and accurate answer based on your knowledge of AI and robotics.
"""
        answer = await gc.generate_answer(prompt, request.question)
        return ChatResponse(answer=answer, sources="General Knowledge")

    except Exception as e:
        print(f"Error in /api/chat/general: {e}")
        raise HTTPException(status_code=500, detail=f"Error generating response: {str(e)}")


@app.post("/api/chat/selected", response_model=ChatResponse)
async def chat_selected_text(request: ChatRequest):
    """RAG Chat based ONLY on selected text + nearest passages from Qdrant."""
    try:
        # Try to reinitialize Qdrant if not available
        if not qc.qdrant:
            print("Qdrant not initialized, attempting to reinitialize...")
            if not qc.initialize_qdrant():
                raise RuntimeError("Qdrant client could not be initialized. Please check environment variables.")
        
        if not request.context:
            raise ValueError("Context (selected text) is required for RAG chat")

        # Search for similar passages in Qdrant
        similar_passages = await qc.search_similar_passages(request.context)

        # Build context
        context_for_llm = f"Selected Text:\n{request.context}\n\n"
        citations = set()

        if similar_passages:
            context_for_llm += "Related Passages:\n---\n"
            for passage in similar_passages:
                context_for_llm += passage.payload.get('text', '') + "\n"
                if 'source' in passage.payload:
                    citations.add(passage.payload['source'])

        # RAG Prompt
        prompt = f"""
You are an assistant for the AI Humanoid Robotics Book.

Based ONLY on the following provided text, answer the user's question.
If the answer is not found in this text, respond with:
"I do not have enough information in the provided text."

Context:
---
{context_for_llm}
---

Question: {request.question}

Answer:
"""

        answer = await gc.generate_answer(prompt, request.question)

        sources_str = ", ".join(list(citations)) if citations else "From selected text"
        return ChatResponse(answer=answer, sources=sources_str)

    except Exception as e:
        print(f"Error in /api/chat/selected: {e}")
        raise HTTPException(status_code=500, detail=f"Error processing selected text: {str(e)}")


@app.post("/api/translate", response_model=TranslateResponse)
async def translate_content(request: TranslateRequest):
    """Translate HTML content to target language while preserving structure."""
    try:
        prompt = f"""You are a professional translator. Translate the following HTML content from English to {request.target_language.title()}. 

IMPORTANT RULES:
1. Preserve ALL HTML tags exactly as they are (do not translate tag names or attributes)
2. Preserve ALL code blocks (anything inside <pre>, <code> tags) - do not translate code
3. Preserve ALL technical terms, variable names, function names, and programming keywords
4. Only translate the readable text content between HTML tags
5. Maintain the exact same HTML structure
6. For headings and titles, provide natural {request.target_language.title()} translations
7. Use proper {request.target_language.title()} grammar and sentence structure
8. Keep URLs, links, and file paths unchanged
9. Preserve any data-* attributes or special markers
10. Return ONLY the translated HTML without any explanations or markdown code blocks

HTML Content to translate:
{request.content}

Translated HTML (return only the HTML, no explanations):"""

        translated = await gc.generate_answer(prompt, "")
        
        # Clean up any markdown code blocks that might be added
        translated = translated.replace('```html\n', '').replace('```html', '').replace('```\n', '').replace('```', '')
        
        return TranslateResponse(translated_content=translated.strip())

    except Exception as e:
        print(f"Error in /api/translate: {e}")
        raise HTTPException(status_code=500, detail=f"Translation failed: {str(e)}")


# -----------------------------------------------------------------------
# ASGI to WSGI Adapter (required for PythonAnywhere)
# -----------------------------------------------------------------------
from fastapi.middleware.wsgi import WSGIMiddleware

application = WSGIMiddleware(app)
