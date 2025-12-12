import os
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from dotenv import load_dotenv
import asyncio

# Load environment variables
load_dotenv()

# Import backend modules
import qdrant_client as qc
import gemini_client as gc

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
    os.getenv("FRONTEND_URL", "").strip() or None
]

# Remove None if FRONTEND_URL is empty
ALLOWED_ORIGINS = [origin for origin in ALLOWED_ORIGINS if origin]

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

# -----------------------------------------------------------------------
# API Endpoints
# -----------------------------------------------------------------------

@app.get("/")
def read_root():
    return {
        "status": "ok",
        "message": "Welcome to the RAG Chatbot API!",
        "version": "1.0.0"
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


# -----------------------------------------------------------------------
# ASGI to WSGI Adapter (required for PythonAnywhere)
# -----------------------------------------------------------------------
from fastapi.middleware.wsgi import WSGIMiddleware

application = WSGIMiddleware(app)
