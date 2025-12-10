import os
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from dotenv import load_dotenv
import asyncio

# Load environment variables
load_dotenv()

# Import backend modules
from . import qdrant_client, gemini_client

# --- FastAPI App Initialization ---
app = FastAPI(
    title="AI Humanoid Robotics Book - RAG Chatbot API",
    description="A RAG chatbot backend using FastAPI with Gemini API, Qdrant, and Neon Postgres.",
    version="1.0.0"
)

# --- CORS Configuration ---
ALLOWED_ORIGINS = [
    "http://localhost:3000",
    "http://localhost:3001",
    "http://localhost:8000",
    "https://shaya9.github.io",
    os.getenv("FRONTEND_URL", "").strip() or None
]

# Filter out None values
ALLOWED_ORIGINS = [origin for origin in ALLOWED_ORIGINS if origin]

app.add_middleware(
    CORSMiddleware,
    allow_origins=ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- Pydantic Models ---
class ChatRequest(BaseModel):
    question: str
    context: str  # The user-selected text from the frontend

class GeneralChatRequest(BaseModel):
    question: str

class ChatResponse(BaseModel):
    answer: str
    sources: str

# --- API Endpoints ---

@app.get("/")
def read_root():
    """Health check endpoint."""
    return {
        "status": "ok",
        "message": "Welcome to the RAG Chatbot API!",
        "version": "1.0.0"
    }

@app.get("/health")
def health_check():
    """Health check endpoint for monitoring."""
    return {"status": "healthy"}

@app.post("/api/chat/general", response_model=ChatResponse)
async def chat_general(request: GeneralChatRequest):
    """
    Endpoint for general, non-RAG chat.
    Forwards the question directly to the Gemini model.
    """
    try:
        # Generate an answer directly from Gemini
        prompt = f"""You are a helpful AI assistant for the AI Humanoid Robotics Book.
        
Question: {request.question}

Please provide a helpful and accurate answer based on your knowledge of AI and robotics."""
        
        answer = await gemini_client.generate_answer(prompt, request.question)

        return ChatResponse(answer=answer, sources="General Knowledge")

    except Exception as e:
        print(f"Error in /api/chat/general: {e}")
        raise HTTPException(status_code=500, detail=f"Error generating response: {str(e)}")


@app.post("/api/chat/selected", response_model=ChatResponse)
async def chat_selected_text(request: ChatRequest):
    """
    This is the main RAG endpoint. It uses the user-selected text
    to find similar passages and generate a focused answer.
    """
    try:
        if not request.context:
            raise ValueError("Context (selected text) is required for RAG chat")

        # Search for similar passages in Qdrant
        similar_passages = await qdrant_client.search_similar_passages(request.context)

        # Build context from original text and search results
        context_for_llm = f"Selected Text:\n{request.context}\n\n"
        citations = set()

        if similar_passages:
            context_for_llm += "Related Passages:\n---\n"
            for passage in similar_passages:
                context_for_llm += passage.payload.get('text', '') + "\n"
                if 'source' in passage.payload:
                    citations.add(passage.payload['source'])

        # Generate answer based on context
        prompt = f"""You are a helpful assistant for the AI Humanoid Robotics Book.
        
Based ONLY on the following text from the book, answer the user's question.
If the answer is not in the text, say you don't have enough information.
Keep your answer concise and directly related to the provided text.

Context:
---
{context_for_llm}
---

Question: {request.question}

Answer:"""
        
        answer = await gemini_client.generate_answer(prompt, request.question)

        # Format sources
        sources_str = ", ".join(list(citations)) if citations else "From selected text"

        return ChatResponse(answer=answer, sources=sources_str)

    except Exception as e:
        print(f"Error in /api/chat/selected: {e}")
        raise HTTPException(status_code=500, detail=f"Error processing selected text: {str(e)}")

