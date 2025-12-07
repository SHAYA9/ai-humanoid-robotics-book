import os
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import backend modules
from . import qdrant_client, gemini_client, mysql_client

# --- FastAPI App Initialization ---
app = FastAPI(
    title="AI Humanoid Robotics Book - RAG Chatbot API",
    description="A RAG chatbot backend using FastAPI, running on PythonAnywhere.",
    version="1.0.0"
)

# --- CORS Configuration ---
# Docusaurus dev server runs on 3000, GitHub Pages is more variable
# PythonAnywhere free accounts are your-username.pythonanywhere.com
ALLOWED_ORIGINS = [
    "http://localhost:3000",
    "http://localhost:3001",
    os.getenv("FRONTEND_URL") # Your GitHub Pages URL
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=[origin for origin in ALLOWED_ORIGINS if origin],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- Pydantic Models ---
class ChatRequest(BaseModel):
    question: str
    context: str # The user-selected text from the frontend

class GeneralChatRequest(BaseModel):
    question: str

class ChatResponse(BaseModel):
    answer: str
    sources: str

# --- API Endpoints ---
@app.on_event("startup")
async def startup_event():
    """On startup, create the MySQL table if it doesn't exist."""
    mysql_client.create_chat_history_table()

@app.get("/")
def read_root():
    """Health check endpoint."""
    return {"status": "ok", "message": "Welcome to the RAG Chatbot API!"}


@app.post("/api/chat/general", response_model=ChatResponse)
async def chat_general(request: GeneralChatRequest):
    """
    Endpoint for general, non-RAG chat.
    Forwards the question directly to the Gemini model.
    """
    try:
        # 1. Generate an answer directly from Gemini
        # We pass a simple prompt template for better instructions
        prompt = f"You are a helpful assistant. Answer the following question: {request.question}"
        answer = await gemini_client.generate_answer(prompt, request.question)

        # 2. Save the interaction to the database (no sources)
        mysql_client.save_chat_message(request.question, answer, "General Inquiry")

        return ChatResponse(answer=answer, sources="General Inquiry")

    except Exception as e:
        print(f"Error in /api/chat/general: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/chat/selected", response_model=ChatResponse)
async def chat_selected_text(request: ChatRequest):
    """
    This is the main RAG endpoint. It uses ONLY the user-selected text
    to find similar passages and generate a focused answer.
    """
    try:
        # 1. Use the selected text to find similar passages from the book in Qdrant.
        # This provides more context around the user's selection.
        similar_passages = await qdrant_client.search_similar_passages(request.context)

        # 2. Combine the original selected text with the search results to create a rich context.
        context_for_llm = request.context + "\n\n--- Relevant Passages ---\n"
        citations = set()

        for passage in similar_passages:
            context_for_llm += passage.payload['text'] + "\n"
            # Assuming payload contains citation info
            if 'source' in passage.payload:
                citations.add(passage.payload['source'])

        # 3. Call Gemini to generate an answer based ONLY on this combined context.
        answer = await gemini_client.generate_answer(context_for_llm, request.question)

        # 4. Format sources and save the interaction to the database.
        sources_str = ", ".join(list(citations)) if citations else "From selected text"
        mysql_client.save_chat_message(request.question, answer, sources_str)

        return ChatResponse(answer=answer, sources=sources_str)

    except Exception as e:
        print(f"Error in /api/chat/selected: {e}")
        raise HTTPException(status_code=500, detail=str(e))
