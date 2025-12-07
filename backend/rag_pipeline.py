import os
import re
from typing import List, Dict
from dotenv import load_dotenv
from qdrant_client import QdrantClient, models
from sentence_transformers import SentenceTransformer
import openai
import anthropic
import google.generativeai as genai

load_dotenv()

# --- Configuration ---
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
LLM_PROVIDER = os.getenv("LLM_PROVIDER", "gemini").lower()
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
ANTHROPIC_API_KEY = os.getenv("ANTHROPIC_API_KEY")
GOOGLE_API_KEY = os.getenv("GOOGLE_API_KEY")
COLLECTION_NAME = "humanoid_robotics_book"
EMBEDDING_MODEL = "all-MiniLM-L6-v2"

# --- Client Initialization ---
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
embedding_model = SentenceTransformer(EMBEDDING_MODEL)
gemini_model = None

if LLM_PROVIDER == "openai":
    openai.api_key = OPENAI_API_KEY
    if not OPENAI_API_KEY:
        raise ValueError("OPENAI_API_KEY is not set for the OpenAI provider.")
elif LLM_PROVIDER == "anthropic":
    if not ANTHROPIC_API_KEY:
        raise ValueError("ANTHROPIC_API_KEY is not set for the Anthropic provider.")
    anthropic_client = anthropic.Anthropic(api_key=ANTHROPIC_API_KEY)
elif LLM_PROVIDER == "gemini":
    if not GOOGLE_API_KEY:
        raise ValueError("GOOGLE_API_KEY is not set for the Gemini provider.")
    genai.configure(api_key=GOOGLE_API_KEY)
    gemini_model = genai.GenerativeModel('gemini-pro')
else:
    raise ValueError(f"Unsupported LLM provider: {LLM_PROVIDER}")

# --- Core Functions ---

def text_to_chunks(text: str, chunk_size: int = 1000, overlap: int = 150) -> List[str]:
    """Splits a long text into smaller chunks."""
    sentences = re.split(r'(?<=[.!?])\s+', text)
    chunks = []
    current_chunk = ""
    for sentence in sentences:
        if len(current_chunk) + len(sentence) < chunk_size:
            current_chunk += " " + sentence
        else:
            chunks.append(current_chunk.strip())
            current_chunk = sentence
    if current_chunk:
        chunks.append(current_chunk.strip())
    return chunks

def ingest_content(source_name: str, content: str):
    """Chunks, embeds, and uploads content to Qdrant."""
    qdrant_client.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=models.VectorParams(
            size=embedding_model.get_sentence_embedding_dimension(),
            distance=models.Distance.COSINE
        ),
    )

    chunks = text_to_chunks(content)
    vectors = embedding_model.encode(chunks, show_progress_bar=True)

    qdrant_client.upsert(
        collection_name=COLLECTION_NAME,
        points=models.Batch(
            ids=[i for i in range(len(chunks))],
            vectors=vectors.tolist(),
            payloads=[
                {"text": chunk, "source": source_name} for chunk in chunks
            ]
        ),
        wait=True
    )
    return {"status": "success", "chunks_ingested": len(chunks)}

def ask_question(question: str, history: List[Dict] = None) -> Dict:
    """Finds relevant context in Qdrant and generates an answer using an LLM."""
    query_vector = embedding_model.encode(question).tolist()

    search_results = qdrant_client.search(
        collection_name=COLLECTION_NAME,
        query_vector=query_vector,
        limit=3
    )

    context = "\n".join([result.payload["text"] for result in search_results])
    source = search_results[0].payload["source"] if search_results else "No source found"

    prompt = f"""
    You are an intelligent chatbot for the 'Physical AI & Humanoid Robotics' book.
    Based on the following context, please answer the user's question.
    If the context doesn't contain the answer, say you don't know.

    Context:
    {context}

    Conversation History:
    {history or "No history"}

    Question:
    {question}

    Answer:
    """

    return _generate_answer(prompt, source)


def ask_question_on_selection(question: str, selected_text: str) -> Dict:
    """Generates an answer based on user-selected text."""
    prompt = f"""
    You are an intelligent chatbot for the 'Physical AI & Humanoid Robotics' book.
    Based ONLY on the provided text selection below, answer the user's question.

    Selected Text:
    {selected_text}

    Question:
    {question}

    Answer:
    """
    return _generate_answer(prompt, "User's text selection")


def _generate_answer(prompt: str, source: str) -> Dict:
    """Helper to call the configured LLM provider."""
    answer = "Sorry, I could not generate a response."
    try:
        if LLM_PROVIDER == "openai":
            response = openai.Completion.create(
                engine="text-davinci-003",
                prompt=prompt,
                max_tokens=250,
                temperature=0.2,
                n=1
            )
            answer = response.choices[0].text.strip()

        elif LLM_PROVIDER == "anthropic":
            message = anthropic_client.messages.create(
                model="claude-2.1",
                max_tokens=300,
                messages=[{"role": "user", "content": prompt}]
            )
            answer = message.content[0].text

        elif LLM_PROVIDER == "gemini":
            response = gemini_model.generate_content(prompt)
            answer = response.text

        return {"answer": answer, "source": source}

    except Exception as e:
        print(f"Error calling LLM provider: {e}")
        # Extract more detailed error message for Gemini if available
        if LLM_PROVIDER == "gemini" and hasattr(e, 'message'):
             answer = f"Sorry, I encountered an error with the Gemini API: {e.message}"
        else:
             answer = "Sorry, I encountered an error while generating a response."
        return {"answer": answer, "source": "error"}

if __name__ == '__main__':
    # Example usage (for testing)
    # Ensure you have a .env file with the correct API keys
    if not all([QDRANT_URL, QDRANT_API_KEY, GOOGLE_API_KEY]):
        print("Please ensure QDRANT_URL, QDRANT_API_KEY, and GOOGLE_API_KEY are set in your .env file.")
    else:
        test_content = "The humanoid robot Atlas, developed by Boston Dynamics, is known for its advanced mobility and balance. It can perform complex gymnastic routines. ROS 2 is the latest version of the Robot Operating System."
        ingest_content("Chapter 1: Intro", test_content)

        response = ask_question("Tell me about the Atlas robot.")
        print("--- General Question Response ---")
        print(response)

        selected_response = ask_question_on_selection("What is ROS 2?", "ROS 2 is the new generation of the Robot Operating System, designed for modern robotics applications.")
        print("\n--- Selected Text Response ---")
        print(selected_response)
