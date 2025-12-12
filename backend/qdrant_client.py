import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

load_dotenv()

# --- Qdrant Client Initialization ---
qdrant = None
QDRANT_COLLECTION_NAME = None

try:
    QDRANT_URL = os.getenv("QDRANT_URL")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
    QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "ai-humanoid-robotics-book")

    if not QDRANT_URL or not QDRANT_API_KEY:
        raise ValueError("QDRANT_URL or QDRANT_API_KEY not found in environment variables.")

    qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    print("✓ Successfully initialized Qdrant client.")

except Exception as e:
    print(f"✗ Error initializing Qdrant client: {e}")
    qdrant = None

async def search_similar_passages(text: str, limit: int = 3):
    """
    Searches for relevant passages in Qdrant based on the input text.
    """
    if not qdrant:
        raise RuntimeError("Qdrant client is not initialized.")

    try:
        # Import here to avoid circular dependency
        from gemini_client import get_embedding
        
        query_vector = await get_embedding(text)
        if not query_vector:
            raise ValueError("Failed to generate query vector from Gemini.")

        search_results = qdrant.search(
            collection_name=QDRANT_COLLECTION_NAME,
            query_vector=query_vector,
            limit=limit,
            with_payload=True,
        )
        return search_results
    except Exception as e:
        print(f"Error searching Qdrant: {e}")
        return []