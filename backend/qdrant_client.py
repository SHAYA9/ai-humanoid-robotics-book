import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load environment variables from .env file (for local development)
load_dotenv()

# --- Qdrant Client Initialization ---
qdrant = None
QDRANT_COLLECTION_NAME = None

def initialize_qdrant():
    """Initialize Qdrant client with environment variables"""
    global qdrant, QDRANT_COLLECTION_NAME
    
    try:
        QDRANT_URL = os.getenv("QDRANT_URL")
        QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
        QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "ai-humanoid-robotics-book")

        print(f"Attempting to initialize Qdrant...")
        print(f"QDRANT_URL present: {bool(QDRANT_URL)}")
        print(f"QDRANT_API_KEY present: {bool(QDRANT_API_KEY)}")
        print(f"Collection name: {QDRANT_COLLECTION_NAME}")

        if not QDRANT_URL:
            raise ValueError("QDRANT_URL not found in environment variables.")
        if not QDRANT_API_KEY:
            raise ValueError("QDRANT_API_KEY not found in environment variables.")

        qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
        
        # Test the connection
        collections = qdrant.get_collections()
        print(f"✓ Successfully initialized Qdrant client. Found {len(collections.collections)} collections.")
        
        return True

    except Exception as e:
        print(f"✗ Error initializing Qdrant client: {e}")
        print(f"Error type: {type(e).__name__}")
        import traceback
        traceback.print_exc()
        qdrant = None
        return False

# Try to initialize on module load
initialize_qdrant()

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