import os
import glob
import time
import google.generativeai as genai
from qdrant_client import QdrantClient, models
from dotenv import load_dotenv

load_dotenv()  # Load from current directory or parent

# --- Load Environment Variables ---
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "ai-humanoid-robotics-book")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")  # Changed from GOOGLE_API_KEY

if not all([QDRANT_URL, QDRANT_API_KEY, QDRANT_COLLECTION_NAME, GEMINI_API_KEY]):
    raise ValueError("One or more required environment variables are missing.")

print(f"Configuration:")
print(f"  Qdrant URL: {QDRANT_URL}")
print(f"  Collection: {QDRANT_COLLECTION_NAME}")
print(f"  Gemini API Key: {'*' * 20}{GEMINI_API_KEY[-4:]}")

# --- Initialize Clients ---
genai.configure(api_key=GEMINI_API_KEY)
qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# --- Configuration ---
DOCS_PATH = "../docs/**/*.md"
CHUNK_SIZE = 1000  # characters
CHUNK_OVERLAP = 150 # characters
BATCH_SIZE = 100 # number of points to upsert at once

def chunk_text(text):
    """Splits text into overlapping chunks."""
    return [
        text[i : i + CHUNK_SIZE]
        for i in range(0, len(text), CHUNK_SIZE - CHUNK_OVERLAP)
    ]

def get_all_embeddings(texts):
    """Gets embeddings for a list of texts, handling API rate limits."""
    embeddings = []
    for i in range(0, len(texts), BATCH_SIZE):
        batch = texts[i : i + BATCH_SIZE]
        try:
            result = genai.embed_content(model="models/text-embedding-004", content=batch)
            embeddings.extend(result['embedding'])
        except Exception as e:
            print(f"Error getting embeddings, retrying in 20 seconds: {e}")
            time.sleep(20)
            result = genai.embed_content(model="models/text-embedding-004", content=batch)
            embeddings.extend(result['embedding'])
        print(f"Embedded batch {i//BATCH_SIZE + 1}...")
    return embeddings

def main():
    print("--- Starting Document Ingestion into Qdrant ---")

    # 1. Recreate Qdrant collection to ensure it's fresh
    print(f"Recreating Qdrant collection: '{QDRANT_COLLECTION_NAME}'")
    qdrant.recreate_collection(
        collection_name=QDRANT_COLLECTION_NAME,
        vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),
    )

    # 2. Find and process all markdown files
    points = []
    file_paths = glob.glob(DOCS_PATH, recursive=True)
    print(f"Found {len(file_paths)} markdown files to process.")

    all_chunks = []
    all_metadata = []

    for path in file_paths:
        with open(path, 'r', encoding='utf-8') as f:
            content = f.read()

        path_parts = path.replace('\\', '/').split('/')
        module = path_parts[-2] if len(path_parts) > 2 else 'general'
        page = os.path.basename(path).replace('.md', '')
        source = f"{module}/{page}"

        chunks = chunk_text(content)
        for chunk in chunks:
            all_chunks.append(chunk)
            all_metadata.append({
                "text": chunk,
                "source": source,
                "module": module,
                "page": page
            })

    print(f"Created {len(all_chunks)} text chunks.")

    # 3. Get all embeddings
    print("Generating embeddings for all text chunks...")
    embeddings = get_all_embeddings(all_chunks)

    # 4. Prepare points for Qdrant
    points = [
        models.PointStruct(
            id=idx,
            vector=embedding,
            payload=metadata,
        )
        for idx, (embedding, metadata) in enumerate(zip(embeddings, all_metadata))
    ]

    # 5. Upsert points to Qdrant in batches
    print(f"Upserting {len(points)} points to Qdrant...")
    qdrant.upsert(
        collection_name=QDRANT_COLLECTION_NAME,
        points=points,
        wait=True,
    )

    print("--- Document Ingestion Complete ---")
    print(f"Total points in collection: {qdrant.get_collection(QDRANT_COLLECTION_NAME).points_count}")

if __name__ == "__main__":
    main()
