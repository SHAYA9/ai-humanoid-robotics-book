import os
import glob
import qdrant_client
from qdrant_client.http import models
import google.generativeai as genai
from dotenv import load_dotenv
import markdown

# --- Configuration ---
load_dotenv()
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
QDRANT_COLLECTION_NAME = "ai-humanoid-robotics-book"
DOCS_PATH = "docs"  # Path to your Docusaurus markdown files

# --- Initialize Clients ---
genai.configure(api_key=GEMINI_API_KEY)
embedding_model = genai.get_generative_model(model="text-embedding-004")
qdrant = qdrant_client.QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

def get_embedding(text):
    return genai.embed_content(model="models/text-embedding-004", content=text)['embedding']

def ingest_docs():
    """Reads markdown files, chunks them, creates embeddings, and upserts to Qdrant."""
    # Check if collection exists, create if not
    try:
        qdrant.get_collection(collection_name=QDRANT_COLLECTION_NAME)
    except Exception:
        qdrant.recreate_collection(
            collection_name=QDRANT_COLLECTION_NAME,
            vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),
        )

    # Process all markdown files
    md_files = glob.glob(os.path.join(DOCS_PATH, "**/*.md"), recursive=True)
    points = []

    for file_path in md_files:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
            html = markdown.markdown(content) # Basic conversion to strip markdown syntax for cleaner text

            # Simple chunking by paragraph
            chunks = html.split('\n\n')

            for i, chunk in enumerate(chunks):
                if not chunk.strip():
                    continue

                # Create embedding
                vector = get_embedding(chunk)

                # Prepare payload
                module, page = os.path.split(file_path)
                payload = {
                    "text": chunk,
                    "module": os.path.basename(module),
                    "page": page,
                    "chunk_id": i
                }

                points.append(
                    models.PointStruct(
                        id=f"{file_path}-{i}",
                        vector=vector,
                        payload=payload
                    )
                )

    # Batch upsert to Qdrant
    if points:
        qdrant.upsert(
            collection_name=QDRANT_COLLECTION_NAME,
            points=points,
            wait=True,
        )
        print(f"Successfully ingested {len(points)} points from {len(md_files)} files.")

if __name__ == "__main__":
    ingest_docs()
