import os
import glob
import time
import asyncio
import aiohttp
from qdrant_client import QdrantClient, models
from dotenv import load_dotenv

load_dotenv()

# --- Load Environment Variables ---
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "ai-humanoid-robotics-book")
QWEN_API_KEY = os.getenv("QWEN_API_KEY")
QWEN_API_BASE = os.getenv("QWEN_API_BASE", "https://openrouter.ai/api/v1")

# Clean up API base URL
QWEN_API_BASE = QWEN_API_BASE.rstrip('/')
if QWEN_API_BASE.endswith('/chat/completions'):
    QWEN_API_BASE = QWEN_API_BASE.replace('/chat/completions', '')

if not all([QDRANT_URL, QDRANT_API_KEY, QDRANT_COLLECTION_NAME, QWEN_API_KEY]):
    raise ValueError("One or more required environment variables are missing.")

# --- Initialize Qdrant Client ---
qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# --- Configuration ---
DOCS_PATH = "docs/**/*.md"
CHUNK_SIZE = 1000  # characters
CHUNK_OVERLAP = 150  # characters
BATCH_SIZE = 10  # Smaller batch size for free tier
EMBEDDING_MODEL = "text-embedding-3-small"  # OpenRouter embedding model

def chunk_text(text):
    """Splits text into overlapping chunks."""
    chunks = []
    for i in range(0, len(text), CHUNK_SIZE - CHUNK_OVERLAP):
        chunk = text[i : i + CHUNK_SIZE]
        if chunk.strip():  # Only add non-empty chunks
            chunks.append(chunk)
    return chunks

async def get_embedding_batch(texts):
    """Gets embeddings for a batch of texts using OpenRouter."""
    url = f"{QWEN_API_BASE}/embeddings"
    headers = {
        "Authorization": f"Bearer {QWEN_API_KEY}",
        "Content-Type": "application/json",
        "HTTP-Referer": "https://shaya9.github.io/ai-humanoid-robotics-book",
        "X-Title": "AI Humanoid Robotics Book"
    }
    
    payload = {
        "model": EMBEDDING_MODEL,
        "input": texts
    }
    
    async with aiohttp.ClientSession() as session:
        async with session.post(url, headers=headers, json=payload) as response:
            if response.status == 200:
                result = await response.json()
                return [item["embedding"] for item in result["data"]]
            else:
                error_text = await response.text()
                raise Exception(f"Error getting embeddings: {response.status} - {error_text}")

async def get_all_embeddings(texts):
    """Gets embeddings for all texts with rate limiting."""
    embeddings = []
    total_batches = (len(texts) + BATCH_SIZE - 1) // BATCH_SIZE
    
    for i in range(0, len(texts), BATCH_SIZE):
        batch = texts[i : i + BATCH_SIZE]
        batch_num = i // BATCH_SIZE + 1
        
        try:
            print(f"Processing batch {batch_num}/{total_batches}...")
            batch_embeddings = await get_embedding_batch(batch)
            embeddings.extend(batch_embeddings)
            
            # Rate limiting - wait between batches
            if i + BATCH_SIZE < len(texts):
                await asyncio.sleep(2)  # 2 second delay between batches
                
        except Exception as e:
            print(f"Error in batch {batch_num}, retrying in 10 seconds: {e}")
            await asyncio.sleep(10)
            try:
                batch_embeddings = await get_embedding_batch(batch)
                embeddings.extend(batch_embeddings)
            except Exception as retry_error:
                print(f"Retry failed for batch {batch_num}: {retry_error}")
                # Skip this batch and continue
                continue
    
    return embeddings

async def main():
    print("=" * 60)
    print("Starting Document Ingestion into Qdrant")
    print("=" * 60)
    print(f"Qdrant URL: {QDRANT_URL}")
    print(f"Collection: {QDRANT_COLLECTION_NAME}")
    print(f"API Base: {QWEN_API_BASE}")
    print(f"Embedding Model: {EMBEDDING_MODEL}")
    print("=" * 60)

    # 1. Recreate Qdrant collection
    print(f"\n[1/5] Recreating collection '{QDRANT_COLLECTION_NAME}'...")
    try:
        qdrant.recreate_collection(
            collection_name=QDRANT_COLLECTION_NAME,
            vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
        )
        print("✓ Collection created successfully")
    except Exception as e:
        print(f"✗ Error creating collection: {e}")
        return

    # 2. Find and process markdown files
    print(f"\n[2/5] Finding markdown files in '{DOCS_PATH}'...")
    file_paths = glob.glob(DOCS_PATH, recursive=True)
    print(f"✓ Found {len(file_paths)} markdown files")

    if not file_paths:
        print("✗ No markdown files found. Check your DOCS_PATH.")
        return

    # 3. Create chunks
    print(f"\n[3/5] Creating text chunks...")
    all_chunks = []
    all_metadata = []

    for path in file_paths:
        try:
            with open(path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract metadata from path
            path_parts = path.replace('\\', '/').split('/')
            
            # Get module name (parent directory)
            if len(path_parts) > 2:
                module = path_parts[-2]
            else:
                module = 'general'
            
            # Get page name
            page = os.path.basename(path).replace('.md', '')
            
            # Create source reference
            source = f"{module}/{page}"

            # Create chunks
            chunks = chunk_text(content)
            
            for chunk in chunks:
                all_chunks.append(chunk)
                all_metadata.append({
                    "text": chunk,
                    "source": source,
                    "module": module,
                    "page": page
                })
            
            print(f"  ✓ Processed {path} → {len(chunks)} chunks")
            
        except Exception as e:
            print(f"  ✗ Error processing {path}: {e}")
            continue

    print(f"✓ Created {len(all_chunks)} text chunks from {len(file_paths)} files")

    if not all_chunks:
        print("✗ No chunks created. Check your markdown files.")
        return

    # 4. Generate embeddings
    print(f"\n[4/5] Generating embeddings for {len(all_chunks)} chunks...")
    print("This may take a while depending on the number of chunks...")
    
    try:
        embeddings = await get_all_embeddings(all_chunks)
        print(f"✓ Generated {len(embeddings)} embeddings")
    except Exception as e:
        print(f"✗ Error generating embeddings: {e}")
        return

    if len(embeddings) != len(all_chunks):
        print(f"⚠ Warning: Mismatch between chunks ({len(all_chunks)}) and embeddings ({len(embeddings)})")
        # Trim to match
        min_len = min(len(embeddings), len(all_chunks))
        embeddings = embeddings[:min_len]
        all_metadata = all_metadata[:min_len]

    # 5. Upsert to Qdrant
    print(f"\n[5/5] Upserting {len(embeddings)} points to Qdrant...")
    
    try:
        points = [
            models.PointStruct(
                id=idx,
                vector=embedding,
                payload=metadata,
            )
            for idx, (embedding, metadata) in enumerate(zip(embeddings, all_metadata))
        ]
        
        qdrant.upsert(
            collection_name=QDRANT_COLLECTION_NAME,
            points=points,
            wait=True,
        )
        
        print("✓ Successfully upserted all points")
        
        # Verify
        collection_info = qdrant.get_collection(QDRANT_COLLECTION_NAME)
        print(f"\n{'=' * 60}")
        print("Document Ingestion Complete!")
        print(f"{'=' * 60}")
        print(f"Total points in collection: {collection_info.points_count}")
        print(f"Vector size: {collection_info.config.params.vectors.size}")
        print(f"{'=' * 60}")
        
    except Exception as e:
        print(f"✗ Error upserting to Qdrant: {e}")
        return

if __name__ == "__main__":
    asyncio.run(main())