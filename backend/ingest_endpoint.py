"""
Qdrant ingestion module for populating the vector database with documentation.
This can be triggered via API endpoint or run as a standalone script.
"""
import os
import glob
import asyncio
from qdrant_client import QdrantClient, models
from dotenv import load_dotenv

load_dotenv()

# Configuration
# Try multiple possible paths for docs folder
POSSIBLE_DOCS_PATHS = [
    "docs/**/*.md",           # If running from project root
    "../docs/**/*.md",        # If running from backend folder
    "/app/docs/**/*.md",      # Docker absolute path
]
CHUNK_SIZE = 1000
CHUNK_OVERLAP = 150

def chunk_text(text):
    """Splits text into overlapping chunks."""
    chunks = []
    for i in range(0, len(text), CHUNK_SIZE - CHUNK_OVERLAP):
        chunk = text[i : i + CHUNK_SIZE]
        if chunk.strip():
            chunks.append(chunk)
    return chunks

async def ingest_documents_to_qdrant():
    """
    Main ingestion function that can be called from API or script.
    Returns a dict with status and details.
    """
    try:
        # Import here to avoid circular dependencies
        from qdrant_service import qdrant, QDRANT_COLLECTION_NAME, initialize_qdrant
        from ai_service import get_embedding
        
        # Ensure Qdrant is initialized
        if not qdrant:
            print("Qdrant not initialized, attempting to initialize...")
            if not initialize_qdrant():
                return {
                    "success": False,
                    "error": "Failed to initialize Qdrant client",
                    "details": "Check QDRANT_URL and QDRANT_API_KEY environment variables"
                }
        
        print(f"Starting document ingestion to collection: {QDRANT_COLLECTION_NAME}")
        
        # Step 1: Recreate collection
        print("Step 1/5: Recreating Qdrant collection...")
        try:
            qdrant.recreate_collection(
                collection_name=QDRANT_COLLECTION_NAME,
                vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),
            )
            print("✓ Collection recreated")
        except Exception as e:
            return {
                "success": False,
                "error": f"Failed to recreate collection: {str(e)}",
                "step": 1
            }
        
        # Step 2: Find markdown files
        print("Step 2/5: Finding markdown files...")
        
        # Try different paths
        file_paths = []
        tried_paths = []
        
        for docs_path in POSSIBLE_DOCS_PATHS:
            tried_paths.append(docs_path)
            found = glob.glob(docs_path, recursive=True)
            if found:
                file_paths = found
                print(f"✓ Found {len(file_paths)} markdown files in: {docs_path}")
                break
            else:
                print(f"  No files found in: {docs_path}")
        
        if not file_paths:
            # Debug: List current directory contents
            import os
            cwd = os.getcwd()
            dir_contents = os.listdir(cwd)
            
            return {
                "success": False,
                "error": "No markdown files found",
                "details": {
                    "tried_paths": tried_paths,
                    "current_directory": cwd,
                    "directory_contents": dir_contents[:20]  # First 20 items
                },
                "step": 2
            }
        
        # Step 3: Create chunks
        print("Step 3/5: Creating text chunks...")
        all_chunks = []
        all_metadata = []
        
        for path in file_paths:
            try:
                with open(path, 'r', encoding='utf-8') as f:
                    content = f.read()
                
                # Extract metadata
                path_parts = path.replace('\\', '/').split('/')
                module = path_parts[-2] if len(path_parts) > 2 else 'general'
                page = os.path.basename(path).replace('.md', '')
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
                
            except Exception as e:
                print(f"Warning: Error processing {path}: {e}")
                continue
        
        if not all_chunks:
            return {
                "success": False,
                "error": "No chunks created from markdown files",
                "step": 3
            }
        
        print(f"✓ Created {len(all_chunks)} chunks from {len(file_paths)} files")
        
        # Step 4: Generate embeddings
        print(f"Step 4/5: Generating embeddings for {len(all_chunks)} chunks...")
        embeddings = []
        
        # Process in batches to avoid memory issues
        batch_size = 50
        for i in range(0, len(all_chunks), batch_size):
            batch = all_chunks[i:i + batch_size]
            print(f"  Processing batch {i//batch_size + 1}/{(len(all_chunks) + batch_size - 1)//batch_size}")
            
            for chunk in batch:
                try:
                    embedding = await get_embedding(chunk)
                    if embedding:
                        embeddings.append(embedding)
                    else:
                        print(f"  Warning: Failed to generate embedding for chunk")
                except Exception as e:
                    print(f"  Warning: Error generating embedding: {e}")
                    continue
                
                # Small delay to avoid rate limits
                await asyncio.sleep(0.1)
        
        if len(embeddings) == 0:
            return {
                "success": False,
                "error": "Failed to generate any embeddings",
                "step": 4
            }
        
        print(f"✓ Generated {len(embeddings)} embeddings")
        
        # Trim metadata to match embeddings if there's a mismatch
        if len(embeddings) < len(all_metadata):
            all_metadata = all_metadata[:len(embeddings)]
        
        # Step 5: Upsert to Qdrant
        print(f"Step 5/5: Upserting {len(embeddings)} points to Qdrant...")
        
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
            
            # Verify
            collection_info = qdrant.get_collection(QDRANT_COLLECTION_NAME)
            points_count = collection_info.points_count
            
            print(f"✓ Successfully ingested {points_count} points")
            
            return {
                "success": True,
                "files_processed": len(file_paths),
                "chunks_created": len(all_chunks),
                "embeddings_generated": len(embeddings),
                "points_uploaded": points_count,
                "collection_name": QDRANT_COLLECTION_NAME
            }
            
        except Exception as e:
            return {
                "success": False,
                "error": f"Failed to upsert to Qdrant: {str(e)}",
                "step": 5
            }
    
    except Exception as e:
        return {
            "success": False,
            "error": f"Unexpected error during ingestion: {str(e)}",
            "details": str(type(e).__name__)
        }

# Synchronous wrapper for use in FastAPI
def ingest_documents_sync():
    """Synchronous wrapper for the async ingestion function."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        result = loop.run_until_complete(ingest_documents_to_qdrant())
        return result
    finally:
        loop.close()

if __name__ == "__main__":
    # Can be run as standalone script
    print("Running document ingestion...")
    result = ingest_documents_sync()
    print("\n" + "="*60)
    if result["success"]:
        print("✓ Ingestion completed successfully!")
        print(f"  Files processed: {result['files_processed']}")
        print(f"  Chunks created: {result['chunks_created']}")
        print(f"  Points uploaded: {result['points_uploaded']}")
    else:
        print("✗ Ingestion failed!")
        print(f"  Error: {result['error']}")
    print("="*60)