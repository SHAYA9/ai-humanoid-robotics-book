# Scripts Directory

## Overview
This directory contains utility scripts for managing the AI Humanoid Robotics Book backend.

## Scripts

### 1. `load_docs_to_qdrant.py` ⭐ **RECOMMENDED**
Populates Qdrant vector database with documentation embeddings using **Gemini API** (free & reliable).

**Usage:**
```bash
python load_docs_to_qdrant.py
```

**Requirements:**
- `QDRANT_URL` - Your Qdrant cloud URL
- `QDRANT_API_KEY` - Your Qdrant API key
- `QDRANT_COLLECTION_NAME` - Collection name (default: ai-humanoid-robotics-book)
- `GEMINI_API_KEY` - Your Google Gemini API key

**What it does:**
1. Recreates the Qdrant collection (clears old data)
2. Reads all markdown files from `docs/` directory
3. Chunks text into 1000-character pieces with 150-character overlap
4. Generates embeddings using Gemini's `text-embedding-004` model
5. Uploads to Qdrant with metadata (source, module, page)

---

### 2. `load_docs_to_qdrant_qwen.py`
Alternative script using **OpenRouter embeddings** instead of Gemini.

**Usage:**
```bash
python load_docs_to_qdrant_qwen.py
```

**Requirements:**
- Same as above, but uses `QWEN_API_KEY` and `QWEN_API_BASE`

**Note:** OpenRouter's free embedding models may have limitations. Use the Gemini script if you encounter issues.

---

### 3. `ingest_book_to_qdrant.py`
Legacy script for ingesting documentation. Use `load_docs_to_qdrant.py` instead.

---

### 4. `test_chatbot.py`
Test script for the chatbot API endpoints.

---

### 5. `setup_neon_tables.py`
Database setup script (if using Neon Postgres).

## Quick Start

### First Time Setup

1. **Make sure your `.env` file is configured** (in project root):
   ```bash
   # Qdrant Configuration
   QDRANT_URL=https://your-qdrant-url.cloud.qdrant.io
   QDRANT_API_KEY=your-qdrant-api-key
   QDRANT_COLLECTION_NAME=ai-humanoid-robotics-book
   
   # Gemini Configuration
   GEMINI_API_KEY=your-gemini-api-key
   ```

2. **Run the ingestion script**:
   ```bash
   cd scripts
   python load_docs_to_qdrant.py
   ```

3. **Wait for completion**. You should see:
   ```
   --- Document Ingestion Complete ---
   Total points in collection: [number]
   ```

4. **Test your chatbot** by selecting text and asking questions!

## Troubleshooting

### "No markdown files found"
- Make sure you're in the project root or scripts directory
- Check that `docs/` folder exists and contains `.md` files

### "GEMINI_API_KEY not found"
- Verify your `.env` file has the correct variable name
- Make sure `.env` is in the project root

### "Rate limit exceeded"
- Wait a few minutes and try again
- The script has built-in retry logic

### Chatbot still says "I do not have enough information"
- Verify Qdrant collection was created: Check Railway logs for "Found X collections"
- Re-run the ingestion script
- Make sure `QDRANT_COLLECTION_NAME` matches in both `.env` and Railway environment variables

## Maintenance

**When to re-run the ingestion script:**
- After adding new documentation pages
- After significant content updates
- If search results seem outdated

The script recreates the collection each time, so it's safe to run multiple times.