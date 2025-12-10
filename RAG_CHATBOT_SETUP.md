# RAG Chatbot Integration Guide

## Overview

This document describes the Retrieval-Augmented Generation (RAG) chatbot system integrated into the AI Humanoid Robotics Book. The chatbot allows users to:

1. **Select text from the book** and ask questions about it
2. **Ask general questions** about AI and robotics
3. **Get source-attributed answers** from the documentation

## Architecture

```
┌─────────────────┐
│   Frontend      │
│ (Docusaurus +   │
│  React Chatbot) │
└────────┬────────┘
         │
    HTTP │ (JSON)
         │
    ┌────▼─────────────────┐
    │   FastAPI Backend    │
    │   (main.py)          │
    └────┬────────┬────────┘
         │        │
    ┌────▼──┐  ┌──▼──────────┐
    │ Gemini│  │ Qdrant Vector│
    │  API  │  │   Database   │
    └───────┘  └──────────────┘
```

## Technologies

- **Frontend**: React + Docusaurus
- **Backend API**: FastAPI (Python)
- **LLM**: Google Gemini API
- **Vector DB**: Qdrant Cloud (Free Tier)
- **Database**: Neon PostgreSQL (Free Tier)
- **Embeddings**: Gemini Text Embedding model

## Setup Instructions

### 1. Backend Setup

#### Prerequisites
- Python 3.9+
- pip or poetry

#### Install Dependencies
```bash
cd backend
pip install -r requirements.txt
```

#### Environment Variables
Copy `.env.example` to `.env` and fill in your API keys:

```bash
# .env
GEMINI_API_KEY=your_gemini_api_key
QDRANT_URL=https://your-qdrant-cluster.com
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=ai-humanoid-robotics-book
NEON_DATABASE_URL=postgresql://user:password@host/dbname
FRONTEND_URL=https://shaya9.github.io/ai-humanoid-robotics-book
```

#### Get API Keys

**Gemini API Key:**
1. Go to [Google AI Studio](https://aistudio.google.com/app/apikeys)
2. Create a new API key
3. Copy it to `.env`

**Qdrant Cloud:**
1. Sign up at [Qdrant Cloud](https://cloud.qdrant.io/)
2. Create a free cluster
3. Get your cluster URL and API key
4. Use `models/text-embedding-004` for embeddings

**Neon PostgreSQL:**
1. Sign up at [Neon](https://neon.tech/)
2. Create a new database
3. Copy the connection string to `.env`

### 2. Running the Backend Locally

```bash
cd backend
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

Test the API:
```bash
curl http://localhost:8000/
```

### 3. Document Ingestion

Ingest your documentation into Qdrant:

```bash
cd scripts
python ingest_book_to_qdrant.py
```

This script:
- Reads all markdown files from `docs/`
- Chunks them into manageable pieces
- Generates embeddings using Gemini
- Uploads them to Qdrant with metadata

### 4. Frontend Configuration

In `docusaurus.config.js`:
```javascript
// Update the chatbot API endpoint
process.env.VITE_API_URL = 'http://localhost:8000'  // for dev
// OR
process.env.VITE_API_URL = 'your-api-url'  // for production
```

## API Endpoints

### 1. General Chat
```
POST /api/chat/general
Content-Type: application/json

{
  "question": "What is ROS 2?"
}

Response:
{
  "answer": "ROS 2 is...",
  "sources": "General Knowledge"
}
```

### 2. Selected Text Chat (RAG)
```
POST /api/chat/selected
Content-Type: application/json

{
  "question": "Can you explain this further?",
  "context": "Selected text from the user..."
}

Response:
{
  "answer": "Based on the selected text...",
  "sources": "From selected text, module-1-ros2/overview"
}
```

## Frontend Integration

The chatbot is automatically embedded in every page via the `<Chatbot />` component in `src/theme/Root.js`.

### Features

1. **Text Selection**: Users can select text and click "Ask about this"
2. **Context-Aware Answers**: Selected text is sent as context to the RAG endpoint
3. **Source Attribution**: Answers display relevant sources
4. **Message History**: Conversation is maintained in the chat window
5. **Error Handling**: User-friendly error messages

### Chatbot Component Props

The chatbot component accepts no props and auto-initializes. It stores state locally.

```javascript
import Chatbot from '@site/src/components/Chatbot/Chatbot';

// In your page
<Chatbot />
```

## Document Chunking Strategy

The ingestion script chunks documents to optimize semantic search:

- **Chunk Size**: ~1000 characters per chunk
- **Overlap**: 150 characters (for context continuity)
- **Metadata**: Source file path and timestamp

```python
# Example chunk with metadata
{
  "id": 1,
  "text": "Chunk of documentation...",
  "source": "docs/module-1-ros2/overview.md",
  "timestamp": "2025-01-10T12:34:56Z"
}
```

## Deployment

### Deploy Backend to Production

**Option 1: PythonAnywhere**
1. Upload code to PythonAnywhere
2. Configure WSGI file
3. Set environment variables
4. Update `FRONTEND_URL` in `.env`

**Option 2: Render/Railway**
```bash
# Create procfile
web: uvicorn backend.main:app --host 0.0.0.0 --port $PORT
```

### Deploy Frontend to GitHub Pages

```bash
npm run build
# Commit and push to gh-pages branch
git push origin gh-pages
```

## Monitoring & Debugging

### Enable Debug Logging
```python
# In backend/main.py
import logging
logging.basicConfig(level=logging.DEBUG)
```

### Check Qdrant Collection Status
```python
from qdrant_client import QdrantClient
client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
info = client.get_collection("ai-humanoid-robotics-book")
print(f"Vectors in collection: {info.points_count}")
```

### Test Gemini API
```python
import google.generativeai as genai
genai.configure(api_key=GEMINI_API_KEY)
model = genai.GenerativeModel("gemini-1.5-flash")
response = model.generate_content("Test prompt")
print(response.text)
```

## Troubleshooting

### Issue: CORS errors in chatbot
**Solution**: Update `ALLOWED_ORIGINS` in `backend/main.py` with your GitHub Pages URL

### Issue: Empty responses from Gemini
**Solution**: Check if your Gemini API key has access to the model. Use `gemini-1.5-flash` or `gemini-pro`

### Issue: No search results from Qdrant
**Solution**: 
1. Verify documents are ingested: `python scripts/ingest_book_to_qdrant.py`
2. Check collection exists: `python -c "from backend import qdrant_client; print(qdrant_client.qdrant.get_collection('ai-humanoid-robotics-book'))"`
3. Verify API keys are correct

### Issue: Chatbot UI not appearing
**Solution**: 
1. Check browser console for errors
2. Ensure `<Chatbot />` is imported in your layout
3. Verify CSS is loading correctly

## Performance Tips

1. **Caching**: Implement Redis caching for common questions
2. **Chunking**: Optimize chunk size based on your documentation
3. **Rate Limiting**: Add rate limits to API endpoints
4. **Pagination**: Return search results in batches

## Security Considerations

1. **API Keys**: Never commit `.env` files
2. **CORS**: Whitelist only trusted domains
3. **Input Validation**: Sanitize user input on both frontend and backend
4. **Rate Limiting**: Implement to prevent abuse
5. **Authentication**: Consider adding user authentication for analytics

## Future Enhancements

- [ ] User authentication and chat history
- [ ] Analytics dashboard (questions asked, popular topics)
- [ ] Multi-language support
- [ ] Feedback mechanism (thumbs up/down for answers)
- [ ] Integration with Discord/Slack bots
- [ ] Fine-tuning Gemini on specific domain knowledge
- [ ] WebSocket support for real-time responses
- [ ] Streaming responses for long answers

## Support

For issues or questions:
1. Check the Troubleshooting section
2. Review backend logs: `tail -f backend.log`
3. Check Qdrant API status at https://cloud.qdrant.io/
4. Open an issue on GitHub

## License

This RAG implementation is part of the AI Humanoid Robotics Book project and follows the same license as the main project.
