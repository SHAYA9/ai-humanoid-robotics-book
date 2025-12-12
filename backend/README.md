# Backend - RAG Chatbot API

## 📁 Structure

```
backend/
├── main.py              # FastAPI application with API endpoints
├── gemini_client.py     # Google Gemini API integration
├── qdrant_client.py     # Qdrant vector database client
├── requirements.txt     # Python dependencies
├── runtime.txt          # Python version (3.11.7)
└── Procfile            # Railway deployment command
```

## 🚀 Quick Start

### Local Development
```bash
cd backend
pip install -r requirements.txt
uvicorn main:app --reload
```

### Environment Variables Required
```bash
GEMINI_API_KEY=your_gemini_key
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_key
QDRANT_COLLECTION_NAME=ai-humanoid-robotics-book
FRONTEND_URL=https://shaya9.github.io
```

## 📡 API Endpoints

### Health Check
```
GET /health
Response: {"status": "healthy"}
```

### Root
```
GET /
Response: {"status": "ok", "message": "Welcome to the RAG Chatbot API!", "version": "1.0.0"}
```

### General Chat
```
POST /api/chat/general
Body: {"question": "What is ROS 2?"}
Response: {"answer": "...", "sources": "General Knowledge"}
```

### RAG Chat (Selected Text)
```
POST /api/chat/selected
Body: {"question": "Explain this", "context": "Selected text..."}
Response: {"answer": "...", "sources": "module-1-ros2/overview"}
```

## 🔧 Dependencies

- **fastapi** - Web framework
- **uvicorn** - ASGI server
- **qdrant-client** - Vector database
- **google-generativeai** - Gemini API
- **python-dotenv** - Environment variables
- **pydantic** - Data validation
- **aiohttp** - Async HTTP client

## 📦 Deployment

### Railway
```bash
# Automatic deployment via GitHub
# Start command: cd backend && uvicorn main:app --host 0.0.0.0 --port $PORT
```

See `../RAILWAY_DEPLOYMENT.md` for detailed instructions.

## 🧹 Recent Changes

- ✅ Removed unused database clients (MySQL, Neon, SQLAlchemy)
- ✅ Removed duplicate RAG pipeline implementation
- ✅ Cleaned up dependencies (removed 8 unused packages)
- ✅ Optimized for Railway deployment
- ✅ 54% reduction in file count (13 → 6 files)

## 📚 Documentation

- [Railway Deployment Guide](../RAILWAY_DEPLOYMENT.md)
- [RAG Implementation Review](../RAG_IMPLEMENTATION_REVIEW.md)
- [Backend Cleanup Summary](../BACKEND_CLEANUP_SUMMARY.md)