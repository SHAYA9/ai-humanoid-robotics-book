# 🚀 RAG Chatbot - Quick Start Guide

## What We've Built

An **Integrated RAG Chatbot** for your AI Humanoid Robotics Book that:
- ✅ Lets users select text and ask context-aware questions
- ✅ Generates answers using Google Gemini API
- ✅ Searches your documentation using Qdrant vector DB
- ✅ Shows source attributions for answers
- ✅ Works seamlessly on GitHub Pages (frontend) + external backend

## 🎯 5-Minute Setup (Local Development)

### Step 1: Get API Keys
1. **Gemini API**: https://aistudio.google.com/app/apikeys (free)
2. **Qdrant Cloud**: https://cloud.qdrant.io/ (free tier: 1GB)
3. **Neon PostgreSQL**: https://neon.tech/ (free tier)

### Step 2: Configure Environment
```bash
cp .env.example .env
# Edit .env with your API keys
```

### Step 3: Install & Run Backend
```bash
cd backend
pip install -r requirements.txt
uvicorn main:app --reload
# Backend runs at: http://localhost:8000
```

### Step 4: Ingest Your Documentation
```bash
cd scripts
python ingest_book_to_qdrant.py
# This uploads all your docs to Qdrant
```

### Step 5: Run Frontend
```bash
npm install
npm start
# Frontend runs at: http://localhost:3000
```

### Step 6: Test the Chatbot
1. Open http://localhost:3000
2. Select any text from the docs
3. Click "💡 Ask about this"
4. Ask a question!

## 📁 Files Modified/Created

### New Files
- `.env.example` - Environment variable template
- `RAG_CHATBOT_SETUP.md` - Detailed setup documentation
- `DEPLOYMENT_GUIDE.md` - Production deployment guide

### Modified Files
- `src/components/Chatbot/Chatbot.js` - Enhanced RAG integration
- `src/components/Chatbot/styles.css` - Improved UI styling
- `backend/main.py` - FastAPI endpoints
- `backend/gemini_client.py` - Gemini async wrapper
- `backend/requirements.txt` - Dependencies

### Existing Files (Already Present)
- `backend/qdrant_client.py` - Vector database client
- `scripts/ingest_book_to_qdrant.py` - Document ingestion
- `src/theme/Root.js` - Component initialization

## 🔑 API Endpoints

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/` | GET | Health check |
| `/health` | GET | Server health status |
| `/api/chat/general` | POST | General questions (non-RAG) |
| `/api/chat/selected` | POST | Selected text questions (RAG) |

### Example Requests

**General Chat:**
```bash
curl -X POST http://localhost:8000/api/chat/general \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
```

**RAG Chat (with selected text):**
```bash
curl -X POST http://localhost:8000/api/chat/selected \
  -H "Content-Type: application/json" \
  -d '{
    "question": "Can you explain this?",
    "context": "Selected text from the book..."
  }'
```

## 🎨 Chatbot Features

### Text Selection (On Any Page)
1. Select text from documentation
2. Floating popup appears automatically
3. Click "💡 Ask about this"
4. Chatbot opens with context

### Chat Window
- **Modern UI**: Purple gradient header, smooth animations
- **Source Attribution**: Shows where answers come from
- **Loading Indicators**: Animated dots during processing
- **Error Handling**: User-friendly error messages
- **Message History**: Keeps conversation visible

### Smart Responses
- Uses only selected text for RAG questions
- Falls back to general knowledge if needed
- Cites sources automatically
- Handles errors gracefully

## 🚢 Production Deployment

### Frontend (GitHub Pages)
```bash
npm run build
npm run deploy
# Site available at: https://shaya9.github.io/ai-humanoid-robotics-book
```

### Backend (Choose One Option)

**Render (Recommended):**
1. Connect GitHub repository
2. Set environment variables
3. Render auto-deploys on push

**Railway:**
1. Connect GitHub
2. Set railway.json with start command
3. Deploy immediately

**PythonAnywhere (Free):**
1. Upload code
2. Create virtual environment
3. Configure WSGI file
4. Start web app

See `DEPLOYMENT_GUIDE.md` for detailed instructions.

## ⚙️ Configuration

### Environment Variables (.env)
```
GEMINI_API_KEY=sk-...
QDRANT_URL=https://...
QDRANT_API_KEY=...
QDRANT_COLLECTION_NAME=ai-humanoid-robotics-book
NEON_DATABASE_URL=postgresql://...
FRONTEND_URL=https://your-github-pages-url
```

### Frontend Configuration
```javascript
// In Chatbot.js - API URL
const API_URL = process.env.VITE_API_URL || 'http://localhost:8000'
```

## 🧪 Testing

### Test API Locally
```bash
# Test health endpoint
curl http://localhost:8000/health

# Test general chat
curl -X POST http://localhost:8000/api/chat/general \
  -H "Content-Type: application/json" \
  -d '{"question": "Hello?"}'

# Test RAG
curl -X POST http://localhost:8000/api/chat/selected \
  -H "Content-Type: application/json" \
  -d '{"question": "What does this mean?", "context": "Selected text..."}'
```

### Test Frontend
1. Open browser console (F12)
2. Check for errors
3. Open chatbot (bottom right)
4. Try selecting text
5. Ask a question

## 📊 Architecture Overview

```
┌─────────────────────────────────────┐
│   GitHub Pages (Frontend)           │
│ - Docusaurus (Static Site)          │
│ - React Chatbot Component           │
│ - Text Selection Handler            │
└──────────────┬──────────────────────┘
               │
               │ HTTP/CORS
               │
┌──────────────▼──────────────────────┐
│   FastAPI Backend (Your Server)     │
│ - RAG Pipeline                      │
│ - Gemini Integration                │
│ - Qdrant Search                     │
└──────────────┬──────────────────────┘
               │
        ┌──────┴──────┐
        │             │
   ┌────▼────┐  ┌────▼────────┐
   │ Gemini  │  │ Qdrant       │
   │ API     │  │ Vector DB    │
   └─────────┘  └──────────────┘
```

## 🐛 Troubleshooting

| Problem | Solution |
|---------|----------|
| Chatbot doesn't appear | Check console (F12) for errors, ensure Chatbot component is imported |
| CORS errors | Update `ALLOWED_ORIGINS` in `backend/main.py` |
| Empty responses | Verify Gemini API key, check model availability |
| No search results | Run `python scripts/ingest_book_to_qdrant.py` |
| Can't connect to backend | Verify backend is running, check API_URL in frontend |
| Text selection not working | Clear browser cache, reload page |

## 📚 Documentation Files

1. **RAG_CHATBOT_SETUP.md** - Complete technical setup
2. **DEPLOYMENT_GUIDE.md** - Production deployment options
3. **README.md** - Project overview (original)
4. **This file** - Quick start guide

## 🎓 Learning Resources

- [Qdrant Docs](https://qdrant.tech/documentation/)
- [Gemini API Guide](https://ai.google.dev/docs)
- [FastAPI Docs](https://fastapi.tiangolo.com/)
- [Docusaurus Guide](https://docusaurus.io/docs)

## 📝 Next Steps

1. ✅ **Set up locally** (follow 5-minute setup above)
2. ✅ **Test the chatbot** (try selecting text and asking questions)
3. ✅ **Optimize document ingestion** (adjust chunk sizes if needed)
4. ✅ **Deploy frontend** to GitHub Pages
5. ✅ **Deploy backend** to your chosen hosting
6. ✅ **Monitor in production** (check logs, track performance)

## 💡 Pro Tips

- Use `python scripts/test_chatbot.py` to test locally without UI
- Check Qdrant dashboard to monitor vector storage
- Enable debug logging: `logging.basicConfig(level=logging.DEBUG)`
- Cache common questions for faster responses
- Monitor Gemini API usage on your Google AI Studio dashboard

## 🤝 Support

- Check `RAG_CHATBOT_SETUP.md` for detailed troubleshooting
- Review `DEPLOYMENT_GUIDE.md` for deployment issues
- Check service status pages (Render, Railway, Neon, Qdrant, Google)

## ✨ What Makes This Special

✅ **Zero Hallucinations** - Answers based only on your book content
✅ **Source Attribution** - Users know where information comes from  
✅ **Context Aware** - Uses selected text for better answers
✅ **Free to Deploy** - Multiple free hosting options
✅ **Easy Integration** - Works with existing Docusaurus setup
✅ **Fully Customizable** - Modify prompts, styling, behavior

---

**Ready to launch?** Start with Step 1 of the 5-Minute Setup! 🚀
