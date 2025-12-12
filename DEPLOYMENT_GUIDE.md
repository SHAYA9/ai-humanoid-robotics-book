# RAG Chatbot - Deployment Guide

Deploy your FastAPI RAG backend to production with these simple steps.

## Option 1: Deploy to Render.com (⭐ Recommended - Free Tier)

### Step 1: Prepare Your Repository
1. Push your code to GitHub (if not already done):
```bash
git add .
git commit -m "Add RAG backend deployment files"
git push origin main
```

### Step 2: Create Render Account
1. Go to [render.com](https://render.com)
2. Sign up with GitHub
3. Click "New +" → "Web Service"
4. Connect your GitHub repository
5. Fill in the details:
   - **Name**: `ai-robotics-rag-api`
   - **Environment**: Python 3
   - **Build Command**: `pip install -r backend/requirements.txt`
   - **Start Command**: `cd backend && uvicorn main:app --host 0.0.0.0 --port $PORT`
   - **Plan**: Free (12-hour auto-sleep)

### Step 3: Add Environment Variables
In Render dashboard → Environment:
```
GEMINI_API_KEY=your_gemini_api_key
QDRANT_URL=https://your-qdrant-cluster.url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLlection_NAME=ai-humanoid-robotics-book
FRONTEND_URL=https://shaya9.github.io/ai-humanoid-robotics-book
```

### Step 4: Deploy
Click "Create Web Service" and wait for deployment (2-3 minutes).

Your API will be at: `https://ai-robotics-rag-api.onrender.com`

---

## Option 2: Deploy to Railway.app (Better Uptime)

### Step 1: Create Railway Account
1. Go to [railway.app](https://railway.app)
2. Sign up with GitHub
3. Create new project

### Step 2: Deploy
```bash
npm i -g @railway/cli
railway login
railway link
railway up
```

### Step 3: Set Environment Variables
In Railway dashboard → Variables:
```
GEMINI_API_KEY=your_gemini_api_key
QDRANT_URL=https://your-qdrant-cluster.url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLlection_NAME=ai-humanoid-robotics-book
FRONTEND_URL=https://shaya9.github.io/ai-humanoid-robotics-book
PORT=8000
```

Your API will be automatically assigned a URL (e.g., `https://railway-app.up.railway.app`)

---

## Option 3: Deploy to PythonAnywhere (No Sleep)

### Step 1: Create Account
1. Go to [pythonanywhere.com](https://pythonanywhere.com)
2. Create free account

### Step 2: Upload Code
1. Create new web app
2. Choose "Python 3.11"
3. Upload your `backend/` folder

### Step 3: Configure WSGI
Edit the WSGI file:
```python
import sys
path = '/home/yourusername/ai-robotics-book/backend'
if path not in sys.path:
    sys.path.append(path)

from main import application
```

### Step 4: Add Environment Variables
In Web app settings → Environment variables:
```
GEMINI_API_KEY=your_gemini_api_key
QDRANT_URL=https://your-qdrant-cluster.url
QDRANT_API_KEY=your_qdrant_api_key
```

Reload web app and you're live!

---

## Update Frontend to Use Live API

Edit `src/components/Chatbot/Chatbot.js`:

```javascript
const getApiUrl = () => {
  // Use live API in production
  if (typeof window !== 'undefined' && window.location.hostname !== 'localhost') {
    return 'https://your-deployed-api.onrender.com'; // Replace with your API URL
  }
  return 'http://localhost:8000'; // Local dev
};
```

Or set as environment variable in `docusaurus.config.ts`:
```javascript
process.env.VITE_API_URL = 'https://your-deployed-api.onrender.com'
```

---

## Testing Your Live Backend

After deployment, test the endpoints:

### 1. Health Check
```bash
curl https://your-api.onrender.com/health
```
Response: `{"status": "healthy"}`

### 2. General Chat
```bash
curl -X POST https://your-api.onrender.com/api/chat/general \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
```

### 3. RAG Chat (Selected Text)
```bash
curl -X POST https://your-api.onrender.com/api/chat/selected \
  -H "Content-Type: application/json" \
  -d '{"question": "Explain this further", "context": "ROS 2 is a middleware..."}'
```

---

## Troubleshooting

### "GEMINI_API_KEY not found"
✅ Check environment variables are set in deployment platform

### "Qdrant client not initialized"
✅ Verify QDRANT_URL and QDRANT_API_KEY are correct

### "ModuleNotFoundError"
✅ Ensure `backend/requirements.txt` is complete

### CORS Errors
✅ Update `ALLOWED_ORIGINS` in `backend/main.py` with your frontend URL

---

## Comparison Table

| Feature | Render | Railway | PythonAnywhere |
|---------|--------|---------|-----------------|
| **Free Tier** | Yes (12h sleep) | Yes (low hours) | Yes |
| **Auto-wake** | No | Optional | Always on |
| **Setup Time** | 5 min | 5 min | 10 min |
| **Recommended** | ✅ Yes | ✅ Yes | Good backup |

**→ Go with Render.com for easiest setup!**

---

## Next Steps

1. ✅ Deploy backend to production
2. ✅ Update frontend API URL
3. ✅ Test RAG endpoints
4. ✅ Share live chatbot with users
