# Railway Deployment Guide for RAG Chatbot

## 🚂 Quick Railway Setup

### 1. Prerequisites
- Railway account (https://railway.app)
- GitHub repository with your code
- Qdrant Cloud account with collection created
- Gemini API key

### 2. Deploy to Railway

#### Option A: Deploy from GitHub (Recommended)
1. Go to https://railway.app
2. Click "New Project" → "Deploy from GitHub repo"
3. Select your repository: `ai-humanoid-robotics-book`
4. Railway will auto-detect the Python app

#### Option B: Deploy via Railway CLI
```bash
# Install Railway CLI
npm i -g @railway/cli

# Login
railway login

# Initialize project
railway init

# Deploy
railway up
```

### 3. Configure Environment Variables

In Railway dashboard, add these variables:

```bash
# Gemini API
GEMINI_API_KEY=your_gemini_api_key_here

# Qdrant Vector Database
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=ai-humanoid-robotics-book

# Frontend URL (update after deployment)
FRONTEND_URL=https://shaya9.github.io

# Optional: Database (if using Neon)
NEON_DATABASE_URL=postgresql://user:pass@host/db
```

### 4. Configure Build Settings

Railway should auto-detect, but verify:

**Start Command:**
```bash
cd backend && uvicorn main:app --host 0.0.0.0 --port $PORT
```

**Build Command:**
```bash
pip install -r backend/requirements.txt
```

**Root Directory:** `.` (project root)

### 5. Health Check Configuration

Railway will use the `/health` endpoint automatically:
- Path: `/health`
- Timeout: 100s
- Interval: 30s

### 6. Get Your Railway URL

After deployment:
1. Go to your Railway project
2. Click on your service
3. Go to "Settings" → "Domains"
4. Copy the generated URL (e.g., `your-app.railway.app`)

### 7. Update Frontend Configuration

Update `src/components/Chatbot/Chatbot.js`:

```javascript
// Line 16 - Update with your Railway URL
if (window.location.hostname === 'shaya9.github.io') {
  return 'https://your-app.railway.app'; // ← Update this
}
```

### 8. Update CORS in Backend

Update `backend/main.py` line 33:

```python
ALLOWED_ORIGINS = [
    "http://localhost:3000",
    "http://localhost:3001",
    "http://localhost:8000",
    "https://shaya9.github.io",
    "https://your-app.railway.app",  # Add Railway URL
    os.getenv("FRONTEND_URL", "").strip() or None
]
```

## 🔍 Verification Steps

### 1. Test Backend Health
```bash
curl https://your-app.railway.app/health
# Expected: {"status": "healthy"}
```

### 2. Test API Root
```bash
curl https://your-app.railway.app/
# Expected: {"status": "ok", "message": "Welcome to the RAG Chatbot API!", ...}
```

### 3. Test General Chat
```bash
curl -X POST https://your-app.railway.app/api/chat/general \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
```

### 4. Test RAG Chat
```bash
curl -X POST https://your-app.railway.app/api/chat/selected \
  -H "Content-Type: application/json" \
  -d '{
    "question": "Explain this concept",
    "context": "ROS 2 is the next generation Robot Operating System"
  }'
```

## 📊 Railway-Specific Features

### Automatic Deployments
- Push to `main` branch → Auto-deploys
- Pull requests → Preview deployments

### Logs & Monitoring
```bash
# View logs via CLI
railway logs

# Or in Railway dashboard:
# Project → Service → Deployments → View Logs
```

### Scaling
Railway auto-scales based on:
- CPU usage
- Memory usage
- Request volume

Free tier limits:
- $5 credit/month
- 500MB RAM
- 1GB disk

### Database Integration
If using Railway's PostgreSQL:
```bash
# Add PostgreSQL service
railway add postgresql

# It auto-creates DATABASE_URL variable
```

## 🐛 Troubleshooting

### Issue: "Module not found" errors
**Solution:** Ensure `railway.json` has correct paths:
```json
{
  "build": {
    "buildCommand": "pip install -r backend/requirements.txt"
  },
  "deploy": {
    "startCommand": "cd backend && uvicorn main:app --host 0.0.0.0 --port $PORT"
  }
}
```

### Issue: CORS errors from frontend
**Solution:** 
1. Check `ALLOWED_ORIGINS` includes your frontend URL
2. Verify Railway URL is correct in frontend config
3. Check Railway logs for CORS-related errors

### Issue: Qdrant connection fails
**Solution:**
1. Verify `QDRANT_URL` and `QDRANT_API_KEY` in Railway env vars
2. Check Qdrant collection exists: `ai-humanoid-robotics-book`
3. Test Qdrant connection locally first

### Issue: Gemini API errors
**Solution:**
1. Verify `GEMINI_API_KEY` is correct
2. Check API quota at https://aistudio.google.com
3. Ensure using correct model: `gemini-1.5-flash`

### Issue: Port binding errors
**Solution:** Railway auto-assigns `$PORT`. Ensure your start command uses it:
```bash
uvicorn main:app --host 0.0.0.0 --port $PORT
```

### Issue: Import errors in Python
**Solution:** Fixed in latest code - imports are now absolute, not relative:
```python
# ✅ Correct
import qdrant_client as qc
import gemini_client as gc

# ❌ Wrong (old code)
from . import qdrant_client as qc
```

## 📈 Performance Optimization

### 1. Enable Caching
Add Redis to Railway:
```bash
railway add redis
```

### 2. Optimize Qdrant Queries
```python
# Limit search results
search_results = qdrant.search(
    collection_name=QDRANT_COLLECTION_NAME,
    query_vector=query_vector,
    limit=3,  # Adjust based on needs
    with_payload=True
)
```

### 3. Use Connection Pooling
Already configured in `qdrant_client.py`

### 4. Monitor Response Times
Check Railway metrics:
- Average response time
- P95/P99 latency
- Error rates

## 💰 Cost Estimation

**Railway Free Tier:**
- $5/month credit
- Suitable for development/testing
- ~100-500 requests/day

**Railway Hobby Plan ($5/month):**
- 500GB bandwidth
- 8GB RAM
- Suitable for production
- ~10,000+ requests/day

**Qdrant Cloud Free Tier:**
- 1GB storage
- ~100,000 vectors
- Suitable for documentation

**Gemini API Free Tier:**
- 60 requests/minute
- 1,500 requests/day
- Free for testing

## 🔐 Security Best Practices

1. **Never commit `.env` files**
2. **Use Railway's secret management** for API keys
3. **Enable HTTPS only** (Railway does this by default)
4. **Implement rate limiting** in production
5. **Monitor API usage** to prevent abuse

## 📚 Additional Resources

- Railway Docs: https://docs.railway.app
- Qdrant Docs: https://qdrant.tech/documentation
- Gemini API: https://ai.google.dev/docs
- FastAPI Docs: https://fastapi.tiangolo.com

## ✅ Deployment Checklist

- [ ] Railway account created
- [ ] GitHub repo connected to Railway
- [ ] All environment variables set
- [ ] Qdrant collection created and populated
- [ ] Gemini API key obtained
- [ ] Backend deployed successfully
- [ ] Health check passing
- [ ] API endpoints tested
- [ ] Frontend updated with Railway URL
- [ ] CORS configured correctly
- [ ] Frontend redeployed to GitHub Pages
- [ ] End-to-end testing completed

## 🎉 Success!

Once all steps are complete, your RAG chatbot should be live at:
- **Backend API:** `https://your-app.railway.app`
- **Frontend:** `https://shaya9.github.io/ai-humanoid-robotics-book`

Test by selecting text on your documentation and asking questions!