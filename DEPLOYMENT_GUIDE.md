# Deployment Guide: RAG Chatbot on GitHub Pages

## Disclaimer
⚠️ **Important**: GitHub Pages hosts static content only. The FastAPI backend must be deployed separately to a server that supports Python.

## Overview

Your RAG Chatbot will be deployed in two parts:

1. **Frontend** (React/Docusaurus) → GitHub Pages (free)
2. **Backend** (FastAPI) → External hosting service (free options available)

## Part 1: Frontend Deployment to GitHub Pages

### Prerequisites
- GitHub account with the repository `ai-humanoid-robotics-book`
- Repository already set up for GitHub Pages

### Build Process

1. **Install dependencies:**
```bash
npm install
```

2. **Build the static site:**
```bash
npm run build
```

3. **Deploy to GitHub Pages:**
```bash
npm run deploy
```

This will:
- Build the Docusaurus site
- Push to the `gh-pages` branch
- Automatically deploy to `https://shaya9.github.io/ai-humanoid-robotics-book`

### Verify Deployment
Visit: https://shaya9.github.io/ai-humanoid-robotics-book

## Part 2: Backend Deployment

### Option 1: Render (Recommended - Free with limitations)

**Pros**: Free tier, automatic deployments, HTTPS included
**Cons**: Spins down after 15 mins of inactivity (free tier)

1. **Create account** at https://render.com

2. **Create a new "Web Service"**:
   - Connect your GitHub repository
   - Set build command: `pip install -r requirements.txt`
   - Set start command: `uvicorn backend.main:app --host 0.0.0.0 --port $PORT`

3. **Set environment variables** in dashboard:
   - `GEMINI_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `FRONTEND_URL=https://shaya9.github.io/ai-humanoid-robotics-book`

4. **Update frontend**:
```javascript
// docusaurus.config.js or .env
VITE_API_URL = "https://your-render-service.onrender.com"
```

### Option 2: Railway (Free with credit)

1. **Create account** at https://railway.app

2. **Connect GitHub repository**


3. **Create Python environment**:
   - Railway auto-detects requirements.txt
   - Set start command: `uvicorn backend.main:app --host 0.0.0.0 --port $PORT`

4. **Set environment variables**

### Option 3: PythonAnywhere (Free tier available)

1. **Sign up** at https://www.pythonanywhere.com (free account)

2. **Upload code** via Git:
```bash
cd /home/yourname
git clone https://github.com/SHAYA9/ai-humanoid-robotics-book.git
```

3. **Create virtual environment**:
```bash
mkvirtualenv --python=/usr/bin/python3.9 chatbot
pip install -r requirements.txt
```

4. **Configure WSGI** in PythonAnywhere dashboard:
```python
# /var/www/yourname_pythonanywhere_com_wsgi.py
import sys
sys.path.insert(0, '/home/yourname/ai-humanoid-robotics-book')

from backend.main import app as application
```

5. **Set environment variables** in WSGI file or use `.env`

6. **Reload** the web app

### Option 4: AWS Lambda + API Gateway (Free tier)

1. **Install serverless framework**:
```bash
npm install -g serverless
```

2. **Create serverless config**:
```yaml
# serverless.yml
service: rag-chatbot

provider:
  name: aws
  runtime: python3.9
  region: us-east-1
  environment:
    GEMINI_API_KEY: ${env:GEMINI_API_KEY}
    QDRANT_URL: ${env:QDRANT_URL}
    QDRANT_API_KEY: ${env:QDRANT_API_KEY}

functions:
  api:
    handler: backend.main.app
    events:
      - http:
          path: /{proxy+}
          method: ANY

plugins:
  - serverless-python-requirements
```

3. **Deploy**:
```bash
serverless deploy
```

## Configuration for GitHub Pages Frontend

### Update environment variable in your build:

**Option A: Environment variable file**
```bash
# .env.production
VITE_API_URL=https://your-backend-url.com
```

**Option B: In docusaurus.config.js**
```javascript
module.exports = {
  // ... other config
  customFields: {
    apiUrl: process.env.API_URL || 'http://localhost:8000'
  }
}
```

Then use in React:
```javascript
const API_URL = process.env.VITE_API_URL || 'http://localhost:8000'
```

### Update CORS in backend

Make sure your `backend/main.py` includes your GitHub Pages URL:

```python
ALLOWED_ORIGINS = [
    "https://shaya9.github.io",
    "https://shaya9.github.io/ai-humanoid-robotics-book",
    "http://localhost:3000",  # for local dev
]
```

## Complete Deployment Checklist

- [ ] Backend API deployed and running
- [ ] Environment variables set on backend
- [ ] Qdrant cluster created and API key configured
- [ ] Gemini API key configured
- [ ] Documents ingested into Qdrant: `python scripts/ingest_book_to_qdrant.py`
- [ ] CORS origins updated to include GitHub Pages URL
- [ ] Frontend `.env` updated with backend API URL
- [ ] Run `npm run build` locally and test
- [ ] Push to `gh-pages` branch: `npm run deploy`
- [ ] Visit your GitHub Pages URL and test chatbot
- [ ] Test text selection and RAG features

## Health Checks

### Backend Health
```bash
curl https://your-backend-url.com/
# Should return: {"status": "ok", ...}

curl https://your-backend-url.com/health
# Should return: {"status": "healthy"}
```

### Frontend
1. Open https://shaya9.github.io/ai-humanoid-robotics-book
2. Check browser console for errors (F12)
3. Test chatbot FAB appears (bottom right corner)
4. Select text from documentation
5. Click "💡 Ask about this"
6. Submit a question

## Monitoring

### Render/Railway
- View logs in dashboard
- Monitor API requests and errors
- Check response times

### PythonAnywhere
- Check Error log under Web tab
- Monitor CPU and memory usage
- Review access logs

### Manual Health Check Script
```python
import requests
import json

def health_check(api_url):
    try:
        # Check if API is up
        response = requests.get(f"{api_url}/health", timeout=5)
        print(f"API Status: {response.status_code}")
        
        # Test a sample question
        payload = {"question": "What is robotics?"}
        response = requests.post(
            f"{api_url}/api/chat/general",
            json=payload,
            timeout=10
        )
        
        if response.status_code == 200:
            data = response.json()
            print(f"✓ API working: Got answer with {len(data['answer'])} chars")
        else:
            print(f"✗ API error: {response.status_code}")
            
    except Exception as e:
        print(f"✗ Connection failed: {e}")

if __name__ == "__main__":
    health_check("https://your-api-url.com")
```

## Troubleshooting Deployment

### Issue: CORS errors in browser
```
Access to XMLHttpRequest has been blocked by CORS policy
```
**Solution**: Update `ALLOWED_ORIGINS` in backend and redeploy

### Issue: API returns 404
```
{
  "detail": "Not Found"
}
```
**Solution**: Check if backend is running and endpoint exists: `curl https://api-url/api/chat/general`

### Issue: Timeout errors
**Solution**: 
- Free tier services may be slow
- Use a keep-alive service: https://betterstack.com/
- Upgrade to paid tier

### Issue: Empty responses from Gemini
**Solution**: 
- Check API key is valid
- Check Gemini quota hasn't been exceeded
- Verify model name (`gemini-1.5-flash` or `gemini-pro`)

## Cost Analysis

| Service | Cost | Notes |
|---------|------|-------|
| GitHub Pages | FREE | Static hosting |
| Qdrant Cloud | FREE (up to 1GB) | Vector database |
| Gemini API | $0.075/1M tokens | Or FREE tier (~60 req/min) |
| Render | FREE (limited) | Sleeps after 15 min inactivity |
| Railway | FREE trial + credit | $5/month typical |
| PythonAnywhere | FREE | 100MB disk, limited CPU |
| **Total** | **~$0-5/month** | Easily deployable |

## Next Steps

1. Choose a backend hosting option
2. Set up environment variables
3. Deploy backend
4. Update frontend `.env`
5. Run `npm run deploy` to push to GitHub Pages
6. Test all features
7. Monitor logs and performance

## Support

For deployment help:
- Check service status pages
- Review API documentation
- Test with `curl` commands
- Enable debug logging in FastAPI

---

**Remember**: Always keep your API keys secure and never commit `.env` files to GitHub!
