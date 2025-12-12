# Railway Dashboard Configuration Guide

## 🎯 Problem: Railway is Running Frontend Instead of Backend

Your logs show Railway is trying to run `npm start` (Docusaurus frontend) instead of the Python backend.

---

## ✅ Fix in Railway Dashboard (5 Minutes)

### Step 1: Open Railway Dashboard
1. Go to https://railway.app
2. Select your project: `ai-humanoid-robotics-book`
3. Click on your service

### Step 2: Configure Settings

#### A. Go to "Settings" Tab

#### B. Scroll to "Build" Section
```
Root Directory: .
(Leave empty or use ".")
```

#### C. Scroll to "Deploy" Section

**Build Command:**
```bash
pip install -r backend/requirements.txt
```

**Start Command:**
```bash
cd backend && uvicorn main:app --host 0.0.0.0 --port $PORT
```

**Watch Paths:** (Optional)
```
backend/**
```

#### D. Scroll to "Environment" Section

Make sure these variables are set:
```
GEMINI_API_KEY=your_key_here
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_key
QDRANT_COLLECTION_NAME=ai-humanoid-robotics-book
FRONTEND_URL=https://shaya9.github.io
```

### Step 3: Save & Redeploy

1. Click "Save" (if there's a save button)
2. Go to "Deployments" tab
3. Click "Redeploy" or trigger new deployment

---

## 🔍 Verify It's Working

### Check Logs (Should See This):
```
✅ Installing dependencies from backend/requirements.txt
✅ Successfully installed fastapi uvicorn qdrant-client...
✅ Starting server...
✅ INFO:     Started server process
✅ INFO:     Uvicorn running on http://0.0.0.0:8080
✅ INFO:     Application startup complete
```

### NOT This (Old Wrong Logs):
```
❌ npm start
❌ docusaurus start
❌ Killed
```

---

## 🧪 Test Your Deployed Backend

Once logs show "Uvicorn running":

### 1. Get Your Railway URL
- In Railway dashboard → "Settings" → "Domains"
- Copy the URL (e.g., `your-app.railway.app`)

### 2. Test Health Endpoint
Open browser or use curl:
```bash
https://your-app.railway.app/health
```

**Expected Response:**
```json
{"status": "healthy"}
```

### 3. Test Root Endpoint
```bash
https://your-app.railway.app/
```

**Expected Response:**
```json
{
  "status": "ok",
  "message": "Welcome to the RAG Chatbot API!",
  "version": "1.0.0"
}
```

### 4. Test Chat Endpoint
```bash
curl -X POST https://your-app.railway.app/api/chat/general \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
```

**Expected Response:**
```json
{
  "answer": "ROS 2 is...",
  "sources": "General Knowledge"
}
```

---

## 🎯 Alternative: Use Railway CLI

If dashboard doesn't work, use CLI:

### Install Railway CLI
```bash
npm i -g @railway/cli
```

### Login
```bash
railway login
```

### Link to Project
```bash
railway link
# Select your project
```

### Set Environment Variables
```bash
railway variables set GEMINI_API_KEY=your_key
railway variables set QDRANT_URL=your_url
railway variables set QDRANT_API_KEY=your_key
railway variables set QDRANT_COLLECTION_NAME=ai-humanoid-robotics-book
```

### Deploy
```bash
railway up
```

---

## 📊 Settings Summary

Copy-paste these exact values into Railway:

| Setting | Value |
|---------|-------|
| **Root Directory** | `.` (or empty) |
| **Build Command** | `pip install -r backend/requirements.txt` |
| **Start Command** | `cd backend && uvicorn main:app --host 0.0.0.0 --port $PORT` |
| **Builder** | NIXPACKS |
| **Python Version** | 3.11 (from runtime.txt) |

---

## ⚠️ If Still Not Working

### Option 1: Delete & Recreate Service
1. Delete current service in Railway
2. Create new service
3. **IMPORTANT:** Set start command BEFORE first deploy
4. Deploy

### Option 2: Create Backend-Only Repo
1. Create new GitHub repo: `ai-humanoid-robotics-backend`
2. Copy only `backend/` folder contents to root
3. Deploy that repo to Railway

### Option 3: Use Dockerfile
Create `Dockerfile` in root:
```dockerfile
FROM python:3.11-slim

WORKDIR /app

COPY backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY backend/ .

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8080"]
```

Then in Railway:
- Builder: DOCKERFILE
- Dockerfile Path: `./Dockerfile`

---

## 🎉 Success Checklist

After fixing, you should have:

- [ ] Railway logs show "Uvicorn running"
- [ ] `/health` endpoint returns `{"status": "healthy"}`
- [ ] `/` endpoint returns welcome message
- [ ] No "npm start" or "docusaurus" in logs
- [ ] No "Killed" messages
- [ ] Backend responds to API calls

---

## 📞 Next Steps After Backend is Running

1. **Copy your Railway URL** (e.g., `your-app.railway.app`)

2. **Update frontend** (`src/components/Chatbot/Chatbot.js`):
   ```javascript
   if (window.location.hostname === 'shaya9.github.io') {
     return 'https://your-app.railway.app'; // ← Your Railway URL
   }
   ```

3. **Deploy frontend to GitHub Pages:**
   ```bash
   npm run build
   npm run deploy
   ```

4. **Test end-to-end:**
   - Visit: `https://shaya9.github.io/ai-humanoid-robotics-book/`
   - Select text
   - Ask question
   - Get response from Railway backend

---

**Remember:** 
- Railway = Python Backend ONLY
- GitHub Pages = React Frontend ONLY
- They communicate via HTTPS API calls