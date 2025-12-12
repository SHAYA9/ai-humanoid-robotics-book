# Railway Deployment Fix

## 🔴 Problem
Railway is detecting `package.json` and trying to run the **frontend (Docusaurus)** instead of the **backend (FastAPI)**.

Your logs show:
```
npm start
docusaurus start
Killed
```

This is wrong! Railway should run your Python backend, not the Node.js frontend.

---

## ✅ Solution

### Method 1: Configure in Railway Dashboard (Recommended)

1. **Go to Railway Dashboard**
   - Open your project
   - Click on your service

2. **Settings → Build & Deploy**
   - **Root Directory:** Leave as `.` (project root)
   - **Build Command:** `pip install -r backend/requirements.txt`
   - **Start Command:** `cd backend && uvicorn main:app --host 0.0.0.0 --port $PORT`

3. **Settings → Environment**
   - **Builder:** Select `NIXPACKS` or `DOCKERFILE`

4. **Redeploy**
   - Click "Deploy" → "Redeploy"

---

### Method 2: Use nixpacks.toml (Already Created)

I've created `nixpacks.toml` in your root directory. Railway will automatically detect it.

**What it does:**
- Tells Railway to use Python 3.11
- Installs dependencies from `backend/requirements.txt`
- Runs the FastAPI backend

**To apply:**
1. Commit the new `nixpacks.toml` file
2. Push to GitHub
3. Railway will auto-redeploy with correct settings

---

### Method 3: Create Separate Backend Repo (Alternative)

If Railway keeps detecting the frontend:

1. **Create new repo:** `ai-humanoid-robotics-backend`
2. **Copy only backend files:**
   ```
   backend/
   ├── main.py
   ├── gemini_client.py
   ├── qdrant_client.py
   ├── requirements.txt
   └── Procfile
   ```
3. **Deploy that repo to Railway**

---

## 🔍 Verify Correct Deployment

After fixing, your Railway logs should show:

```
✅ Installing Python dependencies...
✅ pip install -r backend/requirements.txt
✅ Starting uvicorn server...
✅ INFO:     Started server process
✅ INFO:     Uvicorn running on http://0.0.0.0:8080
```

**NOT:**
```
❌ npm start
❌ docusaurus start
❌ Killed
```

---

## 🧪 Test Your Backend

Once deployed correctly:

```bash
# Test health endpoint
curl https://your-app.railway.app/health

# Expected response:
{"status": "healthy"}

# Test root endpoint
curl https://your-app.railway.app/

# Expected response:
{
  "status": "ok",
  "message": "Welcome to the RAG Chatbot API!",
  "version": "1.0.0"
}
```

---

## 📝 Railway Settings Checklist

In Railway Dashboard, verify:

- [ ] **Root Directory:** `.` (or empty)
- [ ] **Build Command:** `pip install -r backend/requirements.txt`
- [ ] **Start Command:** `cd backend && uvicorn main:app --host 0.0.0.0 --port $PORT`
- [ ] **Builder:** NIXPACKS (not Node.js)
- [ ] **Python Version:** 3.11 (from runtime.txt)

---

## 🎯 What Should Be Where

```
GitHub Pages (Frontend)
├── Docusaurus site
├── React chatbot UI
└── Static files
    ↓ (calls API)
    
Railway (Backend ONLY)
├── FastAPI server
├── Gemini integration
├── Qdrant client
└── Python dependencies
```

---

## ⚠️ Common Mistakes

1. **Deploying entire repo to Railway**
   - ❌ Railway tries to run `npm start`
   - ✅ Configure to run only backend

2. **Wrong start command**
   - ❌ `npm start`
   - ✅ `cd backend && uvicorn main:app --host 0.0.0.0 --port $PORT`

3. **Missing environment variables**
   - Make sure these are set in Railway:
     - `GEMINI_API_KEY`
     - `QDRANT_URL`
     - `QDRANT_API_KEY`
     - `QDRANT_COLLECTION_NAME`

---

## 🚀 Quick Fix Steps

1. **In Railway Dashboard:**
   - Go to Settings → Deploy
   - Change Start Command to: `cd backend && uvicorn main:app --host 0.0.0.0 --port $PORT`
   - Save

2. **Redeploy:**
   - Click "Redeploy"

3. **Check Logs:**
   - Should see "Uvicorn running" instead of "docusaurus start"

4. **Test:**
   - `curl https://your-app.railway.app/health`

---

## 📞 Still Having Issues?

If Railway keeps running the frontend:

1. **Delete the service** in Railway
2. **Create new service**
3. **During setup:**
   - Select "Deploy from repo"
   - In "Advanced Settings" → Set start command BEFORE first deploy
4. **Or:** Create separate backend-only repo

---

**Remember:** Railway = Backend ONLY, GitHub Pages = Frontend ONLY