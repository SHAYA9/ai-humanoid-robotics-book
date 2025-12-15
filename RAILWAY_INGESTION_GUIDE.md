# Railway Qdrant Ingestion Guide

## Problem
Your Qdrant database is empty, so the chatbot can't answer questions about selected text.

## Solution
I've added an API endpoint to your Railway deployment that you can call to populate Qdrant.

---

## Method 1: Using Browser (Easiest)

1. **Open your browser** and go to:
   ```
   https://ai-humanoid-robotics-book-production.up.railway.app/api/admin/ingest-docs
   ```

2. **Wait for the response**. This will take 5-15 minutes. You'll see a JSON response like:
   ```json
   {
     "status": "success",
     "message": "Documents successfully ingested into Qdrant",
     "details": {
       "files_processed": 50,
       "chunks_created": 500,
       "points_uploaded": 500
     }
   }
   ```

3. **Done!** Your Qdrant is now populated.

---

## Method 2: Using PowerShell

Open PowerShell and run:

```powershell
Invoke-WebRequest -Uri "https://ai-humanoid-robotics-book-production.up.railway.app/api/admin/ingest-docs" -Method POST
```

---

## Method 3: Using curl (if installed)

```bash
curl -X POST https://ai-humanoid-robotics-book-production.up.railway.app/api/admin/ingest-docs
```

---

## Verify It Worked

### Option A: Check Status Endpoint

Visit in browser:
```
https://ai-humanoid-robotics-book-production.up.railway.app/api/admin/qdrant-status
```

You should see:
```json
{
  "status": "initialized",
  "collection_name": "ai-humanoid-robotics-book",
  "points_count": 500,  // Should be > 0
  "vector_size": 768,
  "distance": "COSINE"
}
```

### Option B: Check Railway Logs

In Railway dashboard, check the logs. You should see:
```
Starting document ingestion via API endpoint...
Step 1/5: Recreating Qdrant collection...
✓ Collection recreated
Step 2/5: Finding markdown files...
✓ Found 50 markdown files
...
✓ Successfully ingested 500 points
```

### Option C: Test the Chatbot

1. Go to your documentation site
2. Select some text
3. Ask a question about it
4. The chatbot should now provide relevant answers with sources!

---

## Troubleshooting

### "Ingestion failed: No markdown files found"
- This means the `docs/` folder is not in your Railway deployment
- Make sure your `docs/` folder is committed to git and pushed
- Check Railway build logs to ensure `docs/` is included

### "Ingestion failed: Failed to generate embeddings"
- Check that your AI provider (Gemini or Qwen) API key is valid
- Check Railway environment variables are set correctly
- Look at Railway logs for specific error messages

### Request Times Out
- The ingestion takes 5-15 minutes
- Your browser might timeout, but the process continues on Railway
- Check Railway logs to see progress
- Wait a few minutes, then check the status endpoint

### Still Getting "I do not have enough information"
1. Verify points_count > 0 using the status endpoint
2. Make sure `QDRANT_COLLECTION_NAME` is the same in Railway and your code
3. Try re-running the ingestion endpoint

---

## When to Re-run Ingestion

Run the ingestion endpoint again when:
- You add new documentation pages
- You make significant content updates
- Search results seem outdated

The endpoint recreates the collection each time, so it's safe to run multiple times.

---

## Important Notes

⚠️ **This endpoint recreates the collection** - All existing data is replaced

⏱️ **Takes 5-15 minutes** - Be patient, check Railway logs for progress

🔒 **Consider adding authentication** - This is an admin endpoint that should be protected in production

---

## Next Steps

1. ✅ Run the ingestion endpoint (Method 1 above)
2. ✅ Verify it worked using the status endpoint
3. ✅ Test your chatbot with selected text
4. ✅ Enjoy your fully functional RAG chatbot!

---

## Alternative: Run Locally Then Upload

If the Railway ingestion fails, you can:

1. Run the script locally:
   ```bash
   cd scripts
   python load_docs_to_qdrant.py
   ```

2. This uploads directly to your Qdrant cloud instance

3. Railway will then be able to query the populated database