# AI Humanoid Robotics Book - Backend API

A FastAPI-based RAG (Retrieval-Augmented Generation) chatbot backend that supports both **Google Gemini** and **Qwen (Alibaba Cloud)** AI providers.

## Features

- 🤖 **Dual AI Provider Support**: Choose between Google Gemini or Qwen API
- 🔍 **RAG Implementation**: Context-aware responses using Qdrant vector database
- 🌐 **CORS Enabled**: Ready for frontend integration
- ⚡ **Rate Limiting**: Built-in rate limiting to avoid quota issues
- 🔄 **Async Operations**: Fully asynchronous for better performance

## AI Provider Configuration

### Switching Between Providers

Set the `AI_PROVIDER` environment variable to choose your AI provider:

```bash
# Use Google Gemini (default)
AI_PROVIDER=gemini

# Use Qwen (Alibaba Cloud)
AI_PROVIDER=qwen
```

### Google Gemini Setup

1. Get your API key from [Google AI Studio](https://makersuite.google.com/app/apikey)
2. Set environment variables:

```bash
GEMINI_API_KEY=your_gemini_api_key_here
GEMINI_MODEL=gemini-2.0-flash  # or gemini-1.5-flash-latest
```

**Available Gemini Models:**
- `gemini-2.0-flash` - Latest experimental model
- `gemini-1.5-flash-latest` - Stable, recommended
- `gemini-1.5-flash-8b-latest` - Faster, lower cost
- `gemini-1.5-pro-latest` - Most capable

### Qwen (Alibaba Cloud) Setup

1. Get your API key from [Alibaba Cloud DashScope](https://dashscope.console.aliyun.com/)
2. Set environment variables:

```bash
QWEN_API_KEY=your_qwen_api_key_here
QWEN_API_BASE=https://dashscope.aliyuncs.com/api/v1
QWEN_MODEL=qwen-turbo  # or qwen-plus, qwen-max
QWEN_EMBEDDING_MODEL=text-embedding-v2
```

**Available Qwen Models:**
- `qwen-turbo` - Fast, cost-effective
- `qwen-plus` - Balanced performance
- `qwen-max` - Most capable
- `qwen-vl-plus` - Vision + language support

## Installation

1. **Install dependencies:**

```bash
pip install -r requirements.txt
```

2. **Set up environment variables:**

Create a `.env` file in the backend directory:

```bash
cp .env.example .env
```

Edit `.env` with your credentials:

```env
# Choose your AI provider
AI_PROVIDER=gemini  # or qwen

# Gemini Configuration
GEMINI_API_KEY=your_key_here
GEMINI_MODEL=gemini-2.0-flash

# Qwen Configuration (if using Qwen)
QWEN_API_KEY=your_key_here
QWEN_MODEL=qwen-turbo

# Qdrant Configuration
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_key
QDRANT_COLLECTION_NAME=ai-humanoid-robotics-book

# Frontend URL
FRONTEND_URL=http://localhost:3000
```

## Running the Server

### Development

```bash
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

### Production

```bash
uvicorn main:app --host 0.0.0.0 --port 8000
```

## API Endpoints

### Health Check

```
GET /
GET /health
```

Returns server status and active AI provider.

### General Chat

```
POST /api/chat/general
```

**Request Body:**
```json
{
  "question": "What is ROS2?"
}
```

**Response:**
```json
{
  "answer": "ROS2 is...",
  "sources": "General Knowledge"
}
```

### RAG Chat (Selected Text)

```
POST /api/chat/selected
```

**Request Body:**
```json
{
  "question": "Explain this concept",
  "context": "Selected text from the documentation..."
}
```

**Response:**
```json
{
  "answer": "Based on the selected text...",
  "sources": "docs/module-1/overview"
}
```

### Translation

```
POST /api/translate
```

**Request Body:**
```json
{
  "content": "<h1>Hello World</h1>",
  "target_language": "urdu"
}
```

**Response:**
```json
{
  "translated_content": "<h1>ہیلو ورلڈ</h1>"
}
```

## Architecture

```
backend/
├── main.py              # FastAPI application
├── ai_service.py        # AI provider abstraction layer (NEW)
├── gemini_service.py    # Google Gemini implementation
├── qwen_service.py      # Qwen implementation (NEW)
├── qdrant_service.py    # Vector database operations
└── requirements.txt     # Python dependencies
```

## Rate Limiting

Both AI providers include built-in rate limiting:
- Default: 15 requests per minute
- Automatically waits when limit is reached
- Graceful error handling for quota issues

## Error Handling

The API includes comprehensive error handling:
- API connection failures
- Rate limit exceeded
- Invalid requests
- Service unavailability

## CORS Configuration

Allowed origins:
- `http://localhost:3000`
- `http://localhost:3001`
- `http://localhost:8000`
- `https://shaya9.github.io`
- Custom frontend URL from environment

## Deployment

The backend is configured for deployment on:
- Railway
- PythonAnywhere
- Any ASGI-compatible hosting

See `Procfile` and `runtime.txt` for deployment configuration.

## Troubleshooting

### AI Provider Not Initializing

Check your API keys and ensure they're valid:

```bash
# For Gemini
echo $GEMINI_API_KEY

# For Qwen
echo $QWEN_API_KEY
```

### Qdrant Connection Issues

Verify your Qdrant credentials:

```bash
echo $QDRANT_URL
echo $QDRANT_API_KEY
```

### Rate Limit Errors

If you encounter rate limit errors:
1. Wait 60 seconds before retrying
2. Consider upgrading your API plan
3. Adjust rate limiter settings in the service file

## License

MIT License