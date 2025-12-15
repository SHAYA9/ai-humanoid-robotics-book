import os
from dotenv import load_dotenv

load_dotenv()

# Determine which AI service to use based on environment variable
AI_PROVIDER = os.getenv("AI_PROVIDER", "hybrid").lower()

# Import the appropriate service
if AI_PROVIDER == "hybrid":
    print("🔧 Using Hybrid AI service (Gemini embeddings + Qwen text generation)")
    try:
        import hybrid_service as ai_service
        print("✓ Hybrid service loaded successfully")
        print("  - Embeddings: Gemini (matches ingestion)")
        print("  - Text Generation: Qwen/OpenRouter (avoids quota)")
    except ImportError as e:
        print(f"✗ Failed to import Hybrid service: {e}")
        print("✗ Falling back to Gemini")
        import gemini_service as ai_service
        AI_PROVIDER = "gemini"
elif AI_PROVIDER == "qwen":
    print("🔧 Using Qwen AI service")
    try:
        import qwen_service as ai_service
        print("✓ Qwen service loaded successfully")
    except ImportError:
        print("✗ Failed to import Qwen service, falling back to Gemini")
        import gemini_service as ai_service
        AI_PROVIDER = "gemini"
else:
    print("🔧 Using Gemini AI service")
    try:
        import gemini_service as ai_service
        print("✓ Gemini service loaded successfully")
    except ImportError:
        print("✗ Failed to import Gemini service, trying Qwen")
        try:
            import qwen_service as ai_service
            AI_PROVIDER = "qwen"
            print("✓ Qwen service loaded as fallback")
        except ImportError:
            raise RuntimeError("No AI service available. Please check your configuration.")

# Export the functions from the selected service
get_embedding = ai_service.get_embedding
generate_answer = ai_service.generate_answer
generate_answer_sync = ai_service.generate_answer_sync

def get_current_provider():
    """Returns the currently active AI provider"""
    return AI_PROVIDER