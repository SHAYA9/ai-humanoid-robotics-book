// src/components/Chatbot/GeminiClient.js
// This file is deprecated - AI provider is now configured on the backend
// The backend supports both Gemini and Qwen APIs
// Configure AI_PROVIDER environment variable on the backend to switch providers

export const generateChatResponse = async (question, context) => {
  console.warn("GeminiClient is deprecated. Please use the backend API endpoints instead.");
  return "This client-side AI integration is deprecated. Please use the backend API.";
};