// src/components/Chatbot/GeminiClient.js
import { GoogleGenerativeAI } from "@google/generative-ai";

// WARNING: This is NOT safe for production. Your API key will be visible in the browser.
// This is for hackathon/demonstration purposes only.
const API_KEY = ""
if (!API_KEY) {
  console.error("Gemini API key is missing. Please set DOCUSAURUS_GEMINI_API_KEY in your .env file.");
}

const genAI = new GoogleGenerativeAI(API_KEY);
const model = genAI.getGenerativeModel({ model: "gemini-2.0-flash" });

export const generateChatResponse = async (question, context) => {
  try {
    let prompt;

    if (context) {
      // If text was selected, create a prompt that focuses on that context
      prompt = `You are a helpful assistant for the AI Humanoid Robotics Book. Based ONLY on the following selected text, answer the user's question. Do not use any other information. If the answer is not in the text, say so.

Selected Text:
---
${context}
---

Question: ${question}`;
    } else {
      // For general chat
      prompt = `You are a helpful assistant. Answer the following question concisely: ${question}`;
    }

    const result = await model.generateContent(prompt);
    const response = await result.response;
    const text = response.text();
    return text;

  } catch (error) {
    console.error("Error generating response from Gemini:", error);
    return "Sorry, I encountered an error while trying to connect to the AI model.";
  }
};
