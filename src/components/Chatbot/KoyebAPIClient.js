const API_URL = 'https://rag-chatbot.koyeb.app'; // Replace with your actual Koyeb app URL

export const sendMessage = async (question, context) => {
  const response = await fetch(`${API_URL}/chat/selected`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ question, context }),
  });
  if (!response.ok) {
    throw new Error('API request failed');
  }
  return response.json();
};

export const getHistory = async () => {
  const response = await fetch(`${API_URL}/chat/history`);
  if (!response.ok) {
    throw new Error('Failed to fetch chat history');
  }
  return response.json();
};
