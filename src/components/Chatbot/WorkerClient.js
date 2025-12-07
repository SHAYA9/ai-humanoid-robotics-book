const WORKER_URL = 'https://rag-chatbot.your-username.workers.dev'; // Replace with your actual Worker URL

export const sendMessageToWorker = async (question, context) => {
  const response = await fetch(`${WORKER_URL}/api/selected`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ question, context }),
  });
  if (!response.ok) {
    throw new Error('Worker API request failed');
  }
  return response.json();
};

export const getHistoryFromWorker = async () => {
  const response = await fetch(`${WORKER_URL}/api/history`);
  if (!response.ok) {
    throw new Error('Failed to fetch chat history from Worker');
  }
  return response.json();
};
