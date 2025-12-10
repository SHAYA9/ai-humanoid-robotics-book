import React, { useState, useEffect } from 'react';
import { sendMessageToWorker } from './WorkerClient';
import './styles.css'; // Assuming you have a shared styles.css

const CloudflareChat = ({ selectedText }) => {
  const [question, setQuestion] = useState('');
  const [answer, setAnswer] = useState('');
  const [sources, setSources] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');

  useEffect(() => {
    if (selectedText) {
      setQuestion(`What is meant by "${selectedText}"?`);
    }
  }, [selectedText]);

  const handleSend = async () => {
    if (!question.trim()) return;
    setIsLoading(true);
    setError('');
    setAnswer('');
    setSources('');

    try {
      const response = await sendMessageToWorker(question, selectedText || question);
      setAnswer(response.answer);
      setSources(response.sources);
    } catch (err) {
      setError('Failed to get an answer. The worker might be down or an error occurred.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="chat-container">
      <h2>Cloudflare RAG Chatbot</h2>
      <div className="chat-box">
        <textarea
          value={question}
          onChange={(e) => setQuestion(e.target.value)}
          placeholder={selectedText ? "Ask about the selected text..." : "Ask a general question..."}
          rows="3"
        />
        <button onClick={handleSend} disabled={isLoading}>
          {isLoading ? 'Thinking...' : 'Ask'}
        </button>
      </div>
      {error && <p className="error">{error}</p>}
      {answer && (
        <div className="response-box">
          <h3>Answer:</h3>
          <p>{answer}</p>
          {sources && <small><strong>Sources:</strong> {sources}</small>}
        </div>
      )}
    </div>
  );
};

export default CloudflareChat;
