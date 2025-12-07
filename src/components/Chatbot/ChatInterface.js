import React, { useState, useEffect } from 'react';
import { sendMessage } from './KoyebAPIClient';
import ChatHistory from './ChatHistory';
import SourceCitations from './SourceCitations';
import './styles.css';

const ChatInterface = ({ selectedText }) => {
  const [question, setQuestion] = useState('');
  const [answer, setAnswer] = useState('');
  const [sources, setSources] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');

  useEffect(() => {
    if (selectedText) {
      setQuestion(`What about "${selectedText}"?`);
    }
  }, [selectedText]);

  const handleSend = async () => {
    if (!question.trim()) return;
    setIsLoading(true);
    setError('');
    setAnswer('');
    setSources('');

    try {
      const response = await sendMessage(question, selectedText);
      setAnswer(response.answer);
      setSources(response.sources);
    } catch (err) {
      setError('Failed to get an answer. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="chat-container">
      <h2>RAG Chatbot</h2>
      <div className="chat-box">
        <textarea
          value={question}
          onChange={(e) => setQuestion(e.target.value)}
          placeholder="Ask a question about the selected text or anything else..."
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
          <SourceCitations sources={sources} />
        </div>
      )}
      <ChatHistory />
    </div>
  );
};

export default ChatInterface;
