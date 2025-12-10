import React, { useState, useEffect } from 'react';
import { getHistory } from './KoyebAPIClient';

const ChatHistory = () => {
  const [history, setHistory] = useState([]);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    const fetchHistory = async () => {
      try {
        const data = await getHistory();
        setHistory(data.history);
      } catch (error) {
        console.error("Failed to fetch history:", error);
      } finally {
        setIsLoading(false);
      }
    };
    fetchHistory();
  }, []);

  if (isLoading) return <p>Loading history...</p>;

  return (
    <div className="history-container">
      <h3>Conversation History</h3>
      {history.length === 0 ? (
        <p>No history yet.</p>
      ) : (
        <ul>
          {history.map((item, index) => (
            <li key={index}>
              <p><strong>You:</strong> {item.question}</p>
              <p><strong>Bot:</strong> {item.answer}</p>
              {item.sources && <small>Sources: {item.sources}</small>}
            </li>
          ))}
        </ul>
      )}
    </div>
  );
};

export default ChatHistory;
