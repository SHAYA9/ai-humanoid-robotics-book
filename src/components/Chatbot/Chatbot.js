// src/components/Chatbot/Chatbot.js - Enhanced RAG Integration
import React, { useState, useEffect, useRef } from 'react';
import './styles.css';

// --- Configuration ---
const getApiUrl = () => {
  // For production, use live API URL
  if (typeof window !== 'undefined') {
    // Check environment variable first
    if (window.__ENV__?.VITE_API_URL) {
      return window.__ENV__.VITE_API_URL;
    }
    
    // Use live API for GitHub Pages (production)
    if (window.location.hostname === 'shaya9.github.io') {
      return 'https://ai-humanoid-robotics-book-production.up.railway.app/'; // TODO: Update with your Railway URL after deployment
    }
  }
  
  // Default to localhost for development
  return 'http://localhost:8000';
};

// --- Assets and Icons ---
const ChatbotIcon = () => (
  <svg className="chatbot-fab-icon" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="currentColor">
    <path d="M0 0h24v24H0z" fill="none"/>
    <path d="M21 6h-2v9H6v2c0 .55.45 1 1 1h11l4 4V7c0-.55-.45-1-1-1zm-4 6V4c0-.55-.45-1-1-1H3c-.55 0-1 .45-1 1v14l4-4h10c.55 0 1-.45 1-1z"/>
  </svg>
);

const SendIcon = () => (
  <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M3.4 20.4L20.85 12.02L3.4 3.6V10.1L17.2 12L3.4 13.9V20.4Z" fill="white"/>
  </svg>
);

const SourceIcon = () => (
  <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-2 15l-5-5 1.41-1.41L10 14.17l7.59-7.59L19 8l-9 9z" fill="currentColor"/>
  </svg>
);

// --- Text Selector Component ---
const TextSelector = ({ onTextSelect }) => {
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const [visible, setVisible] = useState(false);
  const selectedTextRef = useRef('');
  const popupRef = useRef(null);

  useEffect(() => {
    const handleMouseUp = () => {
      setTimeout(() => {
        const selection = window.getSelection();
        const text = selection.toString().trim();
        if (text.length > 10) {
          selectedTextRef.current = text;
          const range = selection.getRangeAt(0);
          const rect = range.getBoundingClientRect();
          setPosition({
            x: rect.left + window.scrollX + rect.width / 2,
            y: rect.bottom + window.scrollY + 8
          });
          setVisible(true);
        } else {
          setVisible(false);
        }
      }, 10);
    };

    const handleClickOutside = (event) => {
      if (popupRef.current && !popupRef.current.contains(event.target)) {
        setVisible(false);
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  if (!visible) return null;

  return (
    <div
      ref={popupRef}
      className="text-selector-popup"
      style={{
        top: `${position.y}px`,
        left: `${position.x}px`,
        transform: 'translateX(-50%)'
      }}
      onClick={() => {
        onTextSelect(selectedTextRef.current);
        setVisible(false);
      }}
    >
      💡 Ask about this
    </div>
  );
};

// --- Message Component with Source Attribution ---
const Message = ({ msg, isLoading }) => {
  const isBot = msg.type === 'bot';

  return (
    <div className={`message ${msg.type}`}>
      <div className="message-content">
        {msg.text}
        {isBot && msg.sources && (
          <div className="message-sources">
            <div className="sources-label">
              <SourceIcon /> Sources:
            </div>
            <div className="sources-list">
              {msg.sources.split(',').map((source, idx) => (
                <span key={idx} className="source-tag">
                  {source.trim()}
                </span>
              ))}
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

// --- Main Chatbot Component ---
const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [error, setError] = useState('');
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(scrollToBottom, [messages]);

  useEffect(() => {
    if (isOpen && messages.length === 0) {
      const greeting = selectedText
        ? "📖 You've selected text from the book. What would you like to know about it?"
        : "👋 Hi! I'm your AI assistant for the AI Humanoid Robotics Book. You can select text from the documentation or ask me anything!";
      
      setMessages([{ type: 'bot', text: greeting }]);
    }
  }, [isOpen]);

  const handleTextSelect = (text) => {
    setSelectedText(text);
    setIsOpen(true);
  };

  const handleToggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen) {
      if (messages.length === 0) {
        setSelectedText('');
      }
    }
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!input.trim() || isLoading) return;

    const userMessage = { type: 'user', text: input };
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setError('');
    setIsLoading(true);

    try {
      let endpoint, requestBody;

      if (selectedText) {
        // RAG endpoint for selected text
        endpoint = '/api/chat/selected';
        requestBody = {
          question: input,
          context: selectedText
        };
      } else {
        // General chat endpoint
        endpoint = '/api/chat/general';
        requestBody = {
          question: input
        };
      }

      const response = await fetch(`${getApiUrl()}${endpoint}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.statusText}`);
      }

      const data = await response.json();
      const botMessage = {
        type: 'bot',
        text: data.answer,
        sources: data.sources
      };

      setMessages(prev => [...prev, botMessage]);
      setSelectedText(''); // Clear context after use

    } catch (err) {
      console.error('Chatbot error:', err);
      setError(err.message);
      const errorMessage = {
        type: 'bot',
        text: `⚠️ Error: ${err.message || 'Failed to get response. Please try again.'}`
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      <TextSelector onTextSelect={handleTextSelect} />
      
      <button
        className="chatbot-fab"
        onClick={handleToggleChat}
        aria-label="Toggle Chat"
        title={selectedText ? 'Text selected - Ready to chat!' : 'Open AI Assistant'}
      >
        <ChatbotIcon />
        {selectedText && <span className="notification-badge">•</span>}
      </button>

      <div className={`chatbot-container ${isOpen ? '' : 'hidden'}`}>
        <div className="chatbot-header">
          <div className="chatbot-header-title">
            <span className="status-indicator"></span>
            <span>📚 Book Assistant</span>
          </div>
          <button onClick={() => setIsOpen(false)} className="chatbot-close-btn">&times;</button>
        </div>

        <div className="chatbot-messages">
          {selectedText && (
            <div className="selected-text-display">
              <p className="selected-text-label">📌 Selected Text:</p>
              <blockquote className="selected-text-content">
                "{selectedText.substring(0, 150)}..."
              </blockquote>
            </div>
          )}
          
          {messages.map((msg, index) => (
            <Message key={index} msg={msg} isLoading={isLoading && index === messages.length - 1} />
          ))}

          {isLoading && (
            <div className="loading-indicator">
              <span className="dot"></span>
              <span className="dot"></span>
              <span className="dot"></span>
            </div>
          )}

          {error && (
            <div className="error-message">
              ❌ {error}
            </div>
          )}

          <div ref={messagesEndRef} />
        </div>

        <form onSubmit={handleSubmit} className="chatbot-input-form">
          <input
            type="text"
            value={input}
            onChange={(e) => setInput(e.target.value)}
            className="chatbot-input"
            placeholder={selectedText ? 'Ask about the selected text...' : 'Ask me anything about the book...'}
            disabled={isLoading}
            autoFocus={isOpen}
          />
          <button
            type="submit"
            className="chatbot-send-btn"
            disabled={isLoading || !input.trim()}
            title="Send message"
          >
            <SendIcon />
          </button>
        </form>
      </div>
    </>
  );
};

export default Chatbot;
