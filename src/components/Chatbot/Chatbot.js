// src/components/Chatbot/Chatbot.js
import React, { useState, useEffect, useRef } from 'react';
// Import the new, direct Gemini client
import { generateChatResponse } from './GeminiClient';
import './styles.css';

// --- Assets and Icons (no changes) ---
const ChatbotIcon = () => (
  <svg className="chatbot-fab-icon" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="currentColor"><path d="M0 0h24v24H0z" fill="none"/><path d="M21 6h-2v9H6v2c0 .55.45 1 1 1h11l4 4V7c0-.55-.45-1-1-1zm-4 6V4c0-.55-.45-1-1-1H3c-.55 0-1 .45-1 1v14l4-4h10c.55 0 1-.45 1-1z"/></svg>
);
const SendIcon = () => (
  <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg"><path d="M3.4 20.4L20.85 12.02L3.4 3.6V10.1L17.2 12L3.4 13.9V20.4Z" fill="white"/></svg>
);

// --- Text Selector Component (no changes) ---
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
                    setPosition({ x: rect.left + window.scrollX + (rect.width / 2), y: rect.bottom + window.scrollY + 8 });
                    setVisible(true);
                } else { setVisible(false); }
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
        <div ref={popupRef} className="text-selector-popup" style={{ top: `${position.y}px`, left: `${position.x}px`, transform: 'translateX(-50%)' }} onClick={() => { onTextSelect(selectedTextRef.current); setVisible(false); }}>
            Ask about this
        </div>
    );
};

// --- Main Chatbot Component (Updated Logic) ---
const Chatbot = () => {
    const [isOpen, setIsOpen] = useState(false);
    const [messages, setMessages] = useState([]);
    const [input, setInput] = useState('');
    const [isLoading, setIsLoading] = useState(false);
    const [contextText, setContextText] = useState('');
    const messagesEndRef = useRef(null);

    const scrollToBottom = () => {
        messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    };
    useEffect(scrollToBottom, [messages]);

    useEffect(() => {
        if (isOpen) {
            if (contextText) {
                setMessages([{ type: 'bot', text: `You've selected some text. What's your question about it?` }]);
            } else if (messages.length === 0) {
                setMessages([{ type: 'bot', text: 'Hello! I am your AI Assistant. How can I help you today?' }]);
            }
        }
    }, [isOpen, contextText]);

    const handleTextSelect = (text) => {
        setContextText(text);
        setIsOpen(true);
    };

    const handleToggleChat = () => {
        setIsOpen(!isOpen);
        if (!isOpen) {
            setContextText('');
            setMessages([]);
        }
    };

    const handleSubmit = async (e) => {
        e.preventDefault();
        if (!input.trim() || isLoading) return;

        const userMessage = { type: 'user', text: input };
        setMessages(prev => [...prev, userMessage]);
        setInput('');
        setIsLoading(true);

        try {
            // Call the Gemini client directly, passing the context if it exists
            const responseText = await generateChatResponse(input, contextText);
            const botMessage = { type: 'bot', text: responseText };
            setMessages(prev => [...prev, botMessage]);
            setContextText(''); // Clear context after use

        } catch (error) {
            const errorMessage = { type: 'bot', text: `Error: ${error.message}` };
            setMessages(prev => [...prev, errorMessage]);
        } finally {
            setIsLoading(false);
        }
    };

    return (
        <>
            <TextSelector onTextSelect={handleTextSelect} />
            <button className="chatbot-fab" onClick={handleToggleChat} aria-label="Toggle Chat"><ChatbotIcon /></button>
            <div className={`chatbot-container ${isOpen ? '' : 'hidden'}`}>
                <div className="chatbot-header">
                    <div className="chatbot-header-title"><span className="status-indicator"></span><span>AI Assistant</span></div>
                    <button onClick={() => setIsOpen(false)} className="chatbot-close-btn">&times;</button>
                </div>
                <div className="chatbot-messages">
                    {messages.map((msg, index) => (
                        <div key={index} className={`message ${msg.type}`}>{msg.text}</div>
                    ))}
                    {isLoading && (<div className="loading-indicator"><span className="dot"></span><span className="dot"></span><span className="dot"></span></div>)}
                    <div ref={messagesEndRef} />
                </div>
                <form onSubmit={handleSubmit} className="chatbot-input-form">
                    <input type="text" value={input} onChange={(e) => setInput(e.target.value)} className="chatbot-input" placeholder="Ask a question..." disabled={isLoading} />
                    <button type="submit" className="chatbot-send-btn" disabled={isLoading}><SendIcon /></button>
                </form>
            </div>
        </>
    );
};

export default Chatbot;
