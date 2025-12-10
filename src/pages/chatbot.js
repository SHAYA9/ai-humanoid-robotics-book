import React from 'react';
import Layout from '@theme/Layout';
import ChatInterface from '../components/Chatbot/ChatInterface';
import TextSelector from '../components/Chatbot/TextSelector';

function ChatbotPage() {
  return (
    <Layout title="Chatbot" description="Chat with the AI Humanoid Robotics Book assistant">
      <div style={{ padding: '2rem' }}>
        <h1>AI Humanoid Robotics Book Chatbot</h1>
        <p>
          Welcome! You can ask general questions or select text from any page in the book
          to ask a specific question about it.
        </p>
        <TextSelector>
          <ChatInterface />
        </TextSelector>
      </div>
    </Layout>
  );
}

export default ChatbotPage;
