// src/theme/Root.js
import React from 'react';
import { AuthProvider } from '../contexts/AuthContext';
import Chatbot from '../components/Chatbot/Chatbot';

// Default implementation, that you can customize
export default function Root({children}) {
  return (
    <AuthProvider>
      {children}
      <Chatbot />
    </AuthProvider>
  );
}
