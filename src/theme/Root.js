// src/theme/Root.js
import React, { useEffect, useState } from 'react';
import { AuthProvider } from '../contexts/AuthContext';
import Chatbot from '../components/Chatbot/Chatbot';

// Default implementation, that you can customize
export default function Root({children}) {
  const [scrollProgress, setScrollProgress] = useState(0);
  const [isDocumentation, setIsDocumentation] = useState(false);

  useEffect(() => {
    // Check if current page is in /docs path
    const pathname = window.location.pathname;
    setIsDocumentation(pathname.includes('/docs'));
  }, []);

  useEffect(() => {
    const handleScroll = () => {
      const totalHeight = document.documentElement.scrollHeight - document.documentElement.clientHeight;
      const scrollPosition = window.scrollY;
      const progress = (scrollPosition / totalHeight) * 100;
      setScrollProgress(progress);
    };

    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  return (
    <AuthProvider>
      {isDocumentation && (
        <div style={{
          position: 'fixed',
          top: '60px',
          left: 0,
          height: '4px',
          width: `${scrollProgress}%`,
          backgroundColor: 'var(--ifm-color-primary)',
          zIndex: 9999,
        }} />
      )}
      {children}
      <Chatbot />
    </AuthProvider>
  );
}
