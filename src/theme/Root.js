// src/theme/Root.js
import React, { useEffect, useState } from 'react';
import { useLocation } from '@docusaurus/router';
import { AuthProvider } from '../contexts/AuthContext';
import Chatbot from '../components/Chatbot/Chatbot';
import WelcomePopup from '../components/WelcomePopup/WelcomePopup';

// Default implementation, that you can customize
export default function Root({children}) {
  const location = useLocation();
  const [scrollProgress, setScrollProgress] = useState(0);
  const [isDocumentation, setIsDocumentation] = useState(false);

  useEffect(() => {
    // Check if current page is in /docs path - runs on every route change
    const pathname = location.pathname;
    const isDocsPage = pathname.includes('/docs');
    setIsDocumentation(isDocsPage);
    
    console.log('📍 Root: Route changed to', pathname, '- isDocsPage:', isDocsPage);
    
    // Check for Urdu preference
    const preferUrdu = localStorage.getItem('preferUrdu') === 'true';
    if (preferUrdu) {
      // Apply Urdu styles globally when component mounts
      applyUrduGlobalStyles();
    }
  }, [location.pathname]);

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

  const applyUrduGlobalStyles = () => {
    // Create a style element for Urdu typography
    const style = document.createElement('style');
    style.textContent = `
      .urdu-mode {
        font-family: 'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', 'Segoe UI', serif !important;
        direction: rtl !important;
        text-align: right !important;
      }
      
      .urdu-mode h1, 
      .urdu-mode h2, 
      .urdu-mode h3, 
      .urdu-mode h4, 
      .urdu-mode h5, 
      .urdu-mode h6,
      .urdu-mode p,
      .urdu-mode li {
        font-family: 'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', serif !important;
        direction: rtl !important;
        text-align: right !important;
        line-height: 1.8 !important;
      }
      
      /* Preserve code blocks */
      .urdu-mode pre,
      .urdu-mode code,
      .urdu-mode .prism-code {
        font-family: 'Courier New', monospace !important;
        direction: ltr !important;
        text-align: left !important;
      }
      
      /* Adjust tables for RTL */
      .urdu-mode table {
        direction: rtl;
      }
      
      .urdu-mode th,
      .urdu-mode td {
        text-align: right;
      }
    `;
    document.head.appendChild(style);
  };

  return (
    <AuthProvider>
      {/* Load Urdu font */}
      <link
        href="https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu:wght@400;500;600;700&display=swap"
        rel="stylesheet"
      />
      
      {/* Optional: Load additional Urdu font */}
      <link
        href="https://fonts.googleapis.com/css2?family=Jomhuria&display=swap"
        rel="stylesheet"
      />
      
      {isDocumentation && (
        <div style={{
          position: 'fixed',
          top: '60px',
          left: 0,
          height: '4px',
          width: `${scrollProgress}%`,
          backgroundColor: 'var(--ifm-color-primary)',
          zIndex: 9999,
          transition: 'width 0.2s ease',
        }} />
      )}
      {children}
      <WelcomePopup />
      <Chatbot />
      
      {/* Urdu language detection script */}
      <script dangerouslySetInnerHTML={{
        __html: `
          // Check for Urdu preference on page load
          (function() {
            const preferUrdu = localStorage.getItem('preferUrdu') === 'true';
            const currentPath = window.location.pathname;
            
            // Only apply to documentation pages
            if (preferUrdu && currentPath.includes('/docs')) {
              // Add slight delay to ensure DOM is ready
              setTimeout(() => {
                const mainContent = document.querySelector('.theme-doc-markdown, article, .markdown');
                if (mainContent) {
                  mainContent.classList.add('urdu-mode');
                }
              }, 100);
            }
            
            // Listen for Urdu translation events
            window.addEventListener('urduTranslationEnabled', function() {
              const mainContent = document.querySelector('.theme-doc-markdown, article, .markdown');
              if (mainContent) {
                mainContent.classList.add('urdu-mode');
              }
            });
            
            window.addEventListener('urduTranslationDisabled', function() {
              const mainContent = document.querySelector('.theme-doc-markdown, article, .markdown');
              if (mainContent) {
                mainContent.classList.remove('urdu-mode');
              }
            });
          })();
        `
      }} />
    </AuthProvider>
  );
}