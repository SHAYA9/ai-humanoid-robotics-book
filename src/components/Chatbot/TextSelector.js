import React, { useState } from 'react';

const TextSelector = ({ children }) => {
  const [selectedText, setSelectedText] = useState('');
  const [position, setPosition] = useState({ top: 0, left: 0 });

  const handleSelection = () => {
    const text = window.getSelection().toString();
    if (text) {
      const range = window.getSelection().getRangeAt(0);
      const rect = range.getBoundingClientRect();
      setSelectedText(text);
      setPosition({
        top: rect.bottom + window.scrollY,
        left: rect.left + window.scrollX,
      });
    } else {
      setSelectedText('');
    }
  };

  return (
    <div onMouseUp={handleSelection}>
      {selectedText && (
        <div
          style={{
            position: 'absolute',
            top: `${position.top}px`,
            left: `${position.left}px`,
            zIndex: 1000,
          }}
        >
          <button
            className="ask-button"
            onClick={() => {
              // Pass the selected text to the chat interface
              // This part requires state management (e.g., React Context or Zustand)
              // For now, we'll just log it.
              console.log('Selected Text:', selectedText);
            }}
          >
            Ask about this
          </button>
        </div>
      )}
      {React.cloneElement(children, { selectedText })}
    </div>
  );
};

export default TextSelector;
