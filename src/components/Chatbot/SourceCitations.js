import React from 'react';

const SourceCitations = ({ sources }) => {
  if (!sources) return null;

  return (
    <div className="citations">
      <strong>Sources:</strong> {sources}
    </div>
  );
};

export default SourceCitations;
