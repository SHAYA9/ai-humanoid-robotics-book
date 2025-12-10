import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';

const ReadingProgressBar = () => {
  const [progress, setProgress] = useState(0);

  useEffect(() => {
    const handleScroll = () => {
      const { scrollTop, scrollHeight, clientHeight } = document.documentElement;
      if (clientHeight === 0) return;
      const scrollPercentage = (scrollTop / (scrollHeight - clientHeight)) * 100;
      setProgress(scrollPercentage);
    };

    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  return (
    <div className={styles.progressBarContainer}>
      <div
        className={styles.progressBar}
        style={{ width: `${progress}%` }}
      />
    </div>
  );
};

export default ReadingProgressBar;
