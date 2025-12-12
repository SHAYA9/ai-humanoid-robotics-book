import React, { useState, useEffect } from 'react';
import { useAuth } from '@site/src/contexts/AuthContext';
import PersonalizationModal from './PersonalizationModal';
import { personalizeContent, getUserPreferences, saveUserPreferences } from './PersonalizationService';
import './PersonalizeButton.css';

export default function PersonalizeButton() {
  const { user } = useAuth();
  const [showModal, setShowModal] = useState(false);
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [originalContent, setOriginalContent] = useState(null);
  const [personalizedContent, setPersonalizedContent] = useState(null);
  const [preferences, setPreferences] = useState(null);
  const [error, setError] = useState(null);

  useEffect(() => {
    if (user) {
      // Load user preferences
      const savedPrefs = getUserPreferences(user.id);
      setPreferences(savedPrefs);
      
      // Check if this page has cached personalized content
      const cached = sessionStorage.getItem(`personalized_${window.location.pathname}`);
      if (cached && savedPrefs) {
        setPersonalizedContent(cached);
        setIsPersonalized(true);
        applyPersonalization(cached);
      }
    }
  }, [user]);

  const getMainContent = () => {
    return document.querySelector('.theme-doc-markdown, article.markdown, .markdown');
  };

  const applyPersonalization = (personalizedHtml) => {
    const contentElement = getMainContent();
    if (contentElement) {
      contentElement.innerHTML = personalizedHtml;
      contentElement.classList.add('personalized-mode');
    }
  };

  const restoreOriginal = () => {
    const contentElement = getMainContent();
    if (contentElement && originalContent) {
      contentElement.innerHTML = originalContent;
      contentElement.classList.remove('personalized-mode');
    }
  };

  const handlePersonalize = async (userPreferences) => {
    setIsPersonalizing(true);
    setError(null);
    setShowModal(false);

    try {
      const contentElement = getMainContent();
      if (!contentElement) {
        throw new Error('Content not found');
      }

      // Store original content
      if (!originalContent) {
        setOriginalContent(contentElement.innerHTML);
      }

      // Get page title for context
      const pageTitle = document.querySelector('h1')?.textContent || 'Chapter';

      // Personalize content
      const personalized = await personalizeContent(
        contentElement.innerHTML,
        userPreferences,
        pageTitle
      );

      // Store personalized content
      setPersonalizedContent(personalized);
      sessionStorage.setItem(`personalized_${window.location.pathname}`, personalized);

      // Save preferences
      saveUserPreferences(user.id, userPreferences);
      setPreferences(userPreferences);

      // Apply personalization
      setIsPersonalized(true);
      applyPersonalization(personalized);

    } catch (err) {
      console.error('Personalization error:', err);
      setError(err.message || 'Personalization failed. Please try again.');
    } finally {
      setIsPersonalizing(false);
    }
  };

  const handleToggle = () => {
    if (isPersonalized) {
      // Switch back to original
      setIsPersonalized(false);
      restoreOriginal();
    } else if (personalizedContent) {
      // Switch to personalized
      setIsPersonalized(true);
      applyPersonalization(personalizedContent);
    } else {
      // Open settings modal
      setShowModal(true);
    }
  };

  const handleSettings = () => {
    setShowModal(true);
  };

  // Only show button for logged-in users
  if (!user) {
    return null;
  }

  return (
    <>
      <div className="personalize-button-container">
        <div className="personalize-actions">
          <button
            onClick={handleToggle}
            disabled={isPersonalizing}
            className={`personalize-button ${isPersonalized ? 'active' : ''}`}
            title={isPersonalized ? 'Show Original Content' : 'Personalize Content'}
          >
            <span className="personalize-icon">
              {isPersonalizing ? '⚙️' : isPersonalized ? '📄' : '✨'}
            </span>
            <span className="personalize-text">
              {isPersonalizing ? 'Personalizing...' : isPersonalized ? 'Show Original' : 'Personalize for Me'}
            </span>
          </button>
          
          {(isPersonalized || preferences) && (
            <button
              onClick={handleSettings}
              className="personalize-settings-button"
              title="Personalization Settings"
            >
              ⚙️
            </button>
          )}
        </div>
        
        {error && (
          <div className="personalize-error">
            {error}
          </div>
        )}
      </div>

      {showModal && (
        <PersonalizationModal
          currentPreferences={preferences}
          onSave={handlePersonalize}
          onClose={() => setShowModal(false)}
          isLoading={isPersonalizing}
        />
      )}
    </>
  );
}