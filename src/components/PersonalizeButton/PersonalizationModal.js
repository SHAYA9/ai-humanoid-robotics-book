import React, { useState, useEffect } from 'react';
import './PersonalizationModal.css';

export default function PersonalizationModal({ currentPreferences, onSave, onClose, isLoading }) {
  const [preferences, setPreferences] = useState({
    difficulty: 'intermediate',
    background: [],
    interests: '',
    learningStyle: 'balanced',
    customization: {
      moreExamples: true,
      relateToProjects: false,
      simplifyJargon: false,
      addVisuals: false
    }
  });

  useEffect(() => {
    if (currentPreferences) {
      setPreferences(currentPreferences);
    }
  }, [currentPreferences]);

  const handleSubmit = (e) => {
    e.preventDefault();
    onSave(preferences);
  };

  const handleBackgroundToggle = (bg) => {
    setPreferences(prev => ({
      ...prev,
      background: prev.background.includes(bg)
        ? prev.background.filter(b => b !== bg)
        : [...prev.background, bg]
    }));
  };

  const handleCustomizationToggle = (key) => {
    setPreferences(prev => ({
      ...prev,
      customization: {
        ...prev.customization,
        [key]: !prev.customization[key]
      }
    }));
  };

  return (
    <div className="personalization-modal-overlay" onClick={onClose}>
      <div className="personalization-modal" onClick={(e) => e.stopPropagation()}>
        <div className="modal-header">
          <h2>🎯 Personalize Your Learning</h2>
          <button className="modal-close" onClick={onClose}>×</button>
        </div>

        <form onSubmit={handleSubmit} className="modal-content">
          {/* Difficulty Level */}
          <div className="form-section">
            <label className="section-label">📊 Difficulty Level</label>
            <div className="radio-group">
              {['beginner', 'intermediate', 'advanced'].map(level => (
                <label key={level} className="radio-option">
                  <input
                    type="radio"
                    name="difficulty"
                    value={level}
                    checked={preferences.difficulty === level}
                    onChange={(e) => setPreferences({...preferences, difficulty: e.target.value})}
                  />
                  <span>{level.charAt(0).toUpperCase() + level.slice(1)}</span>
                </label>
              ))}
            </div>
          </div>

          {/* Background */}
          <div className="form-section">
            <label className="section-label">💻 Your Background (select all that apply)</label>
            <div className="checkbox-group">
              {['Python', 'C++', 'JavaScript', 'ROS', 'Machine Learning', 'Robotics', 'Web Development'].map(bg => (
                <label key={bg} className="checkbox-option">
                  <input
                    type="checkbox"
                    checked={preferences.background.includes(bg)}
                    onChange={() => handleBackgroundToggle(bg)}
                  />
                  <span>{bg}</span>
                </label>
              ))}
            </div>
          </div>

          {/* Interests/Projects */}
          <div className="form-section">
            <label className="section-label">🎯 Your Interests or Current Projects</label>
            <textarea
              className="interests-input"
              placeholder="E.g., Building a walking robot, Learning computer vision, Working on autonomous navigation..."
              value={preferences.interests}
              onChange={(e) => setPreferences({...preferences, interests: e.target.value})}
              rows={3}
            />
          </div>

          {/* Learning Style */}
          <div className="form-section">
            <label className="section-label">📚 Learning Style</label>
            <div className="radio-group">
              {[
                { value: 'visual', label: 'Visual (diagrams, examples)' },
                { value: 'practical', label: 'Practical (hands-on, code-first)' },
                { value: 'theoretical', label: 'Theoretical (concepts, explanations)' },
                { value: 'balanced', label: 'Balanced (mix of all)' }
              ].map(style => (
                <label key={style.value} className="radio-option">
                  <input
                    type="radio"
                    name="learningStyle"
                    value={style.value}
                    checked={preferences.learningStyle === style.value}
                    onChange={(e) => setPreferences({...preferences, learningStyle: e.target.value})}
                  />
                  <span>{style.label}</span>
                </label>
              ))}
            </div>
          </div>

          {/* Additional Customizations */}
          <div className="form-section">
            <label className="section-label">⚙️ Additional Customizations</label>
            <div className="checkbox-group">
              <label className="checkbox-option">
                <input
                  type="checkbox"
                  checked={preferences.customization.moreExamples}
                  onChange={() => handleCustomizationToggle('moreExamples')}
                />
                <span>Add more practical examples</span>
              </label>
              <label className="checkbox-option">
                <input
                  type="checkbox"
                  checked={preferences.customization.relateToProjects}
                  onChange={() => handleCustomizationToggle('relateToProjects')}
                />
                <span>Relate content to my projects</span>
              </label>
              <label className="checkbox-option">
                <input
                  type="checkbox"
                  checked={preferences.customization.simplifyJargon}
                  onChange={() => handleCustomizationToggle('simplifyJargon')}
                />
                <span>Simplify technical jargon</span>
              </label>
              <label className="checkbox-option">
                <input
                  type="checkbox"
                  checked={preferences.customization.addVisuals}
                  onChange={() => handleCustomizationToggle('addVisuals')}
                />
                <span>Suggest visual aids and diagrams</span>
              </label>
            </div>
          </div>

          {/* Action Buttons */}
          <div className="modal-actions">
            <button type="button" onClick={onClose} className="btn-cancel" disabled={isLoading}>
              Cancel
            </button>
            <button type="submit" className="btn-save" disabled={isLoading}>
              {isLoading ? 'Personalizing...' : 'Apply Personalization'}
            </button>
          </div>
        </form>
      </div>
    </div>
  );
}