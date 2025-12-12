import React, { useState, useEffect } from 'react';
import { useAuth } from '@site/src/contexts/AuthContext';
import { useHistory, useLocation } from '@docusaurus/router';
import './WelcomePopup.css';

export default function WelcomePopup() {
  const { user, loading } = useAuth();
  const history = useHistory();
  const location = useLocation();
  const [showPopup, setShowPopup] = useState(false);

  useEffect(() => {
    // Don't check until auth loading is complete
    if (loading) {
      console.log('⏳ WelcomePopup: Auth still loading, waiting...');
      return;
    }

    const pathname = location.pathname;
    const isDocPage = pathname.includes('/docs');
    
    // Check if this is the first docs page visit in this session
    const hasVisitedDocs = sessionStorage.getItem('has_visited_docs');
    const hasSeenPopup = sessionStorage.getItem('welcome_popup_seen');
    
    console.log('🔍 WelcomePopup Check:', { 
      user: user ? 'logged in' : 'not logged in', 
      loading,
      isDocPage, 
      pathname,
      hasVisitedDocs,
      hasSeenPopup,
      timestamp: new Date().toISOString()
    });
    
    // Show popup if: not logged in, on docs page, first visit, and hasn't dismissed it
    if (!user && isDocPage && !hasVisitedDocs && !hasSeenPopup) {
      console.log('✅ All conditions met - will show popup in 1.5s');
      
      // Show popup after a short delay
      const timer = setTimeout(() => {
        console.log('🎉 Showing welcome popup NOW');
        setShowPopup(true);
        // Mark as visited only AFTER showing the popup
        sessionStorage.setItem('has_visited_docs', 'true');
      }, 1500);
      
      return () => {
        console.log('🧹 Cleaning up popup timer');
        clearTimeout(timer);
      };
    } else {
      console.log('❌ Not showing popup. Reasons:', {
        'user logged in': !!user,
        'not on docs page': !isDocPage,
        'already visited docs': !!hasVisitedDocs,
        'already dismissed': !!hasSeenPopup
      });
      
      // Mark as visited if user is logged in or already dismissed
      if (isDocPage && !hasVisitedDocs && (user || hasSeenPopup)) {
        sessionStorage.setItem('has_visited_docs', 'true');
      }
    }
  }, [user, loading, location.pathname]);

  const handleLogin = () => {
    sessionStorage.setItem('welcome_popup_seen', 'true');
    setShowPopup(false);
    history.push('/ai-humanoid-robotics-book/login');
  };

  const handleContinue = () => {
    // Set expiry time (e.g., 24 hours from now)
    const expiryTime = new Date().getTime() + (24 * 60 * 60 * 1000);
    sessionStorage.setItem('welcome_popup_seen', 'true');
    localStorage.setItem('welcome_popup_expiry', expiryTime.toString());
    setShowPopup(false);
  };

  // Don't render if loading, user is logged in, or popup shouldn't show
  if (loading || user || !showPopup) {
    console.log('🚫 WelcomePopup: Not rendering. loading:', loading, 'user:', !!user, 'showPopup:', showPopup);
    return null;
  }

  console.log('✨ WelcomePopup: Rendering popup!');

  return (
    <div className="welcome-popup-overlay" onClick={handleContinue}>
      <div className="welcome-popup" onClick={(e) => e.stopPropagation()}>
        <div className="welcome-popup-header">
          <div className="welcome-icon">👋</div>
          <h2>Welcome to AI Humanoid Robotics!</h2>
          <button className="popup-close" onClick={handleContinue} aria-label="Close">×</button>
        </div>

        <div className="welcome-popup-content">
          <p className="welcome-message">
            You're currently reading as a <strong>guest</strong>. Sign in to unlock powerful learning features:
          </p>

          <div className="features-grid">
            <div className="feature-item">
              <span className="feature-icon">✨</span>
              <div className="feature-content">
                <h3>Personalize Content</h3>
                <p>Adjust difficulty, add examples based on your background, and customize learning style</p>
              </div>
            </div>

            <div className="feature-item">
              <span className="feature-icon">🇵🇰</span>
              <div className="feature-content">
                <h3>Translate to Urdu</h3>
                <p>Read chapters in Urdu with proper typography and formatting</p>
              </div>
            </div>

            <div className="feature-item">
              <span className="feature-icon">💾</span>
              <div className="feature-content">
                <h3>Save Progress</h3>
                <p>Track your learning journey and pick up where you left off</p>
              </div>
            </div>

            <div className="feature-item">
              <span className="feature-icon">🤖</span>
              <div className="feature-content">
                <h3>AI Assistant</h3>
                <p>Get instant help with concepts and personalized explanations</p>
              </div>
            </div>
          </div>

          <div className="popup-note">
            <span className="note-icon">ℹ️</span>
            <p>Don't worry, you can still read all content without signing in!</p>
          </div>
        </div>

        <div className="welcome-popup-actions">
          <button onClick={handleContinue} className="btn-continue">
            Continue Without Signing In
          </button>
          <button onClick={handleLogin} className="btn-login">
            Sign In to Unlock Features
          </button>
        </div>
      </div>
    </div>
  );
}