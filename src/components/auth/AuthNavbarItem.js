import React from 'react';
import {useAuth} from '@site/src/contexts/AuthContext';
import {useHistory, useLocation} from '@docusaurus/router';
import Link from '@docusaurus/Link';
import './AuthNavbarItem.css';

export default function AuthNavbarItem() {
  const { user, signOut } = useAuth();
  const history = useHistory();
  const location = useLocation();

  async function handleLogout() {
    try {
      await signOut();
      // Redirect to home page immediately after logout
      setTimeout(() => {
        history.push('/');
      }, 100);
    } catch (error) {
      console.error('Error logging out:', error);
    }
  }

  if (user) {
    return (
      <div className="auth-navbar-items">
        <Link to="/dashboard" className="navbar__item navbar__link">Dashboard</Link>
        <Link to="/profile-manager" className="navbar__item navbar__link">Profile</Link>
        <button onClick={handleLogout} className="navbar__item navbar__link auth-button logout-button">
          Logout
        </button>
        <a href="https://github.com/SHAYA9/ai-humanoid-robotics-book" className="navbar__item navbar__link header-github-link" aria-label="GitHub repository"></a>
      </div>
    );
  }

  return (
    <div className="auth-navbar-items">
      <Link to="/login" className="navbar__item navbar__link">Login</Link>
      <Link to="/signup" className="navbar__item navbar__link signup-link">Sign Up</Link>
      <a href="https://github.com/SHAYA9/ai-human-robotics-book" className="navbar__item navbar__link header-github-link" aria-label="GitHub repository"></a>
    </div>
  );
}

