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
      console.log('Logging out...');
      
      // Sign out from Supabase (using local scope to avoid 403)
      await signOut();
      
      console.log('Successfully logged out');
      
      // Force a hard redirect to clear all state
      // Using replace instead of href to prevent back button issues
      window.location.replace('/ai-humanoid-robotics-book/');
      
    } catch (error) {
      console.error('Error logging out:', error);
      // Even if there's an error, redirect to clear the session
      window.location.replace('/ai-humanoid-robotics-book/');
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
      <a href="https://github.com/SHAYA9/ai-humanoid-robotics-book/" className="navbar__item navbar__link header-github-link" aria-label="GitHub repository"></a>
    </div>
  );
}

