import React, { useState, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { supabase } from '../../lib/supabaseClient';
import { getProgress } from '../../lib/personalization/progress-tracker';
import { recommendNextModules } from '../../lib/personalization/recommendation-engine';
import { filterContent } from '../../lib/personalization/content-filter';
import './styles.css';

// Mock data - replace with your actual content structure
const allModules = [
  { id: 'ros2-basics', title: 'ROS 2 Basics', topic: 'ROS 2', difficulty: 'Beginner' },
  { id: 'gazebo-intro', title: 'Intro to Gazebo', topic: 'Simulation', difficulty: 'Beginner' },
  { id: 'urdf-creation', title: 'URDF Creation', topic: 'ROS 2', difficulty: 'Intermediate' },
  { id: 'isaac-sim-basics', title: 'Isaac Sim Basics', topic: 'Isaac', difficulty: 'Intermediate' },
  { id: 'vla-models', title: 'VLA Models', topic: 'VLA', difficulty: 'Advanced' },
];


export default function Dashboard() {
  const { user } = useAuth();
  const [profile, setProfile] = useState(null);
  const [progress, setProgress] = useState([]);
  const [recommendations, setRecommendations] = useState([]);
  const [filteredContent, setFilteredContent] = useState([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    async function fetchData() {
      if (user) {
        // Fetch profile
        const { data: profileData, error: profileError } = await supabase
          .from('user_profiles')
          .select('*')
          .eq('id', user.id)
          .single();
        if (profileError) console.error(profileError);
        else setProfile(profileData);

        // Fetch progress
        const progressData = await getProgress(user.id);
        setProgress(progressData);

        // Get recommendations and filtered content
        // In a real app, you'd fetch user_preferences as well
        const recs = recommendNextModules(progressData, {}, allModules);
        setRecommendations(recs);

        const content = filterContent(allModules, profileData?.skill_level);
        setFilteredContent(content);

        setLoading(false);
      }
    }
    fetchData();
  }, [user]);

  if (loading) {
    return (
      <div className="dashboard-loading">
        <div className="loader"></div>
        <p>Loading your dashboard...</p>
      </div>
    );
  }

  if (!user) {
    return <div className="dashboard-empty">Please log in to see your dashboard.</div>;
  }

  const completedCount = progress.length;
  const totalModules = allModules.length;
  const progressPercentage = totalModules > 0 ? (completedCount / totalModules) * 100 : 0;

  return (
    <div className="dashboard">
      {/* Hero Section */}
      <div className="dashboard-hero">
        <div className="hero-content">
          <h1 className="hero-title">Welcome back, <span className="user-name">{profile?.username || user.email.split('@')[0]}</span>! 👋</h1>
          <p className="hero-subtitle">Continue your AI & Robotics learning journey</p>
        </div>
      </div>

      {/* Stats Cards */}
      <div className="stats-container">
        <div className="stat-card stat-card-1">
          <div className="stat-icon">📚</div>
          <div className="stat-content">
            <div className="stat-value">{completedCount}</div>
            <div className="stat-label">Modules Completed</div>
          </div>
        </div>
        <div className="stat-card stat-card-2">
          <div className="stat-icon">⚡</div>
          <div className="stat-content">
            <div className="stat-value">{profile?.skill_level || 'Beginner'}</div>
            <div className="stat-label">Current Level</div>
          </div>
        </div>
        <div className="stat-card stat-card-3">
          <div className="stat-icon">🎯</div>
          <div className="stat-content">
            <div className="stat-value">{totalModules - completedCount}</div>
            <div className="stat-label">Remaining</div>
          </div>
        </div>
        <div className="stat-card stat-card-4">
          <div className="stat-icon">🔥</div>
          <div className="stat-content">
            <div className="stat-value">7</div>
            <div className="stat-label">Day Streak</div>
          </div>
        </div>
      </div>

      {/* Progress Overview */}
      <div className="progress-section">
        <h2 className="section-title">Overall Learning Progress</h2>
        <div className="progress-overview">
          <div className="progress-bar-container">
            <div className="progress-bar-background">
              <div className="progress-bar-fill" style={{ width: `${progressPercentage}%` }}></div>
            </div>
            <div className="progress-info">
              <span className="progress-text">{Math.round(progressPercentage)}% Complete</span>
              <span className="progress-modules">{completedCount} of {totalModules} modules</span>
            </div>
          </div>
        </div>
      </div>

      {/* Learning Path */}
      <div className="dashboard-section">
        <h2 className="section-title">📚 Your Learning Path</h2>
        <div className="content-grid">
          {filteredContent.length > 0 ? (
            filteredContent.map(item => (
              <div key={item.id} className={`content-card difficulty-${item.difficulty.toLowerCase()}`}>
                <div className="card-header">
                  <span className="difficulty-badge">{item.difficulty}</span>
                </div>
                <h3 className="card-title">{item.title}</h3>
                <p className="card-topic">{item.topic}</p>
                <button className="card-button">Learn More →</button>
              </div>
            ))
          ) : (
            <p className="empty-state">No modules available for your level</p>
          )}
        </div>
      </div>

      {/* Recommendations */}
      {recommendations.length > 0 && (
        <div className="dashboard-section">
          <h2 className="section-title">⭐ Recommended For You</h2>
          <div className="content-grid">
            {recommendations.map(item => (
              <div key={item.id} className="content-card recommendation-card">
                <div className="card-badge">Recommended</div>
                <h3 className="card-title">{item.title}</h3>
                <p className="card-description">Perfect for your current skill level</p>
                <button className="card-button primary">Start Learning →</button>
              </div>
            ))}
          </div>
        </div>
      )}

      {/* Recent Progress */}
      {progress.length > 0 && (
        <div className="dashboard-section">
          <h2 className="section-title">✅ Recent Progress</h2>
          <div className="progress-list">
            {progress.slice(0, 5).map(item => (
              <div key={item.id} className="progress-item">
                <div className="progress-item-left">
                  <div className="progress-item-icon">✓</div>
                  <div className="progress-item-content">
                    <p className="progress-item-title">{item.module_id}</p>
                    <p className="progress-item-date">Score: {item.score}%</p>
                  </div>
                </div>
                <div className="progress-item-score">{item.score}%</div>
              </div>
            ))}
          </div>
        </div>
      )}
    </div>
  );
}
