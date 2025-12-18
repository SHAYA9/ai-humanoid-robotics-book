import React, { useState, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { getDashboardData } from '../../lib/personalization/personalization-engine';
import './dashboard-styles.css';

export default function EnhancedDashboard() {
  const { user } = useAuth();
  const [dashboardData, setDashboardData] = useState(null);
  const [loading, setLoading] = useState(true);
  const [activeTab, setActiveTab] = useState('overview');

  useEffect(() => {
    async function loadDashboard() {
      if (user) {
        const data = await getDashboardData(user.id);
        setDashboardData(data);
        setLoading(false);
      }
    }
    loadDashboard();
  }, [user]);

  if (loading) {
    return (
      <div className="dashboard-loading">
        <div className="loader"></div>
        <p>Loading your personalized dashboard...</p>
      </div>
    );
  }

  if (!user || !dashboardData) {
    return (
      <div className="dashboard-empty">
        <h2>Welcome to AI Humanoid Robotics!</h2>
        <p>Please log in to access your personalized dashboard.</p>
        <a href="/login" className="btn-primary">Log In</a>
      </div>
    );
  }

  const { profile, progress, stats, recommendedPath, nextModules, recommendations } = dashboardData;

  return (
    <div className="enhanced-dashboard">
      {/* Hero Section */}
      <section className="dashboard-hero">
        <div className="hero-content">
          <h1>Welcome back, {profile?.username || user.email.split('@')[0]}! 👋</h1>
          <p className="hero-subtitle">
            {stats.currentStreak > 0 
              ? `🔥 ${stats.currentStreak} day streak! Keep it up!` 
              : "Let's continue your learning journey"}
          </p>
        </div>
        <div className="hero-stats">
          <div className="stat-card">
            <div className="stat-value">{stats.modulesCompleted}</div>
            <div className="stat-label">Modules Completed</div>
          </div>
          <div className="stat-card">
            <div className="stat-value">{stats.skillLevel}</div>
            <div className="stat-label">Skill Level</div>
          </div>
          <div className="stat-card">
            <div className="stat-value">{Math.round(stats.totalTimeSpent / 60)}h</div>
            <div className="stat-label">Time Invested</div>
          </div>
        </div>
      </section>

      {/* Navigation Tabs */}
      <nav className="dashboard-tabs">
        <button 
          className={activeTab === 'overview' ? 'active' : ''}
          onClick={() => setActiveTab('overview')}
        >
          Overview
        </button>
        <button 
          className={activeTab === 'path' ? 'active' : ''}
          onClick={() => setActiveTab('path')}
        >
          Learning Path
        </button>
        <button 
          className={activeTab === 'progress' ? 'active' : ''}
          onClick={() => setActiveTab('progress')}
        >
          Progress
        </button>
        <button 
          className={activeTab === 'achievements' ? 'active' : ''}
          onClick={() => setActiveTab('achievements')}
        >
          Achievements
        </button>
      </nav>

      {/* Tab Content */}
      <div className="dashboard-content">
        {activeTab === 'overview' && (
          <OverviewTab 
            nextModules={nextModules}
            recommendations={recommendations}
            stats={stats}
          />
        )}
        
        {activeTab === 'path' && (
          <LearningPathTab 
            recommendedPath={recommendedPath}
            profile={profile}
          />
        )}
        
        {activeTab === 'progress' && (
          <ProgressTab 
            progress={progress}
            stats={stats}
          />
        )}
        
        {activeTab === 'achievements' && (
          <AchievementsTab 
            badges={stats.badges}
            stats={stats}
          />
        )}
      </div>
    </div>
  );
}

// Overview Tab Component
function OverviewTab({ nextModules, recommendations, stats }) {
  return (
    <div className="overview-tab">
      <section className="next-steps">
        <h2>📚 Continue Learning</h2>
        <div className="module-cards">
          {nextModules.map((module, index) => (
            <div key={module.id} className="module-card">
              <div className="module-header">
                <span className="module-number">#{index + 1}</span>
                <span className={`difficulty-badge ${module.difficulty}`}>
                  {module.difficulty}
                </span>
              </div>
              <h3>{module.title}</h3>
              <a href={`/docs/${module.id}`} className="btn-module">
                Start Module →
              </a>
            </div>
          ))}
        </div>
      </section>

      <section className="recommendations">
        <h2>💡 Recommended for You</h2>
        <div className="recommendation-list">
          {recommendations.map((rec, index) => (
            <div key={index} className="recommendation-card">
              <div className="rec-icon">
                {rec.type === 'tutorial' && '📖'}
                {rec.type === 'module' && '🎓'}
                {rec.type === 'resource' && '📚'}
              </div>
              <div className="rec-content">
                <h4>{rec.title}</h4>
                <p className="rec-reason">{rec.reason}</p>
                <a href={rec.url} className="rec-link">View →</a>
              </div>
            </div>
          ))}
        </div>
      </section>

      <section className="quick-stats">
        <h2>📊 This Week</h2>
        <div className="week-stats">
          <div className="week-stat">
            <div className="stat-icon">🔥</div>
            <div className="stat-info">
              <div className="stat-value">{stats.currentStreak}</div>
              <div className="stat-label">Day Streak</div>
            </div>
          </div>
          <div className="week-stat">
            <div className="stat-icon">⏱️</div>
            <div className="stat-info">
              <div className="stat-value">{Math.round(stats.totalTimeSpent / 60)}h</div>
              <div className="stat-label">Time Spent</div>
            </div>
          </div>
          <div className="week-stat">
            <div className="stat-icon">✅</div>
            <div className="stat-info">
              <div className="stat-value">{stats.modulesCompleted}</div>
              <div className="stat-label">Completed</div>
            </div>
          </div>
        </div>
      </section>
    </div>
  );
}

// Learning Path Tab Component
function LearningPathTab({ recommendedPath, profile }) {
  return (
    <div className="path-tab">
      <div className="path-header">
        <h2>🎯 Your Personalized Learning Path</h2>
        <p className="path-subtitle">{recommendedPath.name}</p>
      </div>

      <div className="path-info">
        <div className="info-card">
          <h3>Duration</h3>
          <p>{recommendedPath.duration}</p>
        </div>
        <div className="info-card">
          <h3>Time Commitment</h3>
          <p>{recommendedPath.hoursPerWeek} hours/week</p>
        </div>
        <div className="info-card">
          <h3>Difficulty</h3>
          <p>{profile?.skill_level || 'Beginner'}</p>
        </div>
      </div>

      <div className="path-description">
        <p>{recommendedPath.description}</p>
      </div>

      <div className="path-modules">
        <h3>Modules in This Path</h3>
        <div className="module-timeline">
          {recommendedPath.modules.map((moduleId, index) => (
            <div key={moduleId} className="timeline-item">
              <div className="timeline-marker">{index + 1}</div>
              <div className="timeline-content">
                <h4>{getModuleName(moduleId)}</h4>
                <a href={`/docs/${moduleId}`} className="timeline-link">
                  View Module →
                </a>
              </div>
            </div>
          ))}
        </div>
      </div>

      <div className="path-actions">
        <a href="/docs/resources/learning-paths" className="btn-secondary">
          View All Paths
        </a>
        <a href="/background-assessment" className="btn-primary">
          Retake Assessment
        </a>
      </div>
    </div>
  );
}

// Progress Tab Component
function ProgressTab({ progress, stats }) {
  const totalModules = 6;
  const completionPercentage = (progress.length / totalModules) * 100;

  return (
    <div className="progress-tab">
      <div className="progress-overview">
        <h2>📈 Your Progress</h2>
        <div className="progress-circle">
          <svg viewBox="0 0 100 100">
            <circle cx="50" cy="50" r="45" fill="none" stroke="#e0e0e0" strokeWidth="10" />
            <circle 
              cx="50" 
              cy="50" 
              r="45" 
              fill="none" 
              stroke="#4CAF50" 
              strokeWidth="10"
              strokeDasharray={`${completionPercentage * 2.827} 282.7`}
              transform="rotate(-90 50 50)"
            />
            <text x="50" y="50" textAnchor="middle" dy="7" fontSize="20" fontWeight="bold">
              {Math.round(completionPercentage)}%
            </text>
          </svg>
        </div>
        <p className="progress-text">
          {progress.length} of {totalModules} modules completed
        </p>
      </div>

      <div className="completed-modules">
        <h3>✅ Completed Modules</h3>
        <div className="module-list">
          {progress.map((item) => (
            <div key={item.module_id} className="completed-module">
              <div className="module-icon">✓</div>
              <div className="module-info">
                <h4>{getModuleName(item.module_id)}</h4>
                <p className="completion-date">
                  Completed {new Date(item.completed_at).toLocaleDateString()}
                </p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}

// Achievements Tab Component
function AchievementsTab({ badges, stats }) {
  return (
    <div className="achievements-tab">
      <h2>🏆 Your Achievements</h2>
      
      <div className="badges-section">
        <h3>Badges Earned</h3>
        <div className="badges-grid">
          {badges.map((badge, index) => (
            <div key={index} className="badge-card">
              <div className="badge-icon">{badge.icon}</div>
              <h4>{badge.name}</h4>
              <p>{badge.description}</p>
            </div>
          ))}
          {badges.length === 0 && (
            <p className="no-badges">Complete modules to earn badges!</p>
          )}
        </div>
      </div>

      <div className="milestones-section">
        <h3>Milestones</h3>
        <div className="milestones-list">
          <div className={`milestone ${stats.modulesCompleted >= 1 ? 'achieved' : ''}`}>
            <div className="milestone-icon">🎯</div>
            <div className="milestone-info">
              <h4>First Module</h4>
              <p>Complete your first module</p>
            </div>
          </div>
          <div className={`milestone ${stats.currentStreak >= 7 ? 'achieved' : ''}`}>
            <div className="milestone-icon">🔥</div>
            <div className="milestone-info">
              <h4>Week Warrior</h4>
              <p>Maintain a 7-day streak</p>
            </div>
          </div>
          <div className={`milestone ${stats.modulesCompleted >= 4 ? 'achieved' : ''}`}>
            <div className="milestone-icon">🎓</div>
            <div className="milestone-info">
              <h4>Module Master</h4>
              <p>Complete all 4 main modules</p>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

// Helper function to get module names
function getModuleName(moduleId) {
  const names = {
    'intro': 'Introduction to Physical AI',
    'prerequisites': 'Prerequisites',
    'module-1-ros2': 'ROS 2 Fundamentals',
    'module-2-simulation': 'Simulation & Digital Twins',
    'module-3-isaac': 'NVIDIA Isaac',
    'module-4-vla': 'Vision-Language-Action'
  };
  return names[moduleId] || moduleId;
}