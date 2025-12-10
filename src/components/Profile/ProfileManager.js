import React, { useState, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { supabase } from '../../lib/supabaseClient';
import './ProfileManager.css';

// This should ideally come from a shared source
const allModules = [
  { id: 'ros2-basics', title: 'ROS 2 Basics' },
  { id: 'gazebo-intro', title: 'Intro to Gazebo' },
  { id: 'urdf-creation', title: 'URDF Creation' },
  { id: 'isaac-sim-basics', title: 'Isaac Sim Basics' },
  { id: 'vla-models', title: 'VLA Models' },
];

export default function ProfileManager() {
  const { user } = useAuth();
  const [loading, setLoading] = useState(true);
  const [username, setUsername] = useState('');
  const [skillLevel, setSkillLevel] = useState('Beginner');
  const [progress, setProgress] = useState(0);
  const [error, setError] = useState('');
  const [message, setMessage] = useState('');
  const [uploading, setUploading] = useState(false);
  const [userStats, setUserStats] = useState({
    completedModules: 0,
    totalTime: 0,
    achievements: 0,
    streak: 0
  });

  useEffect(() => {
    async function fetchProfile() {
      if (user) {
        setLoading(true);

        // Fetch profile, progress, and avatar in parallel
        const profilePromise = supabase
          .from('user_profiles')
          .select('username, skill_level, user_stats')
          .eq('id', user.id)
          .single();

        const progressPromise = supabase
          .from('user_progress')
          .select('module_id, time_spent')
          .eq('user_id', user.id);

        const [{ data: profile, error: profileError }, { data: progressData, error: progressError }] = await Promise.all([profilePromise, progressPromise]);

        if (profileError) {
          console.error('Profile fetch error:', profileError);
          // Create default profile if doesn't exist
          await createDefaultProfile();
        } else if (profile) {
          setUsername(profile.username || '');
          setSkillLevel(profile.skill_level || 'Beginner');
          if (profile.user_stats) {
            setUserStats(profile.user_stats);
          }
        }

        if (progressError) {
          console.error('Progress fetch error:', progressError);
        } else {
          const completedCount = progressData ? progressData.length : 0;
          const totalCount = allModules.length;
          const progressPercentage = totalCount > 0 ? (completedCount / totalCount) * 100 : 0;
          setProgress(progressPercentage);
          
          // Calculate total time spent
          const totalTime = progressData?.reduce((sum, item) => sum + (item.time_spent || 0), 0) || 0;
          setUserStats(prev => ({ ...prev, totalTime, completedModules: completedCount }));
        }

        setLoading(false);
      }
    }
    fetchProfile();
  }, [user]);

  const createDefaultProfile = async () => {
    const defaultUsername = user.email.split('@')[0];
    const { error } = await supabase
      .from('user_profiles')
      .insert({
        id: user.id,
        username: defaultUsername,
        skill_level: 'Beginner',
        user_stats: {
          completedModules: 0,
          totalTime: 0,
          achievements: 0,
          streak: 0
        },
        created_at: new Date(),
        updated_at: new Date()
      });

    if (error) {
      console.error('Default profile creation error:', error);
      setError('Failed to create profile. Please refresh the page.');
    } else {
      setUsername(defaultUsername);
    }
  };


  async function handleSubmit(e) {
    e.preventDefault();
    setLoading(true);
    setError('');
    setMessage('');

    try {
      const updates = {
        id: user.id,
        username: username,
        skill_level: skillLevel,
        updated_at: new Date(),
      };

      const { error: profileError } = await supabase
        .from('user_profiles')
        .upsert(updates);

      if (profileError) {
        throw profileError;
      }

      setMessage('Your profile has been updated successfully!');

      // Clear message after 3 seconds
      setTimeout(() => setMessage(''), 3000);

    } catch (error) {
      setError(error.message);
    } finally {
      setLoading(false);
    }
  }

  if (loading) {
    return (
      <div className="loading-overlay">
        <div className="loading-spinner"></div>
        <p>Loading your profile...</p>
      </div>
    );
  }

  return (
    <div className="profile-manager">
      <div className="profile-header">
        <div className="avatar">
          <img
            src={`https://ui-avatars.com/api/?name=${username || user.email}&background=6366f1&color=fff&bold=true`}
            alt="User Avatar"
            onError={(e) => {
              e.target.src = `https://ui-avatars.com/api/?name=${user.email}&background=6366f1&color=fff&bold=true`;
            }}
          />
        </div>
        
        <h2>{username || user.email}</h2>
        <div className="user-title">
          <span className="skill-badge">{skillLevel}</span>
          <span className="member-since">Member since {new Date(user.created_at).toLocaleDateString('en-US', { month: 'long', year: 'numeric' })}</span>
        </div>

        <div className="user-stats">
          <div className="stat-item">
            <span className="stat-value">{userStats.completedModules}</span>
            <span className="stat-label">Modules Completed</span>
          </div>
          <div className="stat-item">
            <span className="stat-value">{Math.round(userStats.totalTime / 60)}</span>
            <span className="stat-label">Hours Learned</span>
          </div>
          <div className="stat-item">
            <span className="stat-value">{userStats.achievements}</span>
            <span className="stat-label">Achievements</span>
          </div>
          <div className="stat-item">
            <span className="stat-value">{userStats.streak}</span>
            <span className="stat-label">Day Streak</span>
          </div>
        </div>

        <div className="progress-container">
          <div className="progress-label">
            <span>Overall Progress</span>
            <span className="progress-percentage">{Math.round(progress)}%</span>
          </div>
          <div className="progress-bar-background">
            <div className="progress-bar-fill" style={{ width: `${progress}%` }}></div>
          </div>
        </div>
      </div>

      <div className="profile-content">
        {error && (
          <div className="auth-message auth-error">
            <div className="message-icon">⚠️</div>
            <div className="message-text">{error}</div>
          </div>
        )}
        {message && (
          <div className="auth-message auth-success">
            <div className="message-icon">✅</div>
            <div className="message-text">{message}</div>
          </div>
        )}

        <div className="profile-sections">
          <div className="section-card">
            <h3 className="section-title">Profile Picture</h3>
          </div>

          <div className="section-card">
            <h3 className="section-title">Profile Information</h3>
            <form onSubmit={handleSubmit} className="auth-form">
              <div className="form-row">
                <div className="form-group">
                  <label htmlFor="email">Email</label>
                  <input 
                    id="email"
                    type="email" 
                    value={user.email} 
                    disabled 
                    className="form-input"
                  />
                  <div className="form-icon">📧</div>
                </div>
                <div className="form-group">
                  <label htmlFor="username">Username</label>
                  <input
                    id="username"
                    type="text"
                    value={username}
                    onChange={(e) => setUsername(e.target.value)}
                    required
                    className="form-input"
                    placeholder="Choose a username"
                  />
                  <div className="form-icon">👤</div>
                </div>
              </div>

              <div className="form-group">
                <label htmlFor="skillLevel">Skill Level</label>
                <select 
                  id="skillLevel"
                  value={skillLevel} 
                  onChange={(e) => setSkillLevel(e.target.value)}
                  className="form-input"
                >
                  <option value="Beginner">👶 Beginner - New to AI & Robotics</option>
                  <option value="Intermediate">🚀 Intermediate - Some experience</option>
                  <option value="Advanced">🏆 Advanced - Professional experience</option>
                </select>
                <div className="form-icon">📊</div>
              </div>

              <div className="form-actions">
                <button 
                  type="button" 
                  className="btn-secondary"
                  onClick={() => {
                    // Reset form
                    fetchProfile();
                  }}
                >
                  Cancel
                </button>
                <button 
                  type="submit" 
                  className="btn-primary"
                  disabled={loading || uploading}
                >
                  {loading ? 'Updating...' : 'Save Changes'}
                </button>
              </div>
            </form>
          </div>

          <div className="section-card">
            <h3 className="section-title">Account Settings</h3>
            <div className="settings-grid">
              <button className="settings-item">
                <span className="settings-icon">🔒</span>
                <span className="settings-text">Change Password</span>
                <span className="settings-arrow">→</span>
              </button>
              <button className="settings-item">
                <span className="settings-icon">📧</span>
                <span className="settings-text">Email Preferences</span>
                <span className="settings-arrow">→</span>
              </button>
              <button className="settings-item">
                <span className="settings-icon">🔔</span>
                <span className="settings-text">Notification Settings</span>
                <span className="settings-arrow">→</span>
              </button>
              <button className="settings-item settings-item-danger">
                <span className="settings-icon">🗑️</span>
                <span className="settings-text">Delete Account</span>
                <span className="settings-arrow">→</span>
              </button>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}