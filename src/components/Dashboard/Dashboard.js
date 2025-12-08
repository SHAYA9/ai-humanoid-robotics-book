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
    return <div>Loading your dashboard...</div>;
  }

  if (!user) {
    return <div>Please log in to see your dashboard.</div>;
  }

  return (
    <div className="dashboard">
      <h1>Welcome, {profile?.username || user.email}</h1>
      <p>Your skill level is: <strong>{profile?.skill_level}</strong></p>

      <div className="dashboard-section">
        <h2>Your Learning Path</h2>
        <div className="content-grid">
          {filteredContent.map(item => (
            <div key={item.id} className="content-item">
              <h3>{item.title}</h3>
              <p>({item.difficulty})</p>
            </div>
          ))}
        </div>
      </div>

      <div className="dashboard-section">
        <h2>Recommended For You</h2>
        <div className="content-grid">
          {recommendations.map(item => (
            <div key={item.id} className="content-item recommendation">
              <h3>{item.title}</h3>
            </div>
          ))}
        </div>
      </div>

      <div className="dashboard-section">
        <h2>Your Progress</h2>
        <ul>
          {progress.map(item => (
            <li key={item.id}>
              Completed <strong>{item.module_id}</strong> with a score of {item.score}
            </li>
          ))}
        </ul>
      </div>
    </div>
  );
}
