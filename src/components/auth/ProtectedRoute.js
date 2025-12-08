import React, { useState, useEffect } from 'react';
import { useAuth } from '../../../contexts/AuthContext';
import { supabase } from '../../../lib/supabaseClient';
import { useHistory } from '@docusaurus/router';

const skillLevels = ['Beginner', 'Intermediate', 'Advanced'];

export default function ProtectedRoute({ children, requiredSkillLevel }) {
  const { user } = useAuth();
  const history = useHistory();
  const [profile, setProfile] = useState(null);
  const [loading, setLoading] = useState(true);
  const [isAuthorized, setIsAuthorized] = useState(false);

  useEffect(() => {
    async function checkAuthorization() {
      if (!user) {
        setLoading(false);
        setIsAuthorized(false);
        // Optional: redirect to login page
        // history.push('/login');
        return;
      }

      // Fetch user profile
      const { data, error } = await supabase
        .from('user_profiles')
        .select('skill_level')
        .eq('id', user.id)
        .single();

      if (error) {
        console.error(error);
        setLoading(false);
        setIsAuthorized(false);
        return;
      }

      setProfile(data);

      // Check skill level
      if (requiredSkillLevel) {
        const userLevelIndex = skillLevels.indexOf(data.skill_level);
        const requiredLevelIndex = skillLevels.indexOf(requiredSkillLevel);
        if (userLevelIndex >= requiredLevelIndex) {
          setIsAuthorized(true);
        } else {
          setIsAuthorized(false);
        }
      } else {
        // If no skill level is required, just being logged in is enough
        setIsAuthorized(true);
      }

      setLoading(false);
    }

    checkAuthorization();
  }, [user, requiredSkillLevel, history]);

  if (loading) {
    return <div>Loading...</div>;
  }

  if (!isAuthorized) {
    return (
      <div>
        <h2>Not Authorized</h2>
        <p>You do not have the required permissions to view this content.</p>
        {!user && <p><a href="/login">Please log in</a> to continue.</p>}
      </div>
    );
  }

  return <>{children}</>;
}
