import React, { useState, useEffect } from 'react';
import { supabase } from '../../../lib/supabaseClient';
import { useAuth } from '../../../contexts/AuthContext';

export default function ProfileSetup() {
  const { user } = useAuth();
  const [loading, setLoading] = useState(true);
  const [fullName, setFullName] = useState('');
  const [username, setUsername] = useState('');
  const [website, setWebsite] = useState('');
  const [avatarUrl, setAvatarUrl] = useState('');
  const [error, setError] = useState('');
  const [success, setSuccess] = useState('');

  useEffect(() => {
    let isMounted = true;
    async function getProfile() {
      try {
        setLoading(true);
        const { data, error } = await supabase
          .from('user_profiles')
          .select(`full_name, username, website, avatar_url`)
          .eq('id', user.id)
          .single();

        if (isMounted) {
          if (error) throw error;
          if (data) {
            setFullName(data.full_name || '');
            setUsername(data.username || '');
            setWebsite(data.website || '');
            setAvatarUrl(data.avatar_url || '');
          }
        }
      } catch (error) {
        setError(error.message);
      } finally {
        if (isMounted) setLoading(false);
      }
    }
    if (user) getProfile();
    return () => { isMounted = false };
  }, [user]);

  async function updateProfile(e) {
    e.preventDefault();
    try {
      setLoading(true);
      setError('');
      setSuccess('');
      const updates = {
        id: user.id,
        full_name: fullName,
        username,
        website,
        avatar_url: avatarUrl,
        updated_at: new Date(),
      };
      const { error } = await supabase.from('user_profiles').upsert(updates);
      if (error) throw error;
      setSuccess('Profile updated successfully!');
    } catch (error) {
      setError(error.message);
    } finally {
      setLoading(false);
    }
  }

  return (
    <div>
      <h2>Profile Setup</h2>
      <p>Complete your profile to get the most out of your learning experience.</p>
      {error && <p style={{ color: 'red' }}>{error}</p>}
      {success && <p style={{ color: 'green' }}>{success}</p>}
      <form onSubmit={updateProfile}>
        <div>
          <label htmlFor="fullName">Full Name</label>
          <input id="fullName" type="text" value={fullName} onChange={(e) => setFullName(e.target.value)} />
        </div>
        <div>
          <label htmlFor="username">Username</label>
          <input id="username" type="text" value={username} onChange={(e) => setUsername(e.target.value)} />
        </div>
        <div>
          <label htmlFor="website">Website</label>
          <input id="website" type="url" value={website} onChange={(e) => setWebsite(e.target.value)} />
        </div>
        {/* Simple avatar URL input. For a production app, you'd want file uploads. */}
        <div>
          <label htmlFor="avatarUrl">Avatar URL</label>
          <input id="avatarUrl" type="url" value={avatarUrl} onChange={(e) => setAvatarUrl(e.target.value)} />
        </div>
        <div>
          <button type="submit" disabled={loading}>
            {loading ? 'Saving...' : 'Update Profile'}
          </button>
        </div>
      </form>
    </div>
  );
}
