import React from 'react';
import Layout from '@theme/Layout';
import ProfileManager from '../components/Profile/ProfileManager';
import { useAuth } from '../contexts/AuthContext';
import { Redirect } from '@docusaurus/router';

export default function ProfileManagerPage() {
  const { user } = useAuth();

  if (!user) {
    return <Redirect to="/login" />;
  }

  return (
    <Layout title="Profile Manager">
      <ProfileManager />
    </Layout>
  );
}
