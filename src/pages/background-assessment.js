import React from 'react';
import Layout from '@theme/Layout';
import BackgroundAssessment from '../components/auth/BackgroundAssessment';
import { useAuth } from '../contexts/AuthContext';
import { Redirect } from '@docusaurus/router';

export default function BackgroundAssessmentPage() {
  const { user, loading } = useAuth();

  if (loading) {
    return (
      <Layout title="Loading...">
        <div className="container" style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '80vh' }}>
          <h2>Loading User Session...</h2>
        </div>
      </Layout>
    );
  }

  // If loading is finished and there's still no user, redirect
  if (!user) {
    return <Redirect to="/signup" />;
  }

  return (
    <Layout title="Background Assessment">
      <div className="container" style={{ paddingTop: '2rem', paddingBottom: '2rem' }}>
        <div style={{ maxWidth: '600px', margin: '0 auto' }}>
          <BackgroundAssessment />
        </div>
      </div>
    </Layout>
  );
}
