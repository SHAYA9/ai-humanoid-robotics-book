import React from 'react';
import Layout from '@theme/Layout';
import Dashboard from '../components/Dashboard/Dashboard';
import { useAuth } from '../contexts/AuthContext';
import { Redirect } from '@docusaurus/router';

export default function DashboardPage() {
  const { user } = useAuth();

  if (!user) {
    return <Redirect to="/login" />;
  }

  return (
    <Layout title="Dashboard">
      <Dashboard />
    </Layout>
  );
}
