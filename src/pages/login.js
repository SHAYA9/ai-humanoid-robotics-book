import React from 'react';
import Layout from '@theme/Layout';
import LoginForm from '../components/auth/LoginForm';

export default function LoginPage() {
  return (
    <Layout title="Login">
      <div className="auth-page-container">
        <LoginForm />
      </div>
    </Layout>
  );
}
