import React from 'react';
import Layout from '@theme/Layout';
import SignupForm from '../components/auth/SignupForm';

export default function SignupPage() {
  return (
    <Layout title="Sign Up">
      <div className="auth-page-container">
        <SignupForm />
      </div>
    </Layout>
  );
}
