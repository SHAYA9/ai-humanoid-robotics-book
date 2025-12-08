import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';
import Link from '@docusaurus/Link';
import './AuthForms.css';

export default function LoginForm() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);
  const { signIn } = useAuth();
  const history = useHistory();

  async function handleSubmit(e) {
    e.preventDefault();
    try {
      setError('');
      setLoading(true);
      const { error } = await signIn({ email, password });
      if (error) {
        throw error;
      }
      history.push('/dashboard');
    } catch (error) {
      setError(error.message);
    }
    setLoading(false);
  }

  return (
    <div className="auth-form-wrapper">
      <h2>Login</h2>
      {error && <p className="auth-error">{error}</p>}
      <form onSubmit={handleSubmit} className="auth-form">
        <label>
          <span className="input-label">Email</span>
          <input type="email" value={email} onChange={(e) => setEmail(e.target.value)} required />
        </label>
        <label>
          <span className="input-label">Password</span>
          <input type="password" value={password} onChange={(e) => setPassword(e.target.value)} required />
        </label>
        <button disabled={loading} type="submit" className="button button--primary auth-button">
          {loading ? 'Logging in...' : 'Login'}
        </button>
      </form>
      <div className="auth-switch-link">
        <p>Don't have an account? <Link to="/signup">Sign Up</Link></p>
      </div>
    </div>
  );
}
