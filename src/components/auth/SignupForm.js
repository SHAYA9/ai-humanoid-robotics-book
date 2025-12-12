import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import './AuthForms.css';

export default function SignupForm() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [error, setError] = useState('');
  const [success, setSuccess] = useState('');
  const [loading, setLoading] = useState(false);
  const { signUp } = useAuth();
  const history = useHistory();

  async function handleSubmit(e) {
    e.preventDefault();
    
    // Validation
    if (password !== confirmPassword) {
      setError('Passwords do not match');
      return;
    }
    
    if (password.length < 6) {
      setError('Password must be at least 6 characters long');
      return;
    }
    
    try {
      setError('');
      setSuccess('');
      setLoading(true);
      
      const { data, error } = await signUp({ 
        email, 
        password,
        options: {
          emailRedirectTo: `${window.location.origin}/ai-humanoid-robotics-book/dashboard`
        }
      });
      
      if (error) {
        throw error;
      }
      
      // Check if email confirmation is required
      if (data?.user?.identities?.length === 0) {
        setSuccess('Account created! Please check your email to confirm your account before logging in.');
      } else {
        setSuccess('Account created successfully! Redirecting to dashboard...');
        setTimeout(() => {
          window.location.href = '/ai-humanoid-robotics-book/dashboard';
        }, 2000);
      }
      
    } catch (error) {
      console.error('Signup error:', error);
      setError(error.message || 'Failed to create account. Please try again.');
    } finally {
      setLoading(false);
    }
  }

  return (
    <div className="auth-form-wrapper">
      <h2>Create an Account</h2>
      {error && <p className="auth-error">{error}</p>}
      {success && <p className="auth-success">{success}</p>}
      <form onSubmit={handleSubmit} className="auth-form">
        <label>
          <span className="input-label">Email</span>
          <input 
            type="email" 
            value={email} 
            onChange={(e) => setEmail(e.target.value)} 
            placeholder="your@email.com"
            required 
          />
        </label>
        <label>
          <span className="input-label">Password</span>
          <input 
            type="password" 
            value={password} 
            onChange={(e) => setPassword(e.target.value)} 
            placeholder="At least 6 characters"
            minLength={6}
            required 
          />
        </label>
        <label>
          <span className="input-label">Confirm Password</span>
          <input 
            type="password" 
            value={confirmPassword} 
            onChange={(e) => setConfirmPassword(e.target.value)} 
            placeholder="Re-enter your password"
            minLength={6}
            required 
          />
        </label>
        <button disabled={loading} type="submit" className="button button--primary auth-button">
          {loading ? 'Creating Account...' : 'Sign Up'}
        </button>
      </form>
      <div className="auth-switch-link">
        <p>Already have an account? <Link to="/login">Login</Link></p>
      </div>
    </div>
  );
}
