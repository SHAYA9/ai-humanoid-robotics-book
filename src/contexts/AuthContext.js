import React, { createContext, useState, useEffect, useContext } from 'react';
import { supabase } from '../lib/supabaseClient';

const AuthContext = createContext();

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // If supabase is not configured, skip auth initialization
    if (!supabase) {
      setLoading(false);
      return;
    }

    async function getSession() {
      try {
        const { data: { session } } = await supabase.auth.getSession();
        setUser(session?.user ?? null);
      } catch (error) {
        console.warn('Supabase auth error:', error);
        setUser(null);
      } finally {
        setLoading(false);
      }
    }

    getSession();

    const { data: { subscription } } = supabase.auth.onAuthStateChange(
      (_event, session) => {
        setUser(session?.user ?? null);
        setLoading(false);
      }
    );

    return () => {
      subscription?.unsubscribe();
    };
  }, []);

  const value = {
    signUp: (data) => supabase ? supabase.auth.signUp(data) : Promise.reject(new Error('Supabase not configured')),
    signIn: (data) => supabase ? supabase.auth.signInWithPassword(data) : Promise.reject(new Error('Supabase not configured')),
    signOut: () => supabase ? supabase.auth.signOut() : Promise.reject(new Error('Supabase not configured')),
    user,
    loading,
    isConfigured: !!supabase,
  };

  return (
    <AuthContext.Provider value={value}>
      {!loading && children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  return useContext(AuthContext);
};
