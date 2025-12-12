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

  const signOutUser = async () => {
    if (!supabase) {
      return Promise.reject(new Error('Supabase not configured'));
    }
    
    try {
      // Use local scope to avoid 403 errors with global scope
      // This clears the session from the browser without calling the server
      const { error } = await supabase.auth.signOut({ scope: 'local' });
      
      if (error) {
        console.warn('Supabase signOut warning:', error);
        // Even if there's an error, we'll clear local state
      }
      
      // Clear user state immediately
      setUser(null);
      
      // Clear all Supabase-related items from storage
      if (typeof window !== 'undefined') {
        const keysToRemove = [];
        for (let i = 0; i < localStorage.length; i++) {
          const key = localStorage.key(i);
          if (key && key.startsWith('sb-')) {
            keysToRemove.push(key);
          }
        }
        keysToRemove.forEach(key => localStorage.removeItem(key));
      }
      
      return { error: null };
    } catch (error) {
      console.error('Sign out error:', error);
      // Still clear local state even if server call fails
      setUser(null);
      return { error };
    }
  };

  const value = {
    signUp: (data) => supabase ? supabase.auth.signUp(data) : Promise.reject(new Error('Supabase not configured')),
    signIn: (data) => supabase ? supabase.auth.signInWithPassword(data) : Promise.reject(new Error('Supabase not configured')),
    signOut: signOutUser,
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
