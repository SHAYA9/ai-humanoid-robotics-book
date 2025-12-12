import { createClient } from '@supabase/supabase-js';

// Supabase configuration
// Note: These are public credentials (anon key is meant to be public)
// Real security is handled by Row Level Security (RLS) policies in Supabase
const supabaseUrl = 'https://zzhbkiexlaepoldifqfz.supabase.co';
const supabaseAnonKey = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6Inp6aGJraWV4bGFlcG9sZGlmcWZ6Iiwicm9sZSI6ImFub24iLCJpYXQiOjE3NjUxMTY4NDcsImV4cCI6MjA4MDY5Mjg0N30.MdQUIEpaqQCaxgpwSRujCiZPUnHRfE87ueEYoS_qesc';

// Create Supabase client
export const supabase = supabaseUrl && supabaseAnonKey 
  ? createClient(supabaseUrl, supabaseAnonKey)
  : null;

// Log configuration status
if (supabase) {
  console.log('✅ Supabase client initialized successfully');
} else {
  console.warn('⚠️ Supabase client not configured');
}
