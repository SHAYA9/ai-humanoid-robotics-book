import { createClient } from '@supabase/supabase-js';

const supabaseUrl = process.env.DOCUSAURUS_SUPABASE_URL;
const supabaseAnonKey = process.env.DOCUSAURUS_SUPABASE_ANON_KEY;

export const supabase = createClient(supabaseUrl, supabaseAnonKey);
