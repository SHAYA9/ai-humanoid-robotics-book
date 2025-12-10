import { supabase } from '../supabaseClient';

export async function trackModuleCompletion(userId, moduleId, score, timeSpent) {
  const { data, error } = await supabase.from('user_progress').insert([
    {
      user_id: userId,
      module_id: moduleId,
      score,
      time_spent: timeSpent,
    },
  ]);

  if (error) {
    throw error;
  }

  return data;
}

export async function getProgress(userId) {
  const { data, error } = await supabase
    .from('user_progress')
    .select('*')
    .eq('user_id', userId);

  if (error) {
    throw error;
  }

  return data;
}
