/**
 * Personalization Engine
 * Handles all personalization logic for the learning platform
 */

import { supabase } from '../supabaseClient';

/**
 * Get user's learning profile
 */
export async function getUserProfile(userId) {
  const { data, error } = await supabase
    .from('user_profiles')
    .select('*')
    .eq('id', userId)
    .single();
    
  if (error) {
    console.error('Error fetching profile:', error);
    return null;
  }
  
  return data;
}

/**
 * Update user profile
 */
export async function updateUserProfile(userId, updates) {
  const { data, error } = await supabase
    .from('user_profiles')
    .update(updates)
    .eq('id', userId)
    .select()
    .single();
    
  if (error) {
    console.error('Error updating profile:', error);
    return null;
  }
  
  return data;
}

/**
 * Get recommended learning path based on user profile
 */
export function getRecommendedPath(profile) {
  if (!profile) return 'beginner';
  
  const { background, experience_level, goals, time_commitment } = profile;
  
  // Path 1: Complete Beginner
  if (experience_level === 'beginner' && !background?.includes('robotics')) {
    return {
      id: 'path-1',
      name: 'Complete Beginner to Robotics Engineer',
      duration: '16 weeks',
      hoursPerWeek: 20,
      description: 'Start from basics and build up to advanced robotics',
      modules: ['module-1-ros2', 'module-2-simulation', 'module-3-isaac', 'module-4-vla']
    };
  }
  
  // Path 2: AI Engineer to Robotics
  if (background?.includes('ai') || background?.includes('ml')) {
    return {
      id: 'path-2',
      name: 'AI Engineer to Robotics AI',
      duration: '10 weeks',
      hoursPerWeek: 15,
      description: 'Leverage your AI knowledge for robotics applications',
      modules: ['module-1-ros2', 'module-3-isaac', 'module-4-vla'],
      skipTopics: ['python-basics', 'ml-fundamentals']
    };
  }
  
  // Path 3: Robotics Engineer to AI
  if (background?.includes('robotics') || background?.includes('ros')) {
    return {
      id: 'path-3',
      name: 'Robotics Engineer to AI-Powered Systems',
      duration: '12 weeks',
      hoursPerWeek: 15,
      description: 'Add AI capabilities to your robotics expertise',
      modules: ['module-3-isaac', 'module-4-vla'],
      skipTopics: ['ros-basics', 'urdf-basics']
    };
  }
  
  // Path 4: Part-time Professional
  if (time_commitment === 'part-time') {
    return {
      id: 'path-4',
      name: 'Industry Professional (Part-Time)',
      duration: '24 weeks',
      hoursPerWeek: 10,
      description: 'Flexible schedule for working professionals',
      modules: ['module-1-ros2', 'module-2-simulation', 'module-3-isaac', 'module-4-vla'],
      pacing: 'flexible'
    };
  }
  
  // Path 5: Research Track
  if (goals?.includes('research') || experience_level === 'advanced') {
    return {
      id: 'path-5',
      name: 'Research Track',
      duration: '16+ weeks',
      hoursPerWeek: 20,
      description: 'Research-oriented curriculum with paper discussions',
      modules: ['module-1-ros2', 'module-2-simulation', 'module-3-isaac', 'module-4-vla'],
      extras: ['research-papers', 'research-project']
    };
  }
  
  // Default: Beginner path
  return {
    id: 'path-1',
    name: 'Complete Beginner to Robotics Engineer',
    duration: '16 weeks',
    hoursPerWeek: 20,
    description: 'Start from basics and build up to advanced robotics',
    modules: ['module-1-ros2', 'module-2-simulation', 'module-3-isaac', 'module-4-vla']
  };
}

/**
 * Get next recommended modules based on progress
 */
export function getNextModules(progress, profile) {
  const completedModules = progress.map(p => p.module_id);
  const allModules = [
    { id: 'intro', title: 'Introduction to Physical AI', difficulty: 'beginner', order: 1 },
    { id: 'prerequisites', title: 'Prerequisites', difficulty: 'beginner', order: 2 },
    { id: 'module-1-ros2', title: 'ROS 2 Fundamentals', difficulty: 'beginner', order: 3 },
    { id: 'module-2-simulation', title: 'Simulation & Digital Twins', difficulty: 'intermediate', order: 4 },
    { id: 'module-3-isaac', title: 'NVIDIA Isaac', difficulty: 'intermediate', order: 5 },
    { id: 'module-4-vla', title: 'Vision-Language-Action', difficulty: 'advanced', order: 6 },
  ];
  
  // Filter out completed modules
  const availableModules = allModules.filter(m => !completedModules.includes(m.id));
  
  // Sort by order
  availableModules.sort((a, b) => a.order - b.order);
  
  // Return top 3 recommendations
  return availableModules.slice(0, 3);
}

/**
 * Calculate learning streak
 */
export function calculateStreak(activityLog) {
  if (!activityLog || activityLog.length === 0) return 0;
  
  const today = new Date();
  today.setHours(0, 0, 0, 0);
  
  let streak = 0;
  let currentDate = new Date(today);
  
  // Sort activities by date (most recent first)
  const sortedActivities = activityLog
    .map(a => new Date(a.created_at))
    .sort((a, b) => b - a);
  
  for (const activityDate of sortedActivities) {
    activityDate.setHours(0, 0, 0, 0);
    
    if (activityDate.getTime() === currentDate.getTime()) {
      streak++;
      currentDate.setDate(currentDate.getDate() - 1);
    } else if (activityDate.getTime() < currentDate.getTime()) {
      break;
    }
  }
  
  return streak;
}

/**
 * Get personalized content recommendations
 */
export function getContentRecommendations(profile, progress) {
  const recommendations = [];
  
  // Based on skill level
  if (profile?.skill_level === 'beginner') {
    recommendations.push({
      type: 'tutorial',
      title: 'Your First ROS 2 Robot',
      url: '/docs/resources/tutorials#tutorial-1',
      reason: 'Perfect for beginners'
    });
  } else if (profile?.skill_level === 'intermediate') {
    recommendations.push({
      type: 'tutorial',
      title: 'Autonomous Navigation with Nav2',
      url: '/docs/resources/tutorials#tutorial-4',
      reason: 'Build on your existing knowledge'
    });
  }
  
  // Based on interests
  if (profile?.interests?.includes('ai')) {
    recommendations.push({
      type: 'module',
      title: 'Vision-Language-Action Models',
      url: '/docs/module-4-vla/overview',
      reason: 'Matches your AI interests'
    });
  }
  
  if (profile?.interests?.includes('simulation')) {
    recommendations.push({
      type: 'module',
      title: 'Unity High-Fidelity Rendering',
      url: '/docs/module-2-simulation/unity-rendering',
      reason: 'Advanced simulation techniques'
    });
  }
  
  // Based on goals
  if (profile?.goals?.includes('career-change')) {
    recommendations.push({
      type: 'resource',
      title: 'Career Outcomes by Path',
      url: '/docs/resources/learning-paths#career-outcomes-by-path',
      reason: 'Plan your career transition'
    });
  }
  
  return recommendations;
}

/**
 * Track module completion
 */
export async function trackModuleCompletion(userId, moduleId) {
  const { data, error } = await supabase
    .from('user_progress')
    .insert({
      user_id: userId,
      module_id: moduleId,
      completed_at: new Date().toISOString(),
      progress_percentage: 100
    })
    .select()
    .single();
    
  if (error) {
    console.error('Error tracking completion:', error);
    return null;
  }
  
  // Update activity log
  await supabase
    .from('activity_log')
    .insert({
      user_id: userId,
      activity_type: 'module_completed',
      metadata: { module_id: moduleId }
    });
  
  return data;
}

/**
 * Get user statistics
 */
export async function getUserStats(userId) {
  // Get progress
  const { data: progress } = await supabase
    .from('user_progress')
    .select('*')
    .eq('user_id', userId);
  
  // Get activity log
  const { data: activities } = await supabase
    .from('activity_log')
    .select('*')
    .eq('user_id', userId)
    .order('created_at', { ascending: false });
  
  // Calculate stats
  const stats = {
    modulesCompleted: progress?.length || 0,
    currentStreak: calculateStreak(activities || []),
    totalTimeSpent: activities?.reduce((sum, a) => sum + (a.time_spent || 0), 0) || 0,
    lastActive: activities?.[0]?.created_at || null,
    badges: calculateBadges(progress || []),
    skillLevel: calculateSkillLevel(progress || [])
  };
  
  return stats;
}

/**
 * Calculate earned badges
 */
function calculateBadges(progress) {
  const badges = [];
  
  if (progress.length >= 1) {
    badges.push({ name: 'First Steps', icon: '🥉', description: 'Completed first module' });
  }
  
  if (progress.length >= 4) {
    badges.push({ name: 'Module Master', icon: '🥈', description: 'Completed all modules' });
  }
  
  if (progress.some(p => p.module_id.includes('capstone'))) {
    badges.push({ name: 'Capstone Complete', icon: '🥇', description: 'Finished capstone project' });
  }
  
  return badges;
}

/**
 * Calculate skill level based on progress
 */
function calculateSkillLevel(progress) {
  const completionRate = progress.length / 6; // Assuming 6 total modules
  
  if (completionRate < 0.25) return 'Novice';
  if (completionRate < 0.5) return 'Intermediate';
  if (completionRate < 0.75) return 'Advanced';
  return 'Expert';
}

/**
 * Get personalized dashboard data
 */
export async function getDashboardData(userId) {
  const profile = await getUserProfile(userId);
  
  const { data: progress } = await supabase
    .from('user_progress')
    .select('*')
    .eq('user_id', userId);
  
  const stats = await getUserStats(userId);
  const path = getRecommendedPath(profile);
  const nextModules = getNextModules(progress || [], profile);
  const recommendations = getContentRecommendations(profile, progress || []);
  
  return {
    profile,
    progress: progress || [],
    stats,
    recommendedPath: path,
    nextModules,
    recommendations
  };
}