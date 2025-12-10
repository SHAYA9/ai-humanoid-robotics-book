// A simple recommendation engine based on user progress and interests.

export function recommendNextModules(completedModules, userPreferences, allModules) {
  const completedModuleIds = new Set(completedModules.map(m => m.module_id));
  const preferredTopics = new Set(userPreferences.preferred_topics || []);

  const recommendations = allModules
    .filter(m => !completedModuleIds.has(m.id)) // Filter out completed modules
    .map(m => {
      let score = 0;
      if (preferredTopics.has(m.topic)) {
        score += 10; // Boost score for preferred topics
      }
      // Add more sophisticated scoring based on module prerequisites, etc.
      return { ...m, score };
    })
    .sort((a, b) => b.score - a.score); // Sort by score

  return recommendations.slice(0, 3); // Return top 3 recommendations
}
