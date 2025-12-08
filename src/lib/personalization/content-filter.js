// This is a simplified example. In a real application, you would fetch
// the content's metadata (e.g., from a CMS or from frontmatter).

export function filterContent(content, userSkillLevel) {
  if (!userSkillLevel) {
    // Default to beginner content if no skill level is set
    return content.filter(item => item.difficulty === 'Beginner');
  }

  const skillLevels = ['Beginner', 'Intermediate', 'Advanced'];
  const userLevelIndex = skillLevels.indexOf(userSkillLevel);

  // Users get access to their level and all levels below
  return content.filter(item => {
    const itemLevelIndex = skillLevels.indexOf(item.difficulty);
    return itemLevelIndex <= userLevelIndex;
  });
}
