import axios from 'axios';

const STORAGE_KEY = 'user_personalization_preferences';

/**
 * Get user preferences from localStorage
 */
export const getUserPreferences = (userId) => {
  try {
    const stored = localStorage.getItem(`${STORAGE_KEY}_${userId}`);
    return stored ? JSON.parse(stored) : null;
  } catch (error) {
    console.error('Error loading preferences:', error);
    return null;
  }
};

/**
 * Save user preferences to localStorage
 */
export const saveUserPreferences = (userId, preferences) => {
  try {
    localStorage.setItem(`${STORAGE_KEY}_${userId}`, JSON.stringify(preferences));
  } catch (error) {
    console.error('Error saving preferences:', error);
  }
};

/**
 * Extract text content from HTML for personalization
 */
const extractContent = (htmlContent) => {
  const parser = new DOMParser();
  const doc = parser.parseFromString(htmlContent, 'text/html');
  
  // Get main text content, preserving structure
  const headings = Array.from(doc.querySelectorAll('h1, h2, h3, h4, h5, h6'))
    .map(h => ({ tag: h.tagName, text: h.textContent }));
  
  const paragraphs = Array.from(doc.querySelectorAll('p'))
    .map(p => p.textContent);
  
  const lists = Array.from(doc.querySelectorAll('ul, ol'))
    .map(list => Array.from(list.querySelectorAll('li')).map(li => li.textContent));
  
  return {
    headings,
    paragraphs,
    lists,
    fullHtml: htmlContent
  };
};

/**
 * Build personalization prompt based on user preferences
 */
const buildPersonalizationPrompt = (content, preferences, pageTitle) => {
  const { difficulty, background, interests, learningStyle, customization } = preferences;
  
  let prompt = `You are an expert educational content personalizer. Personalize the following educational content about "${pageTitle}" based on these user preferences:

**Difficulty Level:** ${difficulty}
**User Background:** ${background.length > 0 ? background.join(', ') : 'General'}
**Interests/Projects:** ${interests || 'Not specified'}
**Learning Style:** ${learningStyle}

**Customization Requests:**
${customization.moreExamples ? '- Add more practical, relevant examples\n' : ''}${customization.relateToProjects ? '- Relate concepts to user\'s interests and projects\n' : ''}${customization.simplifyJargon ? '- Simplify technical jargon and explain terms clearly\n' : ''}${customization.addVisuals ? '- Suggest where visual aids or diagrams would help\n' : ''}

**Instructions:**
1. Adjust the complexity and depth based on the difficulty level
2. Use examples and analogies relevant to the user's background
3. If user has specific interests, relate concepts to those areas
4. Match the learning style (visual/practical/theoretical/balanced)
5. Preserve all code blocks exactly as they are
6. Maintain the HTML structure and formatting
7. Keep technical accuracy while making content more accessible
8. Add helpful notes, tips, or "💡 For ${background[0] || 'you'}" sections where relevant

**Original Content:**
${content.fullHtml}

**Personalized Content (return only HTML, no explanations):**`;

  return prompt;
};

/**
 * Personalize content using Hugging Face Inference API (free)
 */
export const personalizeContent = async (htmlContent, preferences, pageTitle) => {
  try {
    const content = extractContent(htmlContent);
    
    // For free tier, we'll use a simpler approach with text transformation
    // Using Hugging Face's free inference API
    const prompt = buildPersonalizationPrompt(content, preferences, pageTitle);
    
    // Use Hugging Face's free API (no key required for public models)
    const response = await axios.post(
      'https://api-inference.huggingface.co/models/mistralai/Mixtral-8x7B-Instruct-v0.1',
      {
        inputs: prompt,
        parameters: {
          max_new_tokens: 2000,
          temperature: 0.7,
          return_full_text: false
        }
      },
      {
        headers: {
          'Content-Type': 'application/json'
        },
        timeout: 60000
      }
    );
    
    if (response.data && response.data[0] && response.data[0].generated_text) {
      let personalized = response.data[0].generated_text;
      
      // Clean up the response
      personalized = personalized.replace(/```html\n?/g, '').replace(/```\n?/g, '');
      
      return personalized.trim();
    }
    
    throw new Error('Invalid response from personalization service');
    
  } catch (error) {
    console.error('Personalization error:', error);
    
    if (error.response && error.response.status === 503) {
      throw new Error('Personalization service is loading. Please try again in a few seconds.');
    }
    
    // Fallback: Apply simple text transformations based on preferences
    return applyBasicPersonalization(htmlContent, preferences);
  }
};

/**
 * Fallback: Apply basic personalization without AI
 */
const applyBasicPersonalization = (htmlContent, preferences) => {
  const parser = new DOMParser();
  const doc = parser.parseFromString(htmlContent, 'text/html');
  
  // Add personalization banner
  const banner = doc.createElement('div');
  banner.className = 'personalization-banner';
  banner.innerHTML = `
    <p><strong>📌 Personalized for you:</strong> 
    ${preferences.difficulty} level | 
    ${preferences.background.join(', ') || 'General'} background | 
    ${preferences.learningStyle} learning style
    </p>
  `;
  
  const firstElement = doc.body.firstElementChild;
  if (firstElement) {
    doc.body.insertBefore(banner, firstElement);
  }
  
  // Add difficulty-based notes
  if (preferences.difficulty === 'beginner') {
    const notes = doc.querySelectorAll('p');
    if (notes.length > 0) {
      const tip = doc.createElement('div');
      tip.className = 'beginner-tip';
      tip.innerHTML = '<p><strong>💡 Beginner Tip:</strong> Take your time with this chapter. Don\'t worry if some concepts seem complex at first - they\'ll become clearer with practice!</p>';
      notes[0].parentNode.insertBefore(tip, notes[0]);
    }
  }
  
  return doc.body.innerHTML;
};