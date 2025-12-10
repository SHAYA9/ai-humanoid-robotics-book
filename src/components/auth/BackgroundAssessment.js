import React, { useState } from 'react';
import { supabase } from '../../lib/supabaseClient';
import { useAuth } from '../../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';

const hardwareQuestions = [
  'Have you worked with NVIDIA Jetson?',
  'Do you have experience with ROS 2 compatible hardware?',
  'Have you used robotics kits like Unitree or TurtleBot?',
  'Do you have a GPU workstation for simulation?',
];

const softwareQuestions = [
  'What is your ROS 2 proficiency level? (Beginner, Intermediate, Advanced)',
  'What is your experience with Python/C++? (Beginner, Intermediate, Advanced)',
  'Have you worked with Gazebo or Unity for simulation?',
  'Do you have experience with AI/ML frameworks (e.g., TensorFlow, PyTorch)?',
];

const learningGoalsQuestions = [
  'Which modules are you most interested in? (ROS 2, Simulation, Isaac, VLA)',
  'What project complexity do you desire? (Low, Medium, High)',
  'What is your weekly time commitment? (1-5, 5-10, 10+ hours)',
];

export default function BackgroundAssessment() {
  const { user } = useAuth();
  const history = useHistory();
  const [step, setStep] = useState(0);
  const [answers, setAnswers] = useState({});
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  const questions = [hardwareQuestions, softwareQuestions, learningGoalsQuestions];
  const questionKeys = ['hardware_experience', 'software_experience', 'learning_goals'];

  function handleAnswer(question, answer) {
    setAnswers((prev) => ({ ...prev, [question]: answer }));
  }

  function nextStep() {
    setStep((prev) => prev + 1);
  }

  async function handleSubmit() {
    setLoading(true);
    setError('');

    // Simple logic to determine skill level
    let score = 0;
    if (answers['What is your ROS 2 proficiency level? (Beginner, Intermediate, Advanced)'] === 'Intermediate') score++;
    if (answers['What is your ROS 2 proficiency level? (Beginner, Intermediate, Advanced)'] === 'Advanced') score += 2;
    if (answers['What is your experience with Python/C++? (Beginner, Intermediate, Advanced)'] === 'Intermediate') score++;
    if (answers['What is your experience with Python/C++? (Beginner, Intermediate, Advanced)'] === 'Advanced') score += 2;
    const skillLevel = score < 2 ? 'Beginner' : score < 4 ? 'Intermediate' : 'Advanced';


    try {
      const { error: bgError } = await supabase.from('user_backgrounds').insert({
        id: user.id,
        ...answers,
      });

      if (bgError) throw bgError;

      const { error: profileError } = await supabase
        .from('user_profiles')
        .update({ skill_level: skillLevel })
        .eq('id', user.id);

      if (profileError) throw profileError;

      history.push('/dashboard');
    } catch (error) {
      setError(error.message);
    } finally {
      setLoading(false);
    }
  }

  return (
    <div>
      <h2>Background Assessment</h2>
      {error && <p style={{ color: 'red' }}>{error}</p>}

      {step < questions.length && (
        <>
          <h3>{questionKeys[step].replace('_', ' ')}</h3>
          {questions[step].map((q) => (
            <div key={q}>
              <p>{q}</p>
              {/* This is a simple implementation. You'd want more complex inputs. */}
              {q.includes('(') ? (
                <select onChange={(e) => handleAnswer(q, e.target.value)} required>
                  <option value="">Select an option</option>
                  {q.substring(q.indexOf('(') + 1, q.indexOf(')')).split(', ').map(opt => <option key={opt} value={opt}>{opt}</option>)}
                </select>
              ) : (
                <div>
                  <label><input type="radio" name={q} value="Yes" onChange={(e) => handleAnswer(q, e.target.value)} required /> Yes</label>
                  <label><input type="radio" name={q} value="No" onChange={(e) => handleAnswer(q, e.target.value)} /> No</label>
                </div>
              )}
            </div>
          ))}
          <button onClick={nextStep}>Next</button>
        </>
      )}

      {step === questions.length && (
        <button onClick={handleSubmit} disabled={loading}>
          {loading ? 'Saving...' : 'Complete Profile'}
        </button>
      )}
    </div>
  );
}
