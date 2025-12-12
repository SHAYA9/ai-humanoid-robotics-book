import type { ReactNode } from 'react';
import { useState, useEffect } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { useAuth } from '@site/src/contexts/AuthContext';

import styles from './index.module.css';

// Add particle background component
function ParticleBackground() {
  return (
    <BrowserOnly>
      {() => {
        const { useEffect, useRef } = require('react');
        const ParticleCanvas = () => {
          const canvasRef = useRef<HTMLCanvasElement>(null);
          
          useEffect(() => {
            const canvas = canvasRef.current;
            if (!canvas) return;
            
            const ctx = canvas.getContext('2d');
            if (!ctx) return;
            
            canvas.width = canvas.offsetWidth;
            canvas.height = canvas.offsetHeight;
            
            const particles: Array<{
              x: number;
              y: number;
              size: number;
              speedX: number;
              speedY: number;
              opacity: number;
            }> = [];
            
            const particleCount = 30;
            
            for (let i = 0; i < particleCount; i++) {
              particles.push({
                x: Math.random() * canvas.width,
                y: Math.random() * canvas.height,
                size: Math.random() * 2 + 0.5,
                speedX: Math.random() * 0.5 - 0.25,
                speedY: Math.random() * 0.5 - 0.25,
                opacity: Math.random() * 0.3 + 0.1,
              });
            }
            
            let animationId: number;
            
            const animate = () => {
              ctx.clearRect(0, 0, canvas.width, canvas.height);
              
              particles.forEach(particle => {
                particle.x += particle.speedX;
                particle.y += particle.speedY;
                
                // Boundary check
                if (particle.x < 0) particle.x = canvas.width;
                if (particle.x > canvas.width) particle.x = 0;
                if (particle.y < 0) particle.y = canvas.height;
                if (particle.y > canvas.height) particle.y = 0;
                
                ctx.beginPath();
                ctx.arc(particle.x, particle.y, particle.size, 0, Math.PI * 2);
                ctx.fillStyle = `rgba(255, 255, 255, ${particle.opacity})`;
                ctx.fill();
              });
              
              animationId = requestAnimationFrame(animate);
            };
            
            animate();
            
            const handleResize = () => {
              canvas.width = canvas.offsetWidth;
              canvas.height = canvas.offsetHeight;
            };
            
            window.addEventListener('resize', handleResize);
            
            return () => {
              cancelAnimationFrame(animationId);
              window.removeEventListener('resize', handleResize);
            };
          }, []);
          
          return (
            <canvas
              ref={canvasRef}
              className={styles.particleCanvas}
              aria-hidden="true"
            />
          );
        };
        return <ParticleCanvas />;
      }}
    </BrowserOnly>
  );
}

// Add typing animation component
function TypingAnimation({ texts, speed = 100 }: { texts: string[]; speed?: number }) {
  const [displayText, setDisplayText] = useState('');
  const [textIndex, setTextIndex] = useState(0);
  const [charIndex, setCharIndex] = useState(0);
  const [isDeleting, setIsDeleting] = useState(false);
  
  useEffect(() => {
    const currentText = texts[textIndex];
    
    const timeout = setTimeout(() => {
      if (!isDeleting) {
        if (charIndex < currentText.length) {
          setDisplayText(currentText.substring(0, charIndex + 1));
          setCharIndex(charIndex + 1);
        } else {
          setTimeout(() => setIsDeleting(true), 1500);
        }
      } else {
        if (charIndex > 0) {
          setDisplayText(currentText.substring(0, charIndex - 1));
          setCharIndex(charIndex - 1);
        } else {
          setIsDeleting(false);
          setTextIndex((textIndex + 1) % texts.length);
        }
      }
    }, isDeleting ? speed / 2 : speed);
    
    return () => clearTimeout(timeout);
  }, [charIndex, isDeleting, textIndex, texts, speed]);
  
  return (
    <span className={styles.typingText}>
      {displayText}
      <span className={styles.cursor}>|</span>
    </span>
  );
}

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  const { user } = useAuth();
  const [scrollY, setScrollY] = useState(0);
  
  useEffect(() => {
    const handleScroll = () => {
      setScrollY(window.scrollY);
    };
    
    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);
  
  const typingTexts = [
    'AI & Robotics',
    'Humanoid Design',
    'Sensor Integration',
    'Robotics Programming',
    'Future Tech',
  ];

  return (
    <header 
      className={clsx(styles.heroBanner)}
      style={{
        '--scroll-parallax': `${Math.min(scrollY * 0.5, 200)}px`,
      } as React.CSSProperties}
    >
      <div className={styles.heroBackground}>
        <div className={styles.gradientOverlay} />
        <ParticleBackground />
      </div>
      
      <div className={styles.inner}>
        <div className={styles.titleWrapper}>
          <Heading as="h1" className={styles.title}>
            {siteConfig.title}
          </Heading>
          <div className={styles.titleHighlight} aria-hidden="true" />
        </div>

        <p className={styles.subtitle}>
          Master{' '}
          <TypingAnimation texts={typingTexts} speed={80} />
        </p>

        <div className={styles.buttons}>
          <Link 
            className={clsx("button button--primary button--lg", styles.primaryButton)}
            to="/docs/intro"
            onClick={() => {
              // Clear session storage flags when logged out user clicks Start Reading
              // This ensures the welcome popup will show on the docs page
              if (!user) {
                console.log('🧹 Clearing session storage for Start Reading click');
                sessionStorage.removeItem('has_visited_docs');
                sessionStorage.removeItem('welcome_popup_seen');
              }
            }}
          >
            <span className={styles.buttonContent}>
              <span className={styles.buttonIcon}>📘</span>
              Start Reading
              <span className={styles.buttonArrow}>→</span>
            </span>
          </Link>

          <Link
            className={clsx("button button--secondary button--lg", styles.secondaryButton)}
            to="/docs/module-1-ros2/overview"
          >
            <span className={styles.buttonContent}>
              <span className={styles.buttonIcon}>🚀</span>
              Explore Modules
            </span>
          </Link>
          
          <Link
            className={clsx("button button--outline button--lg", styles.outlineButton)}
            to="/docs/resources/assessments"
          >
            <span className={styles.buttonContent}>
              <span className={styles.buttonIcon}>🔧</span>
              View Projects
            </span>
          </Link>
        </div>

        <div className={styles.quickStats}>
          {[
            { number: '4', label: 'Comprehensive Modules', icon: '📚' },
            { number: '120+', label: 'Pages of Content', icon: '📖' },
            { number: 'Beginner → Pro', label: 'Learning Path', icon: '🎯' },
            { number: 'Hands-on', label: 'Practical Projects', icon: '🔧' },
          ].map((stat, index) => (
            <div 
              key={index} 
              className={styles.stat}
              style={{ animationDelay: `${index * 0.1}s` }}
            >
              <div className={styles.statIcon} aria-hidden="true">
                {stat.icon}
              </div>
              <div className={styles.statNumber}>{stat.number}</div>
              <div className={styles.statLabel}>{stat.label}</div>
            </div>
          ))}
        </div>

        <div className={styles.scrollIndicator}>
          <span className={styles.scrollText}>Scroll to explore</span>
          <div className={styles.scrollArrow}>
            <span></span>
            <span></span>
            <span></span>
          </div>
        </div>
      </div>
    </header>
  );
}

// Add newsletter signup component
function NewsletterSignup() {
  const [email, setEmail] = useState('');
  const [status, setStatus] = useState<'idle' | 'loading' | 'success' | 'error'>('idle');
  
  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setStatus('loading');
    
    // Simulate API call
    setTimeout(() => {
      setStatus('success');
      setEmail('');
      setTimeout(() => setStatus('idle'), 3000);
    }, 1000);
  };
  
  return (
    <section className={styles.newsletterSection}>
      <div className="container">
        <div className={styles.newsletterCard}>
          <div className={styles.newsletterContent}>
            <h3 className={styles.newsletterTitle}>Stay Updated ✨</h3>
            <p className={styles.newsletterDescription}>
              Get notified about new modules, tutorials, and AI/robotics insights.
            </p>
          </div>
          <form onSubmit={handleSubmit} className={styles.newsletterForm}>
            <div className={styles.formGroup}>
              <input
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                placeholder="Enter your email"
                className={styles.newsletterInput}
                required
                disabled={status === 'loading'}
              />
              <button
                type="submit"
                className={styles.newsletterButton}
                disabled={status === 'loading'}
              >
                {status === 'loading' ? (
                  <span className={styles.spinner}></span>
                ) : status === 'success' ? (
                  '🎉 Subscribed!'
                ) : (
                  'Subscribe'
                )}
              </button>
            </div>
            <p className={styles.newsletterHint}>
              No spam. Unsubscribe anytime.
            </p>
          </form>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`${siteConfig.title} – Learn AI & Robotics`}
      description="Explore the world of AI, humanoid robots, sensors, robotics programming, and future tech. Step-by-step guides with theory + hands-on projects."
    >
      <HomepageHeader />
      <main>
        {/* What You Will Learn Section with Grey Theme */}
        <section className={clsx(styles.section, styles.sectionGrey)}>
          <div className="container">
            <div className={styles.sectionHeader}>
              <div className={styles.sectionBadge}>Curriculum</div>
              <h2 className={styles.sectionTitle}>
                What You Will Learn{' '}
                <span className={styles.titleAccent}>📚</span>
              </h2>
              <p className={styles.sectionLead}>
                A comprehensive curriculum covering AI foundations to advanced robotics. 
                Learn through theory, hands-on projects, and real-world applications.
              </p>
            </div>

            <div className={styles.curriculumGrid}>
              {[
                {
                  icon: '🧠',
                  title: 'AI Foundations',
                  description: 'Master machine learning, neural networks, and AI principles from ground up.',
                  topics: ['Machine Learning', 'Deep Learning', 'Computer Vision', 'NLP Basics'],
                  duration: '6 weeks',
                  color: '#667eea',
                  level: 'Beginner'
                },
                {
                  icon: '🤖',
                  title: 'Humanoid Robotics',
                  description: 'Design, build, and program humanoid robots with advanced mechanics.',
                  topics: ['Robot Kinematics', 'Motion Planning', 'Sensor Fusion', 'Gait Analysis'],
                  duration: '8 weeks',
                  color: '#764ba2',
                  level: 'Intermediate'
                },
                {
                  icon: '🔌',
                  title: 'Sensor Integration',
                  description: 'Integrate various sensors for perception and environment interaction.',
                  topics: ['LiDAR & Depth', 'IMU Sensors', 'Computer Vision', 'Touch Sensors'],
                  duration: '6 weeks',
                  color: '#f093fb',
                  level: 'Intermediate'
                },
                {
                  icon: '💻',
                  title: 'Robotics Programming',
                  description: 'Master ROS, Python, and C++ for robotics development.',
                  topics: ['ROS 2', 'Python Robotics', 'C++ Optimization', 'Real-time Systems'],
                  duration: '8 weeks',
                  color: '#4facfe',
                  level: 'Advanced'
                },
                {
                  icon: '🚀',
                  title: 'Advanced AI',
                  description: 'Learn reinforcement learning, autonomous systems, and AI ethics.',
                  topics: ['Reinforcement Learning', 'Autonomous Navigation', 'AI Ethics', 'Edge AI'],
                  duration: '7 weeks',
                  color: '#43e97b',
                  level: 'Advanced'
                },
                {
                  icon: '🏆',
                  title: 'Capstone Project',
                  description: 'Build a complete AI-powered robotics project from scratch.',
                  topics: ['Project Planning', 'Team Collaboration', 'Testing & Deployment', 'Documentation'],
                  duration: '10 weeks',
                  color: '#fa709a',
                  level: 'Expert'
                }
              ].map((module, index) => (
                <div key={index} className={styles.moduleCard}>
                  <div className={styles.moduleHeader}>
                    <div className={styles.moduleIcon} style={{ backgroundColor: `${module.color}20` }}>
                      <span style={{ color: module.color, fontSize: '1.8rem' }}>{module.icon}</span>
                    </div>
                    <div className={styles.moduleTitleWrapper}>
                      <h3 className={styles.moduleTitle}>{module.title}</h3>
                      <div className={styles.moduleLevel} style={{ backgroundColor: `${module.color}20`, color: module.color }}>
                        {module.level}
                      </div>
                    </div>
                  </div>
                  
                  <p className={styles.moduleDescription}>{module.description}</p>
                  
                  <div className={styles.moduleTopics}>
                    <h4 className={styles.topicsTitle}>Topics Covered:</h4>
                    <div className={styles.topicTags}>
                      {module.topics.map((topic, idx) => (
                        <span key={idx} className={styles.topicTag}>{topic}</span>
                      ))}
                    </div>
                  </div>
                  
                  <div className={styles.moduleFooter}>
                    <div className={styles.moduleDuration}>
                      <span className={styles.durationIcon}>⏱️</span>
                      <span>{module.duration}</span>
                    </div>
                    <Link 
                      to={`/docs/modules/${module.title.toLowerCase().replace(/\s+/g, '-')}`}
                      className={styles.moduleLink}
                      style={{ color: module.color }}
                    >
                      Explore Module →
                    </Link>
                  </div>
                </div>
              ))}
            </div>
            
            <div className={styles.featuresSection}>
              <HomepageFeatures />
            </div>
          </div>
        </section>

        <NewsletterSignup />
        
        <section className={clsx(styles.section, styles.sectionDark)}>
          <div className="container">
            <h2 className={styles.sectionTitle}>Learning Path 🗺️</h2>
            <div className={styles.learningPath}>
              {[
                { step: '01', title: 'AI Fundamentals', desc: 'Understand core AI concepts', duration: '2 weeks' },
                { step: '02', title: 'Robotics Basics', desc: 'Learn robot mechanics & electronics', duration: '3 weeks' },
                { step: '03', title: 'Programming', desc: 'Master robotics programming', duration: '4 weeks' },
                { step: '04', title: 'Advanced Projects', desc: 'Build real-world applications', duration: '5 weeks' },
              ].map((stage, index) => (
                <div key={index} className={styles.pathStage}>
                  <div className={styles.pathStep}>{stage.step}</div>
                  <div className={styles.pathContent}>
                    <h4 className={styles.pathTitle}>{stage.title}</h4>
                    <p className={styles.pathDesc}>{stage.desc}</p>
                    <div className={styles.pathDuration}>
                      <span className={styles.durationIcon}>⏱️</span>
                      {stage.duration}
                    </div>
                  </div>
                  {index < 3 && <div className={styles.pathConnector}></div>}
                </div>
              ))}
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}