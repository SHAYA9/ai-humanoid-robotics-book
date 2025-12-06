// src/js/enhancements.js

// Reading progress indicator
function initReadingProgress() {
  const progressBar = document.createElement('div');
  progressBar.className = 'reading-progress';
  document.body.appendChild(progressBar);
  
  window.addEventListener('scroll', () => {
    const winScroll = document.body.scrollTop || document.documentElement.scrollTop;
    const height = document.documentElement.scrollHeight - document.documentElement.clientHeight;
    const scrolled = (winScroll / height) * 100;
    progressBar.style.width = scrolled + '%';
  });
}

// Add copy buttons to code blocks
function initCopyButtons() {
  document.querySelectorAll('pre').forEach((pre) => {
    const button = document.createElement('button');
    button.className = 'copyButton';
    button.textContent = 'Copy';
    button.type = 'button';
    
    button.addEventListener('click', async () => {
      const code = pre.querySelector('code').textContent;
      await navigator.clipboard.writeText(code);
      button.textContent = 'Copied!';
      button.classList.add('copied');
      
      setTimeout(() => {
        button.textContent = 'Copy';
        button.classList.remove('copied');
      }, 2000);
    });
    
    pre.appendChild(button);
  });
}

// Smooth scroll for anchor links
function initSmoothScroll() {
  document.querySelectorAll('a[href^="#"]').forEach(anchor => {
    anchor.addEventListener('click', function (e) {
      e.preventDefault();
      const targetId = this.getAttribute('href');
      if (targetId === '#') return;
      
      const targetElement = document.querySelector(targetId);
      if (targetElement) {
        targetElement.scrollIntoView({
          behavior: 'smooth',
          block: 'start'
        });
      }
    });
  });
}

// Initialize everything when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
  initReadingProgress();
  initCopyButtons();
  initSmoothScroll();
  
  // Add fade-in animation to content
  document.querySelectorAll('.markdown > *').forEach((el, i) => {
    el.style.animationDelay = `${i * 0.05}s`;
    el.classList.add('fade-in');
  });
});