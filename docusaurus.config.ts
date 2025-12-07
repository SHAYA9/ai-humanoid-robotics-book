// docusaurus.config.js
const {themes} = require('prism-react-renderer');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'AI Humanoid Robotics',
  tagline: 'Learn AI and Robotics',
  url: 'https://shaya9.github.io',
  baseUrl: '/ai-humanoid-robotics-book/',
  favicon: 'img/favicon.ico',
  organizationName: 'SHAYA9',
  projectName: 'ai-humanoid-robotics-book',
  
  onBrokenLinks: 'warn',
  
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          routeBasePath: '/docs',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  themeConfig: {
    navbar: {
      title: 'AI Humanoid Robotics',
      logo: {
        alt: 'AI Humanoid Robotics Logo',
        src: 'img/logo.png',
        srcDark: 'img/logo.png',
        href: '/',
        target: '_self',
        width: 35,
        height: 35,
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: '📚 Curriculum',
        },
        
        {
          href: 'https://github.com/SHAYA9/ai-humanoid-robotics-book',
          position: 'right',
          className: 'header-github-link',
          'aria-label': 'GitHub repository',
        },
      ],
    },
    
    // FOOTER SECTION
    footer: {
      style: 'dark',
      
      // Logo in footer (optional)
      logo: {
        alt: 'AI Humanoid Robotics Logo',
        src: 'img/logo.png',
        width: 60,
        height: 60,
        href: 'https://xpertsphere.vercel.app',
      },
      
      // Copyright notice
      copyright: `Copyright © ${new Date().getFullYear()} AI Humanoid Robotics Book. by Xpertsphere`,
      
      // Footer links organized in columns
      links: [
        {
          title: 'Curriculum',
          items: [
            {
              label: 'Getting Started',
              to: '/docs/intro',
            },
            {
              label: 'Module 1: ROS 2',
              to: '/docs/module-1-ros2/overview',
            },
            {
              label: 'Module 2: Simulation',
              to: '/docs/module-2-simulation/overview',
            },
            {
              label: 'Module 3: NVIDIA Isaac',
              to: '/docs/module-3-isaac/overview',
            },
            {
              label: 'Module 4: VLA Integration',
              to: '/docs/module-4-vla/overview',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Hardware Setup',
              to: '/docs/resources/hardware-setup',
            },
            {
              label: 'Cloud Lab',
              to: '/docs/resources/cloud-lab',
            },
            {
              label: 'Assessments',
              to: '/docs/resources/assessments',
            },
            {
              label: 'References',
              to: '/docs/resources/references',
            },
            {
              label: 'FAQ',
              to: '/docs/resources/faq',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/SHAYA9/ai-humanoid-robotics-book',
            },
            {
              label: 'Discord',
              href: 'https://discord.gg/robotics',
            },
            {
              label: 'Twitter / X',
              href: 'https://twitter.com/airoboticsbook',
            },
            {
              label: 'Stack Overflow',
              href: 'https://stackoverflow.com/questions/tagged/ros2',
            },
            {
              label: 'ROS Discourse',
              href: 'https://discourse.ros.org/',
            },
          ],
        },
        {
          title: 'Legal',
          items: [
            {
              label: 'Privacy Policy',
              to: '/docs/legal/privacy',
            },
            {
              label: 'Terms of Use',
              to: '/docs/legal/terms',
            },
            {
              label: 'License',
              to: '/docs/legal/license',
            },
            {
              label: 'Code of Conduct',
              to: '/docs/legal/code-of-conduct',
            },
            {
              label: 'Contact',
              to: '/docs/legal/contact',
            },
          ],
        },
      ],
    },
  },

};

module.exports = config;