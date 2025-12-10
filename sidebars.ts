// sidebars.js
import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'doc',
      id: 'prerequisites',
      label: 'Prerequisites',
    },
    {
      type: 'doc',
      id: 'learning-path',
      label: 'Learning Path',
    },
    {
      type: 'doc',
      id: 'hardware-guide',
      label: 'Hardware Guide',
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      items: [
        'module-1-ros2/overview',
        'module-1-ros2/ros2-architecture',
        'module-1-ros2/nodes-topics-services',
        'module-1-ros2/rclpy-python-agents',
        'module-1-ros2/urdf-humanoids',
        'module-1-ros2/practical-1-ros2-setup',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation',
      items: [
        'module-2-simulation/overview',
        'module-2-simulation/gazebo-physics',
        'module-2-simulation/unity-rendering',
        'module-2-simulation/sensor-simulation',
        'module-2-simulation/environment-building',
        'module-2-simulation/practical-2-digital-twin',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac',
      items: [
        'module-3-isaac/overview',
        'module-3-isaac/isaac-sim',
        'module-3-isaac/isaac-ros',
        'module-3-isaac/vslam-navigation',
        'module-3-isaac/nav2-bipedal',
        'module-3-isaac/practical-3-perception',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA',
      items: [
        'module-4-vla/overview',
        'module-4-vla/whisper-voice',
        'module-4-vla/llm-planning',
        'module-4-vla/multimodal-integration',
        'module-4-vla/capstone-project',
        'module-4-vla/practical-4-autonomous-humanoid',
      ],
    },
    {
      type: 'category',
      label: 'Resources',
      items: [
        'resources/hardware-setup',
        'resources/cloud-lab',
        'resources/assessments',
        'resources/references',
        'resources/faq',
        'resources/glossary',
      ],
    },
  ],
};

export default sidebars;
