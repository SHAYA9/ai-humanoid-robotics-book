import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/ai-humanoid-robotics-book/markdown-page',
    component: ComponentCreator('/ai-humanoid-robotics-book/markdown-page', '420'),
    exact: true
  },
  {
    path: '/ai-humanoid-robotics-book/docs',
    component: ComponentCreator('/ai-humanoid-robotics-book/docs', '5c8'),
    routes: [
      {
        path: '/ai-humanoid-robotics-book/docs',
        component: ComponentCreator('/ai-humanoid-robotics-book/docs', '792'),
        routes: [
          {
            path: '/ai-humanoid-robotics-book/docs',
            component: ComponentCreator('/ai-humanoid-robotics-book/docs', '4cf'),
            routes: [
              {
                path: '/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/mlops-for-robotics/domain-randomization',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/mlops-for-robotics/domain-randomization', '529'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/mlops-for-robotics/gpu-requirements',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/mlops-for-robotics/gpu-requirements', '411'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/mlops-for-robotics/introduction-to-nvidia-isaac',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/mlops-for-robotics/introduction-to-nvidia-isaac', 'af8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/mlops-for-robotics/isaac-sim-pipeline',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/mlops-for-robotics/isaac-sim-pipeline', 'd69'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/mlops-for-robotics/labs-and-benchmarks',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/mlops-for-robotics/labs-and-benchmarks', 'a92'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/mlops-for-robotics/nav2-path-planning',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/mlops-for-robotics/nav2-path-planning', 'eb0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/mlops-for-robotics/reinforcement-learning',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/mlops-for-robotics/reinforcement-learning', 'da1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/mlops-for-robotics/sim2real',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/mlops-for-robotics/sim2real', '31f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/mlops-for-robotics/synthetic-data-generation',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/mlops-for-robotics/synthetic-data-generation', 'f25'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/mlops-for-robotics/vslam-with-isaac-ros',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/mlops-for-robotics/vslam-with-isaac-ros', '4b0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/simulation-environments/importance-of-simulation',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/simulation-environments/importance-of-simulation', '4bc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/simulation-environments/labs',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/simulation-environments/labs', 'a57'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/simulation-environments/physics-simulation-concepts',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/simulation-environments/physics-simulation-concepts', '791'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/simulation-environments/sensor-simulation',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/simulation-environments/sensor-simulation', '64f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/simulation-environments/unity-visualization-plan',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/autonomous-behaviors-multi-modal-sensors-and-cloud-ops/simulation-environments/unity-visualization-plan', '50a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/category/tutorial---basics',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/category/tutorial---basics', '550'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/category/tutorial---extras',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/category/tutorial---extras', '125'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/intro',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/intro', '3be'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/introduction/how-to-use-this-book',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/introduction/how-to-use-this-book', 'cf5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/introduction/safety-primer',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/introduction/safety-primer', 'e26'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/introduction/welcome',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/introduction/welcome', 'ad9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/labs-and-exercises/lab-01-isaac-sim-hello-world',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/labs-and-exercises/lab-01-isaac-sim-hello-world', '2d9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/labs-and-exercises/lab-02-controlling-a-robot',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/labs-and-exercises/lab-02-controlling-a-robot', '5be'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/labs-and-exercises/lab-03-reinforcement-learning-reach-task',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/labs-and-exercises/lab-03-reinforcement-learning-reach-task', 'a66'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/labs-and-exercises/overview',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/labs-and-exercises/overview', 'f12'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/labs-and-exercises/vla-capstone-project/cognitive-planning',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/labs-and-exercises/vla-capstone-project/cognitive-planning', '7b4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/labs-and-exercises/vla-capstone-project/introduction',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/labs-and-exercises/vla-capstone-project/introduction', 'b42'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/labs-and-exercises/vla-capstone-project/object-detection-and-manipulation',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/labs-and-exercises/vla-capstone-project/object-detection-and-manipulation', 'f68'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/labs-and-exercises/vla-capstone-project/pipeline',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/labs-and-exercises/vla-capstone-project/pipeline', 'a9f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/robotics-foundations/intro-to-robotics/what-is-a-robot',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/robotics-foundations/intro-to-robotics/what-is-a-robot', 'f2a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/robotics-foundations/the-robot-operating-system-ros-2/building-a-ros-2-package',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/robotics-foundations/the-robot-operating-system-ros-2/building-a-ros-2-package', '635'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/robotics-foundations/the-robot-operating-system-ros-2/core-concepts',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/robotics-foundations/the-robot-operating-system-ros-2/core-concepts', '73e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/robotics-foundations/the-robot-operating-system-ros-2/labs',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/robotics-foundations/the-robot-operating-system-ros-2/labs', 'ee9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/robotics-foundations/the-robot-operating-system-ros-2/managing-complexity',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/robotics-foundations/the-robot-operating-system-ros-2/managing-complexity', 'c21'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/robotics-foundations/the-robot-operating-system-ros-2/synchronous-communication',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/robotics-foundations/the-robot-operating-system-ros-2/synchronous-communication', '7db'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/robotics-foundations/the-robot-operating-system-ros-2/visualization-and-debugging',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/robotics-foundations/the-robot-operating-system-ros-2/visualization-and-debugging', '640'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/robotics-foundations/the-robot-operating-system-ros-2/what-is-ros-2',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/robotics-foundations/the-robot-operating-system-ros-2/what-is-ros-2', '590'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/tutorial-basics/congratulations',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/tutorial-basics/congratulations', '9c5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/tutorial-basics/create-a-blog-post',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/tutorial-basics/create-a-blog-post', '840'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/tutorial-basics/create-a-document',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/tutorial-basics/create-a-document', '184'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/tutorial-basics/create-a-page',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/tutorial-basics/create-a-page', '762'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/tutorial-basics/deploy-your-site',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/tutorial-basics/deploy-your-site', '129'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/tutorial-basics/markdown-features',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/tutorial-basics/markdown-features', 'b6f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/tutorial-extras/manage-docs-versions',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/tutorial-extras/manage-docs-versions', '790'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics-book/docs/tutorial-extras/translate-your-site',
                component: ComponentCreator('/ai-humanoid-robotics-book/docs/tutorial-extras/translate-your-site', '5c9'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/ai-humanoid-robotics-book/',
    component: ComponentCreator('/ai-humanoid-robotics-book/', '262'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
