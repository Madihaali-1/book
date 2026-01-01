// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/introduction-to-ros2-for-physical-ai',
        'module-1/ros2-communication-model',
        'module-1/robot-structure-with-urdf',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/physics-simulation-with-gazebo',
        'module-2/digital-twins-hri-in-unity',
        'module-2/sensor-simulation-validation',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3/nvidia-isaac-sim-photorealistic-simulation',
        'module-3/isaac-ros-vslam-navigation',
        'module-3/nav2-path-planning-humanoid-robots',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/voice-to-action-openai-whisper',
        'module-4/cognitive-planning-natural-language-ros2',
        'module-4/capstone-project-autonomous-humanoid',
      ],
      collapsed: false,
    },
  ],
};

module.exports = sidebars;