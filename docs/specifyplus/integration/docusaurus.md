# Integration: Docusaurus

Integrating outputs from the SpecifyPlus robotics system into a Docusaurus website can greatly enhance documentation by providing dynamic, visual content. This guide covers strategies for embedding robotics data and visualizations.

## Overview

The goal is to bridge the gap between the ROS 2 environment and the web-based Docusaurus environment. This typically involves converting ROS 2 data into web-friendly formats (images, videos, JSON) and then embedding them in your Markdown files.

## Strategy 1: Recording and Embedding Videos

The easiest way to demonstrate a robot's behavior is through video.

1.  **Record a Simulation:**
    -   Use a screen recording tool (like OBS Studio or Kazam) to capture a video of the robot running in Gazebo or Isaac Sim.
    -   Alternatively, use a ROS 2 tool like `ros2 bag` to record all the topic data from a run. You can then play this bag back and record a clean video from RViz without having to run the live system.

2.  **Convert to a Web-Friendly Format:**
    -   Ensure your video is in a format like MP4. Use a tool like `ffmpeg` if you need to convert it.

3.  **Embed in Docusaurus:**
    -   Place the video file in your `static/img` directory.
    -   Use HTML5 `<video>` tags in your Markdown file to embed it.

**Example Markdown:**

```markdown
Here is a demonstration of the humanoid robot walking:

<video width="100%" controls>
  <source src="/img/humanoid-walking-demo.mp4" type="video/mp4" />
  Your browser does not support the video tag.
</video>
```

## Strategy 2: Visualizing Data with Plots

For showing sensor data or performance metrics, static plots are very effective.

1.  **Log Data:**
    -   Use `ros2 bag record` to save data from topics (e.g., `/joint_states`, `/odometry`).
    -   Alternatively, have a Python node subscribe to a topic and write the data to a CSV file.

2.  **Generate Plots:**
    -   Use a Python library like Matplotlib or Plotly to generate plots from the logged data. Save the plots as images (PNG, SVG).

3.  **Embed in Docusaurus:**
    -   Place the image files in `static/img`.
    -   Use standard Markdown image syntax.

**Example Markdown:**

```markdown
The following plot shows the joint angle of the elbow during a grasp action.

![Elbow Joint Angle](/img/elbow-joint-plot.png)
```

## Strategy 3: Interactive Visualizations with Web Tools

For more advanced, interactive content, you can use web-native visualization libraries.

1.  **Bridge ROS Data to the Web:**
    -   The `ros2-web-bridge` provides a WebSocket server that exposes the ROS 2 graph to web clients. This allows a web page to subscribe and publish to ROS 2 topics in real-time.

2.  **Create a React Component:**
    -   Docusaurus is built on React. You can create a custom React component that uses a library like `roslib.js` to connect to the `ros2-web-bridge`.
    -   Inside your component, you can use libraries like `three.js` (for 3D models) or `echarts` (for plotting) to visualize the live ROS 2 data.

3.  **Use the Component in Docusaurus:**
    -   "Sling" your React component so Docusaurus can use it. You can now use your component directly in your `.mdx` files.

**Example MDX file:**

```mdx
import LiveRobotViewer from '@site/src/components/LiveRobotViewer';

## Live Robot Status

The component below shows a live 3D model of the robot, updated in real-time from the simulation.

<LiveRobotViewer />
```

This approach is the most complex but also the most powerful, as it allows you to create live dashboards and interactive tutorials directly within your documentation.
