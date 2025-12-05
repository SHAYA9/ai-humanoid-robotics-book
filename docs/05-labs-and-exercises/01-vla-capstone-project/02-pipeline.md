---
sidebar_position: 2
---

# 8.2 The Whisper → LLM → ROS 2 Pipeline

The core of our VLA system is the data flow that converts human speech into a series of executable robot commands. This pipeline consists of three main stages, each handled by a dedicated ROS 2 node.

### 1. Speech-to-Text (Whisper)

The first step is to convert spoken language into text. We will use **OpenAI's Whisper**, a state-of-the-art speech recognition model.

-   **Node:** A `WhisperNode` will be responsible for this stage.
-   **Input:** It will listen to an audio stream from a microphone (or a pre-recorded audio file for testing).
-   **Processing:** It uses a local or API-based version of Whisper to transcribe the audio.
-   **Output:** The transcribed text is published as a `std_msgs/msg/String` to a ROS 2 topic named `/voice_command`.

### 2. Text-to-Plan (Large Language Model)

This is the "cognitive" core of our system. This stage takes the transcribed text and converts it into a structured plan that the robot can understand.

-   **Node:** A `CognitivePlannerNode` subscribes to the `/voice_command` topic.
-   **Processing:** When it receives a text command, it queries a **Large Language Model (LLM)**, such as GPT-4 or a local model like Llama 3. The magic happens in the **prompt**: we will instruct the LLM to act as a robotics planner and convert the user's command into a JSON array of specific, pre-defined robot "skills" or functions.
-   **Example:**
    -   **Input Text:** "put the apple on the plate"
    -   **LLM Output (JSON):**
        ```json
        [
          {"skill": "find", "object": "apple"},
          {"skill": "pick", "object": "apple"},
          {"skill": "find", "object": "plate"},
          {"skill": "place", "target": "plate"}
        ]
        ```

### 3. Plan-to-Action (ROS 2)

Once the `CognitivePlannerNode` has the structured JSON plan from the LLM, it acts as an executor, translating each step of the plan into ROS 2 commands.

-   **Execution Loop:** The node iterates through the array of skills.
-   **Dispatching:** For each skill, it makes the appropriate ROS 2 service call or action request to the nodes responsible for the robot's low-level capabilities (vision, motion, grasping).
    -   `{"skill": "find", ...}` → Call the `/vision/find_object` service.
    -   `{"skill": "pick", ...}` → Call the `/motion/move_to` and `/gripper/close` services.

*<p align="center">PLACEHOLDER: VLA Data Flow Diagram</p>*
*<p align="center">A diagram showing the three stages: A microphone icon inputs to the Whisper Node, which publishes a String to the `/voice_command` topic. The Cognitive Planner Node subscribes to this, communicates with an LLM (cloud icon), and then makes ROS 2 service/action calls to the Vision, Motion, and Gripper nodes.</p>*

This pipeline effectively decouples the complexity of language understanding from the robot's core functionalities. The LLM acts as a high-level "brain," translating human intent into a structured format that the robot's more deterministic systems can execute.
