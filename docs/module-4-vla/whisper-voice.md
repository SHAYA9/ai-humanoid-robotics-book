---
sidebar_position: 2
title: Whisper Voice Control
---

# Voice Control with Whisper

**Whisper** is a state-of-the-art automatic speech recognition (ASR) system developed by OpenAI. It's incredibly versatile and can transcribe spoken language from a wide range of audio inputs, making it an excellent choice for adding voice control to a robot.

## How it Works

Whisper is a deep learning model that has been trained on a massive dataset of audio and text. It takes an audio waveform as input and outputs the corresponding text. You can run Whisper locally on your own hardware (including a Jetson), or you can use OpenAI's API.

## Integrating Whisper with ROS 2

To use Whisper for robot control, you'll need to create a ROS 2 node that:

1.  **Captures audio** from a microphone.
2.  **Sends the audio** to the Whisper model for transcription.
3.  **Receives the transcribed text.**
4.  **Publishes the text** to a ROS 2 topic.

Other nodes in your system can then subscribe to this topic to receive the voice commands and act on them.

## Example Use Case

Imagine you have a robotic arm. You could create a voice control system that allows you to say things like:

*   "Pick up the red cube."
*   "Move to your home position."
*   "Open the gripper."

The Whisper node would transcribe these commands, and another node would parse the text and generate the appropriate motor commands for the robotic arm.
