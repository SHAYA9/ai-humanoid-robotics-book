---
sidebar_position: 4
---

# Lab 3: Reinforcement Learning - Reach Task

**Goal:** Train a Franka robot to reach a target using reinforcement learning.

**Estimated Time:** 2 hours

---

This lab is more involved than the previous ones. We will be using the `omni.isaac.gym` extension to train our RL agent.

### Part 1: Setting up the Environment

Isaac Sim provides example environments for common RL tasks. We'll use the `FrankaCabinet` environment as a starting point and modify it for our reach task.

1.  **Locate the Example:**
    *   The RL examples are located in the `python/examples/reinforcement_learning` directory of your Isaac Sim installation.
    *   Find the `franka_cabinet.py` file.
2.  **Create a Copy:**
    *   Make a copy of `franka_cabinet.py` and name it `franka_reach.py`.
3.  **Run the Example:**
    *   To run the training, you'll use the `python.sh` script that comes with Isaac Sim.
    *   Open a terminal and run:
        ```bash
        ./python.sh python/examples/reinforcement_learning/franka_reach.py
        ```
    *   This will start the training process. You should see a window with many Franka robots training in parallel.

### Part 2: Modifying the Environment for a Reach Task

Now, we'll modify `franka_reach.py` to change the task from opening a cabinet to simply reaching a target.

1.  **Simplify the Scene:**
    *   In the `FrankaCabinet` class, find the `set_up_scene` method.
    *   Remove the code that adds the cabinet to the scene. We only need the robot.
2.  **Define the Observation Space:**
    *   In the `get_observations` method, we need to define what the robot "sees". For a reach task, this should include:
        *   The robot's joint positions.
        *   The position of the end-effector (the "hand").
        *   The position of the target.
3.  **Define the Reward Function:**
    *   The `calculate_metrics` method is where you define the reward. A good reward function for a reach task would be:
        *   A dense reward based on the negative distance between the end-effector and the target. The closer it gets, the higher the reward.
        *   A sparse reward for successfully reaching the target.
4.  **Define the Goal:**
    *   At the beginning of each episode, you'll need to randomize the position of the target.

### Part 3: Training the Agent

1.  **Run the Training Again:**
    *   After modifying the script, run it again from the terminal:
        ```bash
        ./python.sh python/examples/reinforcement_learning/franka_reach.py
        ```
2.  **Observe the Training:**
    *   Watch the `mean reward` in the terminal output. It should steadily increase as the robot learns to reach the target.
3.  **Test the Policy:**
    *   Once the training is complete, you can test the learned policy by running the script with the `--test` flag:
        ```bash
        ./python.sh python/examples/reinforcement_learning/franka_reach.py --test
        ```

---

**Congratulations!** You've trained your first RL agent in Isaac Sim. This is a foundational skill for tackling more complex robotics tasks.
