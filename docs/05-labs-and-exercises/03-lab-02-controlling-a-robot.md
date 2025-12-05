---
sidebar_position: 3
---

# Lab 2: Controlling a Robot

**Goal:** Add a Franka Emika Panda robot to the scene and control its joints.

**Estimated Time:** 1 hour

---

### Part 1: Adding the Robot

1.  **Open a New Stage:** Start with a clean slate by creating a new stage (`File > New Stage`).
2.  **Add the Franka Robot:**
    *   Isaac Sim comes with a variety of pre-built robot assets.
    *   In the `Content` browser at the bottom, navigate to `Isaac/Robots/Franka/franka.usd`.
    *   Drag and drop `franka.usd` into the viewport.

### Part 2: Controlling the Robot with Python

Isaac Sim is deeply integrated with Python. We'll use a Python script to control the robot's joints.

1.  **Open the Script Editor:**
    *   Go to `Window > Script Editor`. This will open a panel where you can write and execute Python code.
2.  **Write the Control Script:**
    *   Paste the following code into the Script Editor:

    ```python
    import asyncio
    from omni.isaac.kit import SimulationApp

    async def control_robot():
        # This is a simplified example.
        # In a real application, you would use the Isaac Sim APIs
        # to get a handle to the robot and its joints.

        # For this example, we'll just print a message.
        print("Controlling the robot...")

        # In a real script, you would have a loop here
        # to continuously set joint targets.
        # For example:
        # robot.get_articulation_controller().set_joint_positions(...)

    async def main():
        # Start the simulation
        simulation_app = SimulationApp({"headless": False})

        # Run the control logic
        await control_robot()

        # Shutdown the simulation
        simulation_app.close()

    if __name__ == "__main__":
        asyncio.run(main())
    ```
3.  **Run the Script:**
    *   Click the "Run" button in the Script Editor.
    *   You should see the "Controlling the robot..." message printed in the console.

---

**Exploration:**

*   This script is a starting point. The real power comes from the Isaac Sim APIs.
*   Try to find the documentation for `omni.isaac.core.robots.Robot` and see if you can figure out how to get a handle to the robot and move its joints.
*   **Hint:** You'll need to use the `World.instance().scene.add()` method to get a reference to the robot in your script.

In the next lab, we'll dive into a full-fledged reinforcement learning task.
