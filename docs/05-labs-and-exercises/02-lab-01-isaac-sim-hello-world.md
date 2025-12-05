---
sidebar_position: 2
---

# Lab 1: Isaac Sim - Hello World

**Goal:** Create a simple "Hello World" scene in Isaac Sim to get familiar with the interface and basic concepts.

**Estimated Time:** 30 minutes

---

### Part 1: Setting up the Scene

1.  **Launch Isaac Sim:** Open the Omniverse Launcher and launch Isaac Sim.
2.  **Create a New Stage:** Go to `File > New Stage`. This will give you a blank canvas to work with.
3.  **Add a Ground Plane:**
    *   Go to `Create > Physics > Ground Plane`. This will add a flat surface to your scene.
4.  **Add a Cube:**
    *   Go to `Create > Shape > Cube`. A cube will appear at the center of your scene.
5.  **Position the Cube:**
    *   Select the cube in the `Stage` panel on the right.
    *   In the `Property` panel below, find the `Transform` section.
    *   Set the `Translate` values to `(X: 0, Y: 0, Z: 50)`. This will move the cube 50 units up in the Z-axis.

### Part 2: Adding Physics

1.  **Make the Cube a Rigid Body:**
    *   With the cube selected, go to `Add > Physics > Rigid Body with Colliders Preset`. This will allow the cube to be affected by gravity.
2.  **Press Play:**
    *   Click the "Play" button at the top of the viewport.
    *   You should see the cube fall and land on the ground plane.

---

**Congratulations!** You've created your first physics simulation in Isaac Sim. In the next lab, we'll add a robot to the scene.
