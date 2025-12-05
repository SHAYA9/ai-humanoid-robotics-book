---
sidebar_position: 3
---

# 8.3 Cognitive Planning: Natural Language to Actions

The most innovative part of our VLA architecture is the use of a Large Language Model (LLM) as a high-level task planner. The LLM's vast knowledge of language and reasoning allows it to bridge the ambiguous, flexible world of human commands with the structured, precise world of robot operations.

### The Power of Prompt Engineering and Tool Use

We don't train the LLM; we **prompt** it. We will use a technique often called **Tool Use** or **Function Calling**. We give the LLM a special "system prompt" that describes the robot's capabilities as a set of tools (or functions) it can use.

The system prompt will define:
1.  **The LLM's Role:** "You are a helpful robotics assistant. Your job is to convert user commands into a structured plan using the provided functions."
2.  **The Available Functions:** We will describe each low-level robot skill, its purpose, and its required parameters.
    -   `find_object(object_name: str)`: Locates an object and returns its coordinates.
    -   `move_to(x: float, y: float, z: float)`: Moves the arm to a specific coordinate.
    -   `set_gripper(state: str)`: Opens or closes the gripper ('open' or 'close').
3.  **The Desired Output Format:** We will instruct the LLM to always respond with a JSON object representing a sequence of these function calls.

### Example Interaction

Let's trace a command through the `CognitivePlannerNode`.

**1. User Command (from `/voice_command` topic):**
`"Please pick up the red cube and put it on the blue cylinder."`

**2. The Node Constructs the Full Prompt to the LLM:**
```
System Prompt:
You are a robotics assistant... You have access to the following tools:
- find_object(object_name: str)
- move_to(x: float, y: float, z: float)
- set_gripper(state: str ['open'|'close'])
Please respond with a JSON list of tool calls.

User Prompt:
"Please pick up the red cube and put it on the blue cylinder."
```

**3. The LLM Processes the Prompt and Returns a Structured Plan:**
The LLM uses its reasoning capabilities to decompose the command into a logical sequence of steps using *only the tools it was given*.

**LLM Response (JSON):**
```json
[
  {"tool": "find_object", "params": {"object_name": "red cube"}},
  {"tool": "move_to", "params": {"x": 0.5, "y": 0.2, "z": 0.1}}, // Placeholder coords from find
  {"tool": "set_gripper", "params": {"state": "close"}},
  {"tool": "find_object", "params": {"object_name": "blue cylinder"}},
  {"tool": "move_to", "params": {"x": -0.3, "y": 0.4, "z": 0.2}}, // Placeholder coords from find
  {"tool": "set_gripper", "params": {"state": "open"}}
]
```
*(Note: In a real implementation, the `move_to` calls would be dynamically filled in by the `find_object` results.)*

### The Executor

The `CognitivePlannerNode` receives this JSON. Its final job is to be an **executor**. It parses the JSON array and iterates through it, making the actual ROS 2 service calls for each step in the sequence. It calls the vision service to get coordinates, then the motion planning service to move the arm, and so on.

This architecture is incredibly powerful because if you want to add a new skill to your robot (e.g., `rotate_wrist`), you don't need to retrain a complex AI model. You simply add the new tool to the LLM's system prompt and implement the corresponding ROS 2 service. The LLM can then intelligently incorporate this new skill into its plans.
