# AI Humanoid Robotics Constitution
<!-- The Supreme Law governing all AI-Humanoid Robotics projects developed within this learning ecosystem -->

## Core Principles

### I. Safety-First (NON-NEGOTIABLE)
All development is subservient to human safety. Systems must be designed with immutable fail-safes, emergency stop protocols, and robust physical boundaries. A project is ethically invalid if its safety is ambiguous or cannot be verified in simulation before real-world testing.

### II. Simplicity & Modularity (YAGNI in Robotics)
Begin with the simplest viable implementation (e.g., a single sensor, one degree of freedom). Complex behaviors must emerge from the composition of simple, self-contained, and well-tested modules (libraries). No component is built for a hypothetical future need.

### III. Simulation-to-Reality (Sim2Real)
Every significant algorithm, controller, or behavior must be validated in a high-fidelity simulation environment (e.g., Gazebo, Isaac Sim) with defined success metrics before being deployed on physical hardware. The simulation model is a first-class citizen of the project.

### IV. Spec-Driven & Explainable Architecture
System goals, constraints, and high-level architecture must be explicitly documented (`spec.md`) before implementation. AI/ML components, especially in perception and control, must prioritize interpretability. "Black box" models require exceptional justification and additional safety scrutiny.

### V. Test-First Robotics (Red-Green-Refactor)
Adopt Test-Driven Development (TDD) for all software: Write tests for a module's intended behavior first. For physical behaviors, this means defining pass/fail criteria in simulation. The cycle is: 1) Specify desired behavior, 2) Create a failing test in simulation, 3) Implement until the test passes, 4) Refactor.

### VI. Ethical Embodiment
Robots are physical entities. Their design must explicitly consider environmental impact, resource use, end-of-life disposal, and societal effect. Projects must document an "Ethical Impact Assessment" addressing bias, privacy, security, and potential misuse.

## Technical & Development Standards

*   **Stack:** **ROS 2** is the standard middleware. **Python** for AI/ML and high-level logic; **C++** for performance-critical, real-time modules.
*   **Data:** All training data for machine learning models must be documented for source, potential bias, and labeling methodology.
*   **Versioning:** Use Semantic Versioning for all reusable libraries and modules. A change in a robot's physical capability or safety envelope is a **MAJOR** version change.
*   **Documentation:** Every module requires: 1) A clear purpose statement, 2) API documentation, 3) Known limitations and failure modes.

## Project Workflow
1.  **Specification (`/specify`):** Define the robot's mission, functional requirements, and success criteria in `spec.md`.
2.  **Planning (`/plan`):** Translate the spec into a technical architecture, selecting sensors, actuators, algorithms, and defining the Sim2Real validation strategy.
3.  **Tasking (`/tasks`):** Decompose the plan into atomic, testable implementation tasks executable by an individual or AI agent.
4.  **Validation:** Each task must satisfy its predefined simulation or unit tests before integration. The integrated system must pass a final simulation suite before any physical deployment.

## Governance
This constitution supersedes all ad-hoc practices and temporary shortcuts. All project proposals and Pull Requests must include a "Constitutional Compliance" statement.
*   **Amendments:** Changes to these core principles require documented rationale, team review, and an update to the "Last Amended" date.
*   **Arbitration:** If a development decision has unclear constitutional compliance, work stops, and the issue is escalated to a team review with reference to this document and the project's `spec.md`.

**Version**: 1.0.0 | **Ratified**: 2025-12-11 | **Last Amended**: 2025-12-11