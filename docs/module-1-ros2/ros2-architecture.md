---
sidebar_position: 2
title: ROS 2 Architecture
---

# ROS 2 Architecture

The architecture of ROS 2 is designed to be a flexible, scalable, and robust platform for robotics. It is a significant evolution from ROS 1, built to support a wider range of applications, from small, single-board computers to large, multi-robot systems.

## The ROS 2 Graph

The fundamental concept in ROS 2 is the **ROS 2 graph**. The graph is a network of ROS 2 nodes that communicate with each other. This distributed, peer-to-peer architecture is one of the key strengths of ROS 2.

### Key Components of the Graph

-   **Nodes:** A node is the main computational unit in ROS 2. Each node is a process that performs a specific task, such as controlling a motor, reading sensor data, or planning a path.
-   **Topics:** Topics are the primary mechanism for asynchronous, publish/subscribe communication. A node can publish messages to a topic, and any number of nodes can subscribe to that topic to receive the messages.
-   **Services:** Services are used for synchronous, request/response communication. A node can provide a service, and another node can call that service and wait for a response.
-   **Actions:** Actions are used for long-running, asynchronous tasks that provide feedback. A node can provide an action, and another node can send a goal to that action and receive feedback and a final result.

## The Middleware Layer: DDS

Under the hood, ROS 2 is built on top of the **Data Distribution Service (DDS)**, an industry-standard middleware for real-time systems. DDS provides a robust and efficient communication layer, with features like:

-   **Discovery:** Nodes can automatically discover each other on the network.
-   **Serialization:** Messages are automatically serialized and deserialized.
-   **Quality of Service (QoS):** ROS 2 provides a rich set of QoS policies that allow you to fine-tune the communication for different use cases, such as reliability, durability, and latency.

## Security

ROS 2 has built-in security features that provide a secure communication channel between nodes. This is a critical feature for production robotics systems. The security features include:

-   **Authentication:** Nodes can verify the identity of other nodes.
-   **Encryption:** Messages can be encrypted to prevent eavesdropping.
-   **Access Control:** You can define fine-grained access control rules to restrict which nodes can publish or subscribe to which topics.

In the next section, we will take a closer look at the building blocks of the ROS 2 graph: [Nodes, Topics, and Services](./nodes-topics-services.md).
