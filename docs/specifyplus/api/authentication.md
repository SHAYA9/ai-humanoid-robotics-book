# API Authentication

In the SpecifyPlus framework, "authentication" and "authorization" are handled by the ROS 2 Security features, also known as SROS2. This system provides a comprehensive security model for the robotics application, ensuring that only trusted nodes can communicate on the network.

## Core Concepts

SROS2 is built on top of the security plugins of the underlying DDS implementation. It uses a Public Key Infrastructure (PKI) to manage identities and permissions.

-   **Identity:** Each ROS 2 node is given a cryptographic identity in the form of a signed certificate.
-   **Permissions:** A permissions file (XML) defines what each node is allowed to do (e.g., which topics it can publish or subscribe to, which services it can call).
-   **Authentication:** When two nodes try to communicate, they use their certificates to authenticate each other.
-   **Encryption:** All communication between authenticated nodes is encrypted.

## Enabling Security

Security is not enabled by default. To activate it, you must set an environment variable before launching your nodes:

```bash
export ROS_SECURITY_ENABLE=true
```

You also need to provide the location of the security artifacts (keys, certificates, permissions files), which are typically stored in a "keystore".

```bash
export ROS_SECURITY_KEYSTORE=/path/to/your/keystore
```

## Creating Security Artifacts

ROS 2 provides a command-line tool, `ros2 security`, to create the necessary keys and certificates.

**1. Create a Keystore:**

```bash
# This creates a directory with a Certificate Authority (CA)
ros2 security create_keystore my_keystore
```

**2. Create Keys and Certificates for a Node:**

```bash
cd my_keystore
# This creates the private key, certificate request, and signed certificate for a node named 'my_agent'
ros2 security create_key my_agent
```

## Defining Permissions

Permissions are defined in a `permissions.xml` file. This file grants or denies specific actions to each node based on its identity.

**Example `permissions.xml`:**

This example allows a `camera_node` to publish on the `/camera/image_raw` topic and a `vision_node` to subscribe to it. No other communication is permitted.

```xml
<?xml version="1.0" encoding="UTF-8"?>
<policy version="0.2.0"
  xmlns="http://www.ros.org/wiki/ROS2/Security/Policy">
  <enclaves>
    <enclave path="/my_secure_app">
      <profiles>
        <profile name="camera_node_profile">
          <publish>
            <topic name="/camera/image_raw"/>
          </publish>
        </profile>
        <profile name="vision_node_profile">
          <subscribe>
            <topic name="/camera/image_raw"/>
          </subscribe>
        </profile>
      </profiles>
      <nodes>
        <node name="camera_node" profile="camera_node_profile"/>
        <node name="vision_node" profile="vision_node_profile"/>
      </nodes>
    </enclave>
  </enclaves>
</policy>
```

This permissions file must be signed by the Certificate Authority to be valid.

By using SROS2, you can build a secure and robust SpecifyPlus system, ensuring data integrity and preventing unauthorized access to the robot's control and perception APIs.
