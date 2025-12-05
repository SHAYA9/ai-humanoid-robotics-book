---
sidebar_position: 6
---

# 7.6 Building a ROS 2 Package

So far, we've focused on writing and running individual nodes. However, the real power of ROS 2 comes from organizing your code into reusable, shareable units called **packages**. A package is a directory containing your nodes, launch files, custom message definitions, and a set of files that describe its contents and dependencies.

The standard tool for building ROS 2 packages is **`colcon`** (collective construction). It's a command-line tool that finds all the packages in your workspace, builds them in the correct order, and generates the necessary setup files to make them available to the ROS 2 environment.

### Package Structure

A minimal Python-based ROS 2 package has the following structure:

```
my_robot_pkg/
├── package.xml
├── setup.py
├── setup.cfg
└── my_robot_pkg/
    ├── __init__.py
    ├── talker_node.py
    └── listener_node.py
```

-   **`package.xml`**: This is the manifest file. It contains metadata about the package, such as its name, version, author, and, most importantly, its dependencies on other ROS 2 packages.

    ```xml
    <package format="3">
      <name>my_robot_pkg</name>
      <version>0.0.0</version>
      <description>Example package</description>
      <maintainer email="user@example.com">Your Name</maintainer>
      <license>Apache License 2.0</license>

      <depend>rclpy</depend>
      <depend>std_msgs</depend>

      <test_depend>ament_copyright</test_depend>
      <test_depend>ament_flake8</test_depend>
      <test_depend>ament_pep257</test_depend>
      <test_depend>python3-pytest</test_depend>

      <export>
        <build_type>ament_python</build_type>
      </export>
    </package>
    ```

-   **`setup.py`**: This is a standard Python `setuptools` script. It tells `colcon` how to build your package and, crucially, defines the **entry points** that make your nodes executable.

    ```python
    from setuptools import setup

    package_name = 'my_robot_pkg'

    setup(
        # ... other metadata
        packages=[package_name],
        data_files=[
            # ... data files like launch files
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        entry_points={
            'console_scripts': [
                'talker = my_robot_pkg.talker_node:main',
                'listener = my_robot_pkg.listener_node:main',
            ],
        },
    )
    ```
    The `entry_points` dictionary is key. It creates executable scripts (`talker` and `listener`) that call the `main` function in your Python node files.

### The `colcon` Build Process

A typical ROS 2 workflow involves a **workspace** directory that contains all your source packages.

```
ros2_ws/
├── src/
│   └── my_robot_pkg/
│       ├── ... (package files)
├── install/
├── log/
└── build/
```

1.  **Source Code:** You place your package source code inside the `src` directory.
2.  **Build:** From the root of the workspace (`ros2_ws`), you run `colcon build`. `colcon` will find all the packages in `src`, resolve their dependencies, and compile them, placing intermediate files in the `build` directory.
3.  **Install:** The final, built executables, libraries, and Python modules are placed in the `install` directory.
4.  **Source:** Before you can use your new package, you must **source** the workspace's setup file: `source install/setup.bash`. This command updates your environment, adding your package's executables to your `PATH` and making its modules available to Python.

After sourcing, you can run your nodes using `ros2 run`:
```bash
ros2 run my_robot_pkg talker
```

*<p align="center">PLACEHOLDER: `colcon` Build Process Flowchart</p>*
*<p align="center">A flowchart showing the process: source code in `src` -> `colcon build` -> artifacts in `build` and `install` -> `source install/setup.bash` -> executables available in the environment.</p>*

This structured approach is what makes ROS 2 a powerful framework for large-scale robotics development. It ensures that projects are modular, dependencies are managed, and code is easily shareable and reusable.
