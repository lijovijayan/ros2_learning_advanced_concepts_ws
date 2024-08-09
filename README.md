# Learning Advanced Concepts of ROS2

Welcome to the **Learning Advanced Concepts of ROS2** repository! This repository contains the source code and learning materials as I explore and master advanced concepts in ROS2.

## Overview

In this repository, you'll find code examples, notes, and documentation that cover advanced ROS2 topics, including but not limited to:

- **Actions:** Understanding and implementing ROS2 actions, which allow nodes to perform long-running tasks that can be preempted or canceled.
- **Lifecycle Events:** Learning about ROS2 node lifecycle management, which includes different states (e.g., unconfigured, inactive, active) and transitions between these states.

## Repository Structure

The repository is structured as follows:

```
learning-ros2-advanced-concepts/
├── actions/
│   ├── action_example_1/
│   └── action_example_2/
├── lifecycle_events/
│   ├── lifecycle_event_example_1/
│   └── lifecycle_event_example_2/
├── README.md
└── ...
```

- **actions/**: Contains examples and source code related to ROS2 actions.
- **lifecycle_events/**: Contains examples and source code related to ROS2 node lifecycle events.

## How to Use

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/learning-ros2-advanced-concepts.git
   cd learning-ros2-advanced-concepts
   ```

2. Navigate to the topic you want to explore, e.g., `actions` or `lifecycle_events`.

3. Follow the README or documentation within each folder to understand the concepts and run the code examples.

## Prerequisites

- **ROS2**: Ensure you have ROS2 installed on your system. This repository is compatible with [ROS2 Foxy](https://docs.ros.org/en/foxy/) or later.
- **Colcon**: Make sure you have `colcon` installed to build and run the ROS2 workspace.

## Building the Workspace

To build the workspace, navigate to the root directory of the repository and run:

```bash
colcon build
```

## Running Examples

After building the workspace, source your setup files and run the examples:

```bash
source install/setup.bash
ros2 run <package_name> <node_name>
```

Refer to the specific example directories for detailed instructions.

## Contributing

Contributions are welcome! If you have suggestions for improving the examples or adding new topics, feel free to open an issue or submit a pull request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
