# Panda Arm Pick and Place

![Panda Arm Pick and Place](path/to/your/image.png)

## Overview

The Panda Arm Pick and Place project is a robotic manipulation project implemented using ROS 2 Humble. The project focuses on achieving pick and place actions using the Panda robotic arm. The system is designed to include components for robot description, control, kinematics, motion planning, and computer vision.

## Project Structure

The project is organized into several ROS packages:

- **panda_bot_description:** Contains the robot description files, including the Unified Robot Description Format (URDF) files.

- **panda_bot_control:** Handles the configuration of controllers for the Panda robotic arm.

- **panda_bot_kinematics:** Computes both Forward Kinematics (FK) and Inverse Kinematics (IK) of the robot using the URDF file.

- **panda_bot_moveit2:** Responsible for motion planning and trajectory execution using MoveIt2.

- **panda_bot_vision:** Manages object detection and positioning of objects using computer vision.

## Dependencies

Make sure you have the necessary dependencies installed before running the project:

- ROS 2 Humble
- MoveIt2
- [Additional dependencies...]

## Build and Installation

```bash
# Clone the repository
git clone https://github.com/your_username/panda_arm_pick_and_place.git
cd panda_arm_pick_and_place

# Build the workspace
colcon build
```

## Running the Project

### 1. Launch Robot Description

```bash
source install/setup.bash
ros2 launch panda_bot_description robot_description.launch.py
```

### 2. Launch Controllers

```bash
source install/setup.bash
ros2 launch panda_bot_control controllers.launch.py
```

### 3. Kinematics Computation

```bash
source install/setup.bash
ros2 run panda_bot_kinematics kinematics_node
```

### 4. MoveIt2 Planning and Execution

```bash
source install/setup.bash
ros2 launch panda_bot_moveit2 moveit_planning_execution.launch.py
```

### 5. Computer Vision

```bash
source install/setup.bash
ros2 run panda_bot_vision vision_node
```

## Contributing

Contributions to the Panda Arm Pick and Place project are welcome. Feel free to submit issues, feature requests, or pull requests.

## License

This project is licensed under the [MIT License](LICENSE).
