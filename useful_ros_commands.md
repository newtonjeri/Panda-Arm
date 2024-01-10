
```markdown
# ROS 2 Command Cheat Sheet

## Workspace Management

### Creating a Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
```

### Source the Workspace
```bash
source ~/ros2_ws/install/setup.bash
```

## Package Management

### Creating a Package
```bash
ros2 pkg create --build-type ament_python my_package
```

### Building a Package
```bash
colcon build --packages-select my_package
```

### Running a Package
```bash
ros2 run my_package my_node
```

## Node and Topic Information

### List Available Nodes
```bash
ros2 node list
```

### Node Information
```bash
ros2 node info /my_node
```

### List Topics
```bash
ros2 topic list
```

### Topic Information
```bash
ros2 topic info /my_topic
```

## Launch System

### Creating a Launch File
```bash
mkdir -p ~/ros2_ws/src/my_package/launch
touch ~/ros2_ws/src/my_package/launch/my_launch_file.py
```

### Running a Launch File
```bash
ros2 launch my_package my_launch_file.py
```

## Tool Commands

### RViz Visualization
```bash
ros2 run rviz2 rviz2
```

### Convert xacro to urdf
```bash
ros2 run xacro xacro -o <urdf_file> <xacro_file>
```

### RQT Graph
```bash
ros2 run rqt_graph rqt_graph
```

### ROS 2 Command-line Tools
```bash
ros2 doctor             # Check system for common issues
ros2 bag record         # Record ROS 2 topics to a bag file
ros2 param get /node   # Get parameter value
ros2 param set /node   # Set parameter value
```

## System Status

### System Status
```bash
ros2 doctor
```

### System Resource Usage
```bash
ros2 node hz /my_node
```

## Miscellaneous

### ROS 2 Documentation
[ROS 2 Documentation](https://docs.ros.org/en/foxy/)

### ROS 2 Source Installation
[ROS 2 Source Installation](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/)

```