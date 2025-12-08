# dragoman_ground_example

Ground station visualization for YAMCS telemetry using ROS2 and RViz.

## Overview

Provides ground-side visualization that connects to a YAMCS server and displays robot telemetry in RViz, demonstrating YAMCS-ROS2 integration for spacecraft/robot ground operations.

## Package Contents

- **`launch/curiosity_ground.launch.py`** - Curiosity rover ground visualization
- **`launch/imetro_ground.launch.py`** - iMetro CLR robot ground visualization
- **`rviz/`** - RViz configurations for each robot
- **`scripts/`** - Custom joint state publishers that convert YAMCS telemetry packets to ROS2 joint states

## Usage

**Prerequisites:** YAMCS server running with telemetry data (see [`dragoman_yamcs_project`](../dragoman_yamcs_project/dragoman_yamcs_project/README.md) and [`dragoman_fsw_sim`](../dragoman_fsw_sim/README.md))

```bash
# Curiosity rover
ros2 launch dragoman_ground_example curiosity_ground.launch.py

# iMetro CLR robot
ros2 launch dragoman_ground_example imetro_ground.launch.py

```

## Architecture

**Ground Station Visualization Flow:**
1. **robot_state_publisher** - Publishes TF transforms from URDF
2. **yamcs_ros_bridge** - Subscribes to YAMCS telemetry, publishes ROS2 messages
3. **joint_state_publisher** (iMetro only) - Converts telemetry packets to joint states
4. **RViz2** - Visualizes robot with real-time telemetry from YAMCS
