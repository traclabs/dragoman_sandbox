# dragoman_ground_example

Ground-side visualization example for the dragoman project, demonstrating how to visualize telemetry data from YAMCS using ROS2 and RViz.

## Overview

This package provides a complete ground station visualization setup that connects to a YAMCS server and displays robot telemetry in RViz. It demonstrates the integration between YAMCS (Yet Another Mission Control System) and ROS2 for spacecraft/robot ground operations.

## Package Contents

### Launch Files

- **`launch/curiosity_ground_vis.launch.py`** - Ground-side visualization for Curiosity rover with RViz
  - Launches RViz for visualization
  - Starts robot_state_publisher for TF transforms
  - Connects to YAMCS server via the dragoman_yamcs_ros_bridge
  - No simulation - purely for visualizing telemetry from YAMCS

### RViz Configuration

- **`rviz/curiosity.rviz`** - RViz configuration for Curiosity rover visualization

## Dependencies

- ROS2 packages: `robot_state_publisher`, `rviz2`, `xacro`
- `curiosity_description` - Curiosity rover URDF models
- [`dragoman_yamcs_ros_bridge`](../dragoman_yamcs_ros_bridge/README.md) - Bridge between YAMCS and ROS2

## Usage

### Prerequisites

1. Ensure YAMCS server is running with the Curiosity instance (see [`dragoman_yamcs_project`](../dragoman_yamcs_project/dragoman_yamcs_project/README.md))
2. Ensure telemetry data is being published to YAMCS (see [`dragoman_fsw_sim`](../dragoman_fsw_sim/README.md))

### Launch Ground Visualization

```bash
ros2 launch dragoman_ground_example curiosity_ground_vis.launch.py
```

### Launch Arguments

- `use_sim_time` (default: `False`) - Use simulation time
- `yamcs_url` (default: `localhost:8090`) - YAMCS server URL
- `yamcs_instance` (default: `curiosity`) - YAMCS instance name
- `yamcs_processor` (default: `realtime`) - YAMCS processor name

### Example with Custom YAMCS Server

```bash
ros2 launch dragoman_ground_example curiosity_ground_vis.launch.py \
    yamcs_url:=192.168.1.100:8090 \
    yamcs_instance:=my_mission
```

## How It Works

1. **robot_state_publisher** - Publishes TF transforms based on the Curiosity rover URDF model
2. **yamcs_ros_bridge** - Subscribes to YAMCS telemetry and publishes ROS2 joint states
3. **RViz2** - Visualizes the robot model with real-time joint positions from YAMCS

This creates a complete ground station visualization that mirrors the state of the robot/spacecraft as reported through YAMCS telemetry.

## Related Packages

- [`dragoman_fsw_sim`](../dragoman_fsw_sim/README.md) - Flight software simulators that generate telemetry
- [`dragoman_yamcs_project`](../dragoman_yamcs_project/dragoman_yamcs_project/README.md) - YAMCS server configuration
- [`dragoman_yamcs_ros_bridge`](../dragoman_yamcs_ros_bridge/README.md) - Bridge implementation
