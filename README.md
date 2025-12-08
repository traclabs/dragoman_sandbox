# dragoman_sandbox

Telemetry and command handling demonstrations integrating YAMCS with ROS2 for spacecraft/robot ground operations.

## Overview

This project demonstrates the integration between YAMCS (Yet Another Mission Control System) and ROS2, providing:
- XTCE-based telemetry definitions
- Flight software simulators
- YAMCS-ROS2 bridge for telemetry/command handling
- Ground station visualization with RViz

## Package Organization

- **[`dragoman_sample_xtce`](dragoman_sample_xtce/README.md)** - XTCE file generation scripts
- **[`dragoman_fsw_sim`](dragoman_fsw_sim/README.md)** - Flight software simulators (spacecraft-side)
- **[`dragoman_yamcs_project`](dragoman_yamcs_project/dragoman_yamcs_project/README.md)** - YAMCS server configuration
- **[`dragoman_yamcs_ros_bridge`](dragoman_yamcs_ros_bridge/README.md)** - YAMCS-ROS2 bridge
- **[`dragoman_xtce2msg`](dragoman_xtce2msg/README.md)** - XTCE to ROS2 message converter
- **[`dragoman_generated_msgs`](dragoman_generated_msgs/README.md)** - Auto-generated ROS2 messages from XTCE
- **[`dragoman_ground_example`](dragoman_ground_example/README.md)** - Ground station visualization (ground-side)

## Quick Start

### Preparation (one-time setup)

1. Generate XTCE files
   ```bash
   ros2 run dragoman_sample_xtce generate_xtces.sh
   colcon build --packages-select dragoman_sample_xtce
   ```

2. Generate ROS messages from XTCE
   ```bash
   ros2 run dragoman_xtce2msg generate_msgs.sh
   ```

3. Build generated messages
   ```bash
   colcon build --packages-select dragoman_generated_msgs
   ```

## Quick Start Demo

### 1. Start YAMCS Server
```bash
ros2 run dragoman_yamcs_project dragoman_yamcs
```
Open web interface at `http://localhost:8090/`

### 2. Launch Spacecraft-Side Simulation
Choose one:
```bash
# iMetro dummy demo (hardcoded telemetry)
ros2 launch dragoman_fsw_sim imetro_dummy_demo.launch.py

# iMetro robot demo (ros2_control fake hardware)
ros2 launch dragoman_fsw_sim imetro_robot_simple_demo.launch.py

# Curiosity rover demo (Gazebo simulation)
ros2 launch dragoman_fsw_sim curiosity_simulation.launch.py
```

### 3. Launch Ground-Side Visualization (Optional)
```bash
# For iMetro demos
ros2 launch dragoman_ground_example imetro_ground.launch.py

# For Curiosity demo
ros2 launch dragoman_ground_example curiosity_ground.launch.py
```

**Note:** Spacecraft-side runs in `ROS_DOMAIN_ID=100`, ground-side in `ROS_DOMAIN_ID=0` to isolate communications.

## Architecture

**Ground Station Visualization Flow:**
1. **robot_state_publisher** - Publishes TF transforms from URDF
2. **yamcs_ros_bridge** - Subscribes to YAMCS telemetry, publishes ROS2 messages
3. **joint_state_publisher** (iMetro only) - Converts telemetry packets to joint states
4. **RViz2** - Visualizes robot with real-time telemetry from YAMCS

See individual package READMEs for detailed information.
