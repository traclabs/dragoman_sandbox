# dragoman_sandbox

Main package for dragoman telemetry and command handling demonstrations.

## Overview

This package serves as the central coordination point for the dragoman project, which demonstrates integration between YAMCS (Yet Another Mission Control System) and ROS2 for spacecraft/robot telemetry and command handling.

## Package Organization

The dragoman project has been refactored into specialized packages for better organization:

- **[`dragoman_sample_xtce`](dragoman_sample_xtce/README.md)** - XTCE file generation scripts for telemetry definitions
- **[`dragoman_fsw_sim`](dragoman_fsw_sim/README.md)** - Flight software simulators and demo launch files
- **[`dragoman_yamcs_project`](dragoman_yamcs_project/dragoman_yamcs_project/README.md)** - YAMCS server configurations and setup
- **[`dragoman_yamcs_ros_bridge`](dragoman_yamcs_ros_bridge/README.md)** - Bridge between YAMCS and ROS2
- **[`dragoman_xtce2msg`](dragoman_xtce2msg/README.md)** - Tools for converting XTCE to ROS2 message definitions
- **[`dragoman_generated_msgs`](dragoman_generated_msgs/README.md)** - Auto-generated ROS2 message definitions from XTCE files
- **[`dragoman_ground_example`](dragoman_ground_example/README.md)** - Ground station visualization example

## Quick Start

For detailed usage instructions, see the individual package READMEs linked above. A typical workflow involves:

1. Generate XTCE files using [`dragoman_sample_xtce`](dragoman_sample_xtce/README.md)
2. Start YAMCS server using [`dragoman_yamcs_project`](dragoman_yamcs_project/dragoman_yamcs_project/README.md)
3. Run simulations/demos using [`dragoman_fsw_sim`](dragoman_fsw_sim/README.md)
4. (Optional) Visualize telemetry using [`dragoman_ground_example`](dragoman_ground_example/README.md)

## Dependencies

- ROS2 packages: `rclpy`, `sensor_msgs`, `trajectory_msgs`, etc.
- Related dragoman packages (see Package Organization above)
