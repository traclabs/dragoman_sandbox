# dragoman_yamcs_ros_bridge

YAMCS to ROS2 bridge package for bidirectional telemetry and command handling.

## Overview

This package provides Python-based bridges that connect YAMCS (Yet Another Mission Control System) telemetry and command systems with ROS2. It enables seamless integration between spacecraft/robot telemetry systems and ROS2-based ground control or simulation environments.

## Features

- **Generic YAMCS-ROS Bridge**: Configurable bridge supporting multiple telemetry types via YAML configuration
- **Curiosity-specific Bridge**: Specialized bridge for Curiosity rover joint state telemetry
- Flexible field mapping between YAMCS parameters and ROS2 messages
- Support for all XTCE data types (integers, floats, strings, arrays, aggregates, etc.)
- Dynamic message type loading

## Package Contents

### Scripts

- `yamcs_ros_bridge.py` - Generic configurable bridge for YAMCS to ROS2
- `curiosity_yamcs_ros_bridge.py` - Specialized bridge for Curiosity rover telemetry

### Configuration

- `config/yamcs_bridge_params.yaml` - Example configuration file showing bridge setup for multiple telemetry types

## Usage

### Generic Bridge

```bash
ros2 run dragoman_yamcs_ros_bridge yamcs_ros_bridge.py --config /path/to/config.yaml
```

Or with YAMCS URL override:

```bash
ros2 run dragoman_yamcs_ros_bridge yamcs_ros_bridge.py --config /path/to/config.yaml --yamcs-url localhost:8090
```

### Curiosity Bridge

```bash
ros2 run dragoman_yamcs_ros_bridge curiosity_yamcs_ros_bridge.py
```

With parameters:

```bash
ros2 run dragoman_yamcs_ros_bridge curiosity_yamcs_ros_bridge.py --ros-args \
  -p yamcs_url:=localhost:8090 \
  -p yamcs_instance:=curiosity \
  -p yamcs_processor:=realtime
```

## Configuration Format

The generic bridge uses YAML configuration files with the following structure:

```yaml
yamcs:
  url: "localhost:8090"
  instance: "curiosity"
  processor: "realtime"

bridges:
  - name: "telemetry_bridge"
    yamcs_parameter:
      - "/Spacecraft/Parameter1"
      - "/Spacecraft/Parameter2"
    ros_topic: "/telemetry"
    ros_message_type: "dragoman_generated_msgs/msg/TelemetryPacket"
    field_mapping:
      Parameter1: field1
      Parameter2: field2
```

## Dependencies

- `rclpy` - ROS2 Python client library
- `sensor_msgs` - Standard ROS2 sensor messages
- `dragoman_generated_msgs` - Custom message definitions for Dragoman project
- `yamcs-client` - Python client for YAMCS

## Integration

This package is part of the Dragoman project and works alongside:
- `dragoman_yamcs_project` - YAMCS server configurations and XTCE definitions
- `dragoman_generated_msgs` - Custom ROS2 message definitions
- `dragoman_fsw_sim` - Flight software simulators

## Notes

- The bridges subscribe to YAMCS parameters (not containers) and publish to ROS2 topics
- Field mapping supports both direct parameter mapping and aggregate member access
- All bridges handle proper type conversions between YAMCS and ROS2 data types
