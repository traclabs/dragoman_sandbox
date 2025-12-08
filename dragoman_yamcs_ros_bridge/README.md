# dragoman_yamcs_ros_bridge

YAMCS to ROS2 bridge package for bidirectional telemetry and command handling.

## Usage

```bash
ros2 run dragoman_yamcs_ros_bridge yamcs_ros_bridge.py --config $(ros2 pkg prefix  --share dragoman_yamcs_ros_bridge)/config/yamcs_bridge_params.yaml
```

## Configuration

Example YAML config:
```yaml
yamcs:
  url: "localhost:8090"
  instance: "curiosity"
  processor: "realtime"

bridges:
  - name: "telemetry_bridge"
    yamcs_parameter:
      - "/Spacecraft/Parameter1"
    ros_topic: "/telemetry"
    ros_message_type: "dragoman_generated_msgs/msg/TelemetryPacket"
    field_mapping:
      Parameter1: field1
```

## Dependencies

- `rclpy`, `sensor_msgs`, `yamcs-client`, `dragoman_generated_msgs`
