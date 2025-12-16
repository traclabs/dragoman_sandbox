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
  instance: "dragoman"
  processor: "realtime"

bridges:
  - ros_message_type: "dragoman_generated_msgs/msg/IMetroTelemetryPacket"
    ros_topic: "/yamcs/imetro"
    yamcs_packet_name: "IMetro/IMetroTelemetryPacket"
```
