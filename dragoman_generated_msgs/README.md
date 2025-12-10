# dragoman_generated_msgs

Auto-generated ROS 2 message definitions from XTCE telemetry specifications. These messages are used for telemetry communication between flight software simulators, YAMCS, and ROS 2 nodes.

**Important**: Do not manually edit message files. Regenerate them using [`dragoman_xtce2msg`](../dragoman_xtce2msg/README.md).

## Regenerating Messages

```bash
ros2 run dragoman_xtce2msg generate_msgs.sh
colcon build --packages-select dragoman_generated_msgs
source install/setup.bash
```

## Usage

```python
from dragoman_generated_msgs.msg import CuriosityTelemetryPacket

msg = CuriosityTelemetryPacket()
# Use the message...
```
