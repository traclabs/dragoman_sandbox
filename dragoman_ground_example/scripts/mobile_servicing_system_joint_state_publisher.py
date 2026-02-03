#!/usr/bin/env python3
"""
Mobile Servicing System Joint State Publisher

Subscribes to MobileServicingSystem telemetry messages and converts them to ROS2 JointState messages
for the MobileServicingSystem
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dragoman_sample_msgs.msg import MobileServicingSystemTelemetryPacket


class MobileServicingSystemJointStatePublisher(Node):
    """
    Joint state publisher for MobileServicingSystem big arm.

    Subscribes to MobileServicingSystemTelemetryPacket messages and converts them to ROS2 JointState messages.
    """

    # Joint names in the order they appear in the telemetry packet
    # These names must match the joint names in the URDF
    JOINT_NAMES = [
    "joint_canadarm2_1", "joint_canadarm2_2", "joint_canadarm2_3", "joint_canadarm2_4", "joint_canadarm2_5", "joint_canadarm2_6", "joint_canadarm2_7",
    "joint_dextre_arm_1_elbow_pitch", "joint_dextre_arm_1_shoulder_pitch", "joint_dextre_arm_1_shoulder_roll",
    "joint_dextre_arm_1_shoulder_yaw", "joint_dextre_arm_1_wrist_pitch_yaw", "joint_dextre_arm_1_wrist_roll",
    "joint_dextre_arm_2_elbow_pitch", "joint_dextre_arm_2_shoulder_pitch", "joint_dextre_arm_2_shoulder_roll",
    "joint_dextre_arm_2_shoulder_yaw", "joint_dextre_arm_2_wrist_pitch_yaw", "joint_dextre_arm_2_wrist_roll",
    "joint_dextre_body",
    "joint_mbs",
    "joint_port_bga_1", "joint_port_bga_2", "joint_port_bga_3", "joint_port_bga_4",
    "joint_port_sarj",
    "joint_starboard_bga_1", "joint_starboard_bga_2", "joint_starboard_bga_3", "joint_starboard_bga_4",
    "joint_starboard_sarj"
    ]

    def __init__(self):
        super().__init__('mobile_servicing_system_joint_state_publisher')

        # Declare parameters
        self.declare_parameter('input_topic', '/yamcs/mobile_servicing_system')

        # Get parameters
        input_topic = self.get_parameter('input_topic').value

        # Create ROS2 publisher
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Subscribe to Mobile Servicing System telemetry topic
        self.subscription = self.create_subscription(
            MobileServicingSystemTelemetryPacket,
            input_topic,
            self.telemetry_callback,
            10
        )

        self.get_logger().info(f'Mobile Servicing System Joint State Publisher initialized')
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing to: /joint_states')

    def telemetry_callback(self, msg):
        """Convert MobileServicingSystemTelemetryPacket to JointState message and publish."""
        joint_state_msg = JointState()

        # Set timestamp
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = ''

        # Set joint positions and names from telemetry packet
        joint_state_msg.position = list(msg.joint_state)
        joint_state_msg.name = self.JOINT_NAMES

        # Publish the message
        self.publisher.publish(joint_state_msg)
        self.get_logger().debug(f'Published {len(joint_state_msg.name)} joint states')


def main(args=None):
    rclpy.init(args=args)
    node = MobileServicingSystemJointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
