#!/usr/bin/env python3
"""
Gateway Joint State Publisher

Subscribes to Gateway telemetry messages and converts them to ROS2 JointState messages
for the Gateway big arm.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dragoman_generated_msgs.msg import GatewayTelemetryPacket


class GatewayJointStatePublisher(Node):
    """
    Joint state publisher for Gateway big arm.

    Subscribes to GatewayTelemetryPacket messages and converts them to ROS2 JointState messages.
    """

    # Joint names in the order they appear in the telemetry packet
    # These names must match the joint names in the URDF
    # Big arm has 7 revolute joints: big_arm_joint_2 through big_arm_joint_8
    JOINT_NAMES = [
        "big_arm_joint_2",
        "big_arm_joint_3",
        "big_arm_joint_4",
        "big_arm_joint_5",
        "big_arm_joint_6",
        "big_arm_joint_7",
        "big_arm_joint_8",
    ]

    def __init__(self):
        super().__init__('gateway_joint_state_publisher')

        # Declare parameters
        self.declare_parameter('input_topic', '/yamcs/gateway')

        # Get parameters
        input_topic = self.get_parameter('input_topic').value

        # Create ROS2 publisher
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Subscribe to Gateway telemetry topic
        self.subscription = self.create_subscription(
            GatewayTelemetryPacket,
            input_topic,
            self.telemetry_callback,
            10
        )

        self.get_logger().info(f'Gateway Joint State Publisher initialized')
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing to: /joint_states')

    def telemetry_callback(self, msg):
        """Convert GatewayTelemetryPacket to JointState message and publish."""
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
    node = GatewayJointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
