#!/usr/bin/env python3
"""
iMetro Joint State Publisher

Subscribes to iMetro telemetry messages and converts them to ROS2 JointState messages
for the iMetro/CLR robot.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dragoman_generated_msgs.msg import IMetroTelemetryPacket


class IMetroJointStatePublisher(Node):
    """
    Joint state publisher for iMetro/CLR robot.

    Subscribes to IMetroTelemetryPacket messages and converts them to ROS2 JointState messages.
    """

    # Joint names in the order they appear in the telemetry packet
    # These names must match the joint names in the URDF
    JOINT_NAMES = [
        "ewellix_lift_lower_to_higher",
        "vention_rail_base_to_carriage",
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
        "robotiq_hande_left_finger_joint",
    ]

    def __init__(self):
        super().__init__('imetro_joint_state_publisher')

        # Declare parameters
        self.declare_parameter('input_topic', '/yamcs/imetro')

        # Get parameters
        input_topic = self.get_parameter('input_topic').value

        # Create ROS2 publisher
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Subscribe to iMetro telemetry topic
        self.subscription = self.create_subscription(
            IMetroTelemetryPacket,
            input_topic,
            self.telemetry_callback,
            10
        )

        self.get_logger().info(f'iMetro Joint State Publisher initialized')
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing to: /joint_states')

    def telemetry_callback(self, msg):
        """Convert IMetroTelemetryPacket to JointState message and publish."""
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
    node = IMetroJointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
