#!/usr/bin/env python3
"""
Curiosity Yamcs ROS Bridge

Bidirectional bridge between Yamcs and ROS2 for the Curiosity rover.

Current functionality:
- Subscribes to joint state parameters from Yamcs and publishes them as ROS2 JointState messages

Planned functionality:
- Subscribe to ROS2 command topics and send commands to Yamcs
"""

import traceback

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from yamcs.client import YamcsClient


class YamcsJointStatePublisher(Node):
    """
    Bridge node for Curiosity rover telemetry and commands between Yamcs and ROS2.

    Currently handles joint state telemetry from Yamcs to ROS2.
    Will be extended to handle commands from ROS2 to Yamcs.
    """

    # Joint names in the order they appear in the XTCE definition
    # These names must match the joint names in the URDF (with _joint suffix)
    JOINT_NAMES = [
        "arm_01_joint", "arm_02_joint", "arm_03_joint", "arm_04_joint", "arm_tools_joint",
        "back_wheel_L_joint", "back_wheel_R_joint", "front_wheel_L_joint", "front_wheel_R_joint",
        "mast_02_joint", "mast_cameras_joint", "mast_p_joint",
        "middle_wheel_L_joint", "middle_wheel_R_joint",
        "suspension_arm_B2_L_joint", "suspension_arm_B2_R_joint",
        "suspension_arm_B_L_joint", "suspension_arm_B_R_joint",
        "suspension_arm_F_L_joint", "suspension_arm_F_R_joint",
        "suspension_steer_B_L_joint", "suspension_steer_B_R_joint",
        "suspension_steer_F_L_joint", "suspension_steer_F_R_joint"
    ]

    def __init__(self):
        super().__init__('yamcs_joint_state_publisher')

        # Declare parameters
        self.declare_parameter('yamcs_url', 'localhost:8090')
        self.declare_parameter('yamcs_instance', 'curiosity')
        self.declare_parameter('yamcs_processor', 'realtime')
        self.declare_parameter('joint_state_parameter', '/Spacecraft/joint_state')

        # Get parameters
        yamcs_url = self.get_parameter('yamcs_url').value
        yamcs_instance = self.get_parameter('yamcs_instance').value
        yamcs_processor = self.get_parameter('yamcs_processor').value
        self.joint_state_param = self.get_parameter('joint_state_parameter').value

        # Create ROS2 publisher
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Connect to Yamcs
        self.get_logger().info(f'Connecting to Yamcs at {yamcs_url}...')
        try:
            self.client = YamcsClient(yamcs_url)
            self.processor = self.client.get_processor(yamcs_instance, yamcs_processor)
            self.get_logger().info(f'Connected to Yamcs instance: {yamcs_instance}, processor: {yamcs_processor}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Yamcs: {e}')
            raise

        # Subscribe to Yamcs parameter updates
        self.get_logger().info(f'Subscribing to parameter: {self.joint_state_param}')
        self.subscription = self.processor.create_parameter_subscription(
            self.joint_state_param,
            on_data=self.yamcs_callback
        )

        self.get_logger().info('Yamcs ROS Bridge initialized')

    def yamcs_callback(self, data):
        """Callback for Yamcs parameter updates - publishes to ROS2."""
        for parameter in data.parameters:
            self.publish_joint_state(parameter)

    def publish_joint_state(self, parameter):
        """Convert Yamcs parameter to ROS2 JointState message and publish."""
        try:
            msg = JointState()

            # Set timestamp
            now = self.get_clock().now()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = ''

            # Set joint positions and names from Yamcs parameter
            msg.position = list(parameter.eng_value)
            msg.name = self.JOINT_NAMES

            # Publish the message
            self.publisher.publish(msg)
            self.get_logger().debug(f'Published {len(msg.name)} joint states')

        except Exception as e:
            self.get_logger().error(f'Error publishing joint state: {e}')
            self.get_logger().error(traceback.format_exc())

    def destroy_node(self):
        """Clean up resources."""
        if hasattr(self, 'subscription'):
            self.subscription.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = YamcsJointStatePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
