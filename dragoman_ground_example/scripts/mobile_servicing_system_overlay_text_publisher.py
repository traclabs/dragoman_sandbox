#!/usr/bin/env python3
"""
Mobile Servicing System Overlay Text Publisher

Subscribes to Mobile Servicing System telemetry messages and publishes motion
command status for each subsystem as RViz overlay text
"""

import rclpy
from rclpy.node import Node
from dragoman_sample_msgs.msg import MobileServicingSystemTelemetryPacket
from rviz_2d_overlay_msgs.msg import OverlayText

class MobileServicingSystemOverlayTextPublisher(Node):
    """
    Overlay text publisher for Mobile Servicing System demo.

    Subscribes to MobileServicingSystemTelemetryPacket messages and publishes
    motion command status for each subsystem as RViz overlay text.
    """

    def __init__(self):
        super().__init__('mobile_servicing_system_overlay_text_publisher')

        # Declare parameters
        self.declare_parameter('input_topic', '/yamcs/mobile_servicing_system')
        self.declare_parameter('output_topic_prefix', '/overlay_text/mss_')
        self.declare_parameter('width', 300)
        self.declare_parameter('height', 30)
        self.declare_parameter('left', 10)
        self.declare_parameter('top', 10)
        self.declare_parameter('line_spacing', 25)
        self.declare_parameter('text_size', 10)
        self.declare_parameter('font', 'Roboto Mono')

        # Get parameters
        input_topic = self.get_parameter('input_topic').value
        output_topic_prefix = self.get_parameter('output_topic_prefix').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.left = self.get_parameter('left').value
        self.top = self.get_parameter('top').value
        self.line_spacing = self.get_parameter('line_spacing').value
        self.text_size = self.get_parameter('text_size').value
        self.font = self.get_parameter('font').value

        # Define subsystems with their labels and vertical positions
        self.subsystems = [
            ('mbs', 'MBS', 0),
            ('canadarm2', 'SSRMS', 1),
            ('dextre_body', 'SPDM BODY', 2),
            ('dextre_arm_1', 'SPDM ARM1', 3),
            ('dextre_arm_2', 'SPDM ARM2', 4),
            ('sarj', 'SARJ', 5),
            ('port_bga', 'PORT BGA', 6),
            ('starboard_bga', 'STAR BGA', 7),
        ]

        # Create publishers for each subsystem
        self.subsystem_publishers = {}
        for field_name, label, position in self.subsystems:
            topic = f'{output_topic_prefix}{field_name}'
            self.subsystem_publishers[field_name] = self.create_publisher(OverlayText, topic, 10)

        # Subscribe to MobileServicingSystem telemetry topic
        self.subscription = self.create_subscription(
            MobileServicingSystemTelemetryPacket,
            input_topic,
            self.telemetry_callback,
            10
        )

        self.get_logger().debug('MobileServicingSystem Overlay Text Publisher initialized')
        self.get_logger().debug(f'Subscribing to: {input_topic}')
        for field_name, label, position in self.subsystems:
            topic = f'{output_topic_prefix}{field_name}'
            self.get_logger().debug(f'Publishing {label} to: {topic}')

    def get_status_color(self, status_value):
        """Return RGBA color tuple based on status string."""
        status = status_value.upper()

        if status == "IDLE":
            # Gray for idle
            return (0.7, 0.7, 0.7, 0.7)
        elif status == "IN_PROGRESS":
            # Yellow/orange for in progress
            return (1.0, 0.8, 0.0, 0.7)
        elif status == "DONE":
            # Green for done
            return (0.0, 1.0, 0.0, 0.7)
        else:  # UNKNOWN
            # Red for unknown/error
            return (1.0, 0.0, 0.0, 0.7)

    def create_overlay_message(self, label, status_value, vertical_position):
        """Create an overlay text message for a subsystem status."""
        overlay_msg = OverlayText()

        # Set action to ADD
        overlay_msg.action = OverlayText.ADD

        # Set dimensions and position
        overlay_msg.width = int(self.width)
        overlay_msg.height = int(self.height)
        overlay_msg.horizontal_distance = int(self.left)
        overlay_msg.vertical_distance = int(self.top + (vertical_position * self.line_spacing))
        overlay_msg.horizontal_alignment = OverlayText.LEFT
        overlay_msg.vertical_alignment = OverlayText.TOP

        # Set background color (transparent)
        overlay_msg.bg_color.r = 0.0
        overlay_msg.bg_color.g = 0.0
        overlay_msg.bg_color.b = 0.0
        overlay_msg.bg_color.a = 0.0

        # Set text properties
        overlay_msg.line_width = int(2)
        overlay_msg.text_size = float(self.text_size)
        overlay_msg.font = self.font

        # Set text color based on status
        color = self.get_status_color(status_value)
        overlay_msg.fg_color.r = color[0]
        overlay_msg.fg_color.g = color[1]
        overlay_msg.fg_color.b = color[2]
        overlay_msg.fg_color.a = color[3]

        # Set text with label and status
        overlay_msg.text = f"{label}: {status_value}"

        return overlay_msg

    def telemetry_callback(self, msg):
        """Convert motion command status to overlay text messages and publish."""
        motion_status = msg.motion_command_status

        # Publish overlay text for each subsystem
        for field_name, label, position in self.subsystems:
            # Get status value from motion_command_status
            status_value = getattr(motion_status, field_name)

            # Create and publish overlay message
            overlay_msg = self.create_overlay_message(label, status_value, position)
            self.subsystem_publishers[field_name].publish(overlay_msg)

            self.get_logger().debug(f'Published {label}: {status_value}')


def main(args=None):
    rclpy.init(args=args)
    node = MobileServicingSystemOverlayTextPublisher()

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
