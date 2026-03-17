#!/usr/bin/env python3
"""
Lunar Exploration Overlay Pie Chart Publisher

Subscribes to Lunar Exploration telemetry messages and publishes battery power
and solar panel data as RViz overlay pie charts
"""

import rclpy
from rclpy.node import Node
from dragoman_sample_msgs.msg import LunarExplorationTelemetryPacket
from std_msgs.msg import Float32

class LunarExplorationOverlayPieChartPublisher(Node):
    """
    Overlay pie chart publisher for Lunar Exploration demo.

    Subscribes to LunarExplorationTelemetryPacket messages and publishes
    battery power and solar panel data as RViz overlay pie charts.
    """

    def __init__(self):
        super().__init__('lunar_exploration_overlay_pie_chart_publisher')

        # Declare parameters
        self.declare_parameter('input_topic', '/yamcs/lunar_exploration_space')
        self.declare_parameter('output_topic', '/overlay_pie_chart/battery_power')

        # Get parameters
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # Create ROS2 publishers
        self.battery_publisher = self.create_publisher(Float32, output_topic, 10)
        self.solar_left_publisher = self.create_publisher(Float32, '/overlay_pie_chart/solar_panel_left', 10)
        self.solar_right_publisher = self.create_publisher(Float32, '/overlay_pie_chart/solar_panel_right', 10)
        self.solar_rear_publisher = self.create_publisher(Float32, '/overlay_pie_chart/solar_panel_rear', 10)

        # Subscribe to LunarExploration telemetry topic
        self.subscription = self.create_subscription(
            LunarExplorationTelemetryPacket,
            input_topic,
            self.telemetry_callback,
            10
        )

        self.get_logger().debug('LunarExploration Overlay Pie Chart Publisher initialized')
        self.get_logger().debug(f'Subscribing to: {input_topic}')
        self.get_logger().debug(f'Publishing to: {output_topic}')
        self.get_logger().debug('Publishing to: /overlay_pie_chart/solar_panel_left')
        self.get_logger().debug('Publishing to: /overlay_pie_chart/solar_panel_right')
        self.get_logger().debug('Publishing to: /overlay_pie_chart/solar_panel_rear')

    def telemetry_callback(self, msg):
        """Convert battery and solar panel data to Float32 messages and publish."""
        # Publish battery percentage
        battery_msg = Float32()
        battery_msg.data = float(msg.battery.percentage)
        self.battery_publisher.publish(battery_msg)
        self.get_logger().debug(f'Published battery percentage: {battery_msg.data:.1f}%')

        # Publish solar panel data
        solar_left_msg = Float32()
        solar_left_msg.data = float(msg.solar_panels.left)
        self.solar_left_publisher.publish(solar_left_msg)
        self.get_logger().debug(f'Published solar panel left: {solar_left_msg.data:.1f}%')

        solar_right_msg = Float32()
        solar_right_msg.data = float(msg.solar_panels.right)
        self.solar_right_publisher.publish(solar_right_msg)
        self.get_logger().debug(f'Published solar panel right: {solar_right_msg.data:.1f}%')

        solar_rear_msg = Float32()
        solar_rear_msg.data = float(msg.solar_panels.rear)
        self.solar_rear_publisher.publish(solar_rear_msg)
        self.get_logger().debug(f'Published solar panel rear: {solar_rear_msg.data:.1f}%')


def main(args=None):
    rclpy.init(args=args)
    node = LunarExplorationOverlayPieChartPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
