#!/usr/bin/env python3
"""
YAMCS-ROS Bridge

Bridges YAMCS telemetry parameters to ROS2 messages based on YAML configuration.
Supports multiple parameter-to-message mappings with flexible field mapping.

Usage:
    python3 yamcs_ros_bridge.py --config bridge_config.yaml
    python3 yamcs_ros_bridge.py --config bridge_config.yaml --yamcs-url localhost:8090

Configuration file format (YAML):
    yamcs:
      url: "localhost:8090"
      instance: "curiosity"
      processor: "realtime"

    bridges:
      - name: "curiosity_telemetry"
        yamcs_parameter: "/Spacecraft/joint_state"
        ros_topic: "/curiosity/telemetry"
        ros_message_type: "dragoman_msgs/Curiosity"
        field_mapping:
          joint_state: joint_state

Author: Generated for Dragoman project
"""

import argparse
import sys
import traceback
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from yamcs.client import YamcsClient
import importlib


class YamcsRosBridge(Node):
    """
    ROS2 node that bridges YAMCS telemetry to ROS2 messages.

    Reads configuration from YAML file to determine which YAMCS parameters
    to subscribe to and which ROS2 messages to publish.
    """

    def __init__(self, config_file, yamcs_url_override=None):
        """
        Initialize the YAMCS-ROS bridge.

        Args:
            config_file: Path to YAML configuration file
            yamcs_url_override: Optional override for YAMCS URL from config
        """
        super().__init__('yamcs_ros_bridge')

        self.get_logger().info(f'Loading configuration from: {config_file}')

        # Load and validate configuration
        try:
            self.config = self.load_config(config_file)
        except Exception as e:
            self.get_logger().error(f'Failed to load configuration: {e}')
            raise

        # Override YAMCS URL if provided
        if yamcs_url_override:
            self.config['yamcs']['url'] = yamcs_url_override
            self.get_logger().info(f'Overriding YAMCS URL: {yamcs_url_override}')

        # Connect to YAMCS
        try:
            self.connect_yamcs()
        except Exception as e:
            self.get_logger().error(f'Failed to connect to YAMCS: {e}')
            raise

        # Setup all configured bridges
        try:
            self.setup_bridges()
        except Exception as e:
            self.get_logger().error(f'Failed to setup bridges: {e}')
            raise

        self.get_logger().info('YAMCS-ROS Bridge initialized successfully')

    def load_config(self, config_file):
        """
        Load and validate YAML configuration file.

        Args:
            config_file: Path to YAML configuration file

        Returns:
            dict: Parsed configuration

        Raises:
            FileNotFoundError: If config file doesn't exist
            yaml.YAMLError: If config file is invalid YAML
            ValueError: If required fields are missing
        """
        config_path = Path(config_file)

        if not config_path.exists():
            raise FileNotFoundError(f'Configuration file not found: {config_file}')

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Validate required fields
        if 'yamcs' not in config:
            raise ValueError('Configuration missing required "yamcs" section')

        required_yamcs_fields = ['url', 'instance', 'processor']
        for field in required_yamcs_fields:
            if field not in config['yamcs']:
                raise ValueError(f'YAMCS configuration missing required field: {field}')

        if 'bridges' not in config or not config['bridges']:
            raise ValueError('Configuration missing "bridges" section or no bridges defined')

        # Validate each bridge configuration
        for i, bridge in enumerate(config['bridges']):
            required_bridge_fields = ['name', 'yamcs_parameter', 'ros_topic', 'ros_message_type', 'field_mapping']
            for field in required_bridge_fields:
                if field not in bridge:
                    raise ValueError(f'Bridge {i} missing required field: {field}')

        return config

    def connect_yamcs(self):
        """
        Connect to YAMCS server and get processor instance.

        Raises:
            Exception: If connection fails
        """
        yamcs_config = self.config['yamcs']

        self.get_logger().info(f'Connecting to YAMCS at {yamcs_config["url"]}...')

        self.client = YamcsClient(yamcs_config['url'])
        self.processor = self.client.get_processor(
            yamcs_config['instance'],
            yamcs_config['processor']
        )

        self.get_logger().info(
            f'Connected to YAMCS instance: {yamcs_config["instance"]}, '
            f'processor: {yamcs_config["processor"]}'
        )

    def setup_bridges(self):
        """
        Create publishers and YAMCS subscriptions for all configured bridges.
        """
        self.bridges = []

        for bridge_config in self.config['bridges']:
            try:
                bridge = self.create_bridge(bridge_config)
                self.bridges.append(bridge)
                self.get_logger().info(
                    f'Created bridge "{bridge_config["name"]}": '
                    f'{bridge_config["yamcs_parameter"]} → {bridge_config["ros_topic"]}'
                )
            except Exception as e:
                self.get_logger().error(
                    f'Failed to create bridge "{bridge_config["name"]}": {e}'
                )
                self.get_logger().error(traceback.format_exc())
                raise

    def create_bridge(self, config):
        """
        Create a single bridge (ROS publisher + YAMCS subscription).

        Args:
            config: Bridge configuration dictionary

        Returns:
            dict: Bridge information including publisher and subscription
        """
        # Load ROS message type dynamically
        # Handle both formats: "dragoman_msgs/MessageType" and "dragoman_msgs/msg/MessageType"
        msg_type_parts = config['ros_message_type'].split('/')

        if len(msg_type_parts) == 3:
            # Format: "dragoman_msgs/msg/MessageType"
            msg_module = msg_type_parts[0]
            msg_class = msg_type_parts[2]
        elif len(msg_type_parts) == 2:
            # Format: "dragoman_msgs/MessageType"
            msg_module = msg_type_parts[0]
            msg_class = msg_type_parts[1]
        else:
            raise ValueError(
                f'Invalid message type format: {config["ros_message_type"]}. '
                f'Expected "package/MessageType" or "package/msg/MessageType"'
            )

        try:
            module = importlib.import_module(f'{msg_module}.msg')
            MessageType = getattr(module, msg_class)
        except (ImportError, AttributeError) as e:
            raise ValueError(
                f'Failed to load message type {config["ros_message_type"]}: {e}'
            )

        # Create ROS publisher
        publisher = self.create_publisher(
            MessageType,
            config['ros_topic'],
            10
        )

        # Create YAMCS parameter subscription
        subscription = self.processor.create_parameter_subscription(
            config['yamcs_parameter'],
            on_data=lambda data: self.yamcs_callback(
                data, config, publisher, MessageType
            )
        )

        return {
            'config': config,
            'publisher': publisher,
            'subscription': subscription,
            'message_type': MessageType
        }

    def yamcs_callback(self, data, config, publisher, MessageType):
        """
        Handle YAMCS parameter updates and publish to ROS.

        Args:
            data: YAMCS parameter data
            config: Bridge configuration
            publisher: ROS publisher
            MessageType: ROS message class
        """
        try:
            import numpy as np

            # Create a single message to accumulate all parameter values
            msg = MessageType()

            # Log message fields for debugging
            self.get_logger().debug(f'Message type: {MessageType.__name__}')
            self.get_logger().debug(f'Message fields: {[f for f in dir(msg) if not f.startswith("_")]}')

            # Map all parameters to the message
            for parameter in data.parameters:
                param_name = parameter.name.split('/')[-1]
                self.get_logger().debug(f'Processing parameter: {param_name}, value type: {type(parameter.eng_value).__name__}')
                self.map_fields(parameter, msg, config['field_mapping'])

            # NOTE: Do NOT convert numpy arrays to lists!
            # ROS2 messages expect numpy arrays for fixed-size array fields

            # Set timestamp if message has a header
            if hasattr(msg, 'header'):
                msg.header.stamp = self.get_clock().now().to_msg()

            # Log before publishing
            self.get_logger().debug(f'Publishing message with {len(data.parameters)} parameters')

            # Log message state for debugging
            self.get_logger().debug(f'Message before publish: {msg}')

            # Publish the complete message
            # print(msg)
            # from dragoman_msgs.msg import AllTypes  # Example import, adjust as needed
            # # msg=AllTypes()
            # # print(msg)
            publisher.publish(msg)

            self.get_logger().debug(
                f'Published to {config["ros_topic"]} with {len(data.parameters)} parameters'
            )

        except Exception as e:
            self.get_logger().error(
                f'Error in callback for {config["name"]}: {e}'
            )
            self.get_logger().error(traceback.format_exc())
            # Log the message state for debugging
            self.get_logger().error(f'Message state: {msg}')

    def map_fields(self, parameter, msg, field_mapping):
        """
        Map fields from YAMCS parameter to ROS message.

        Args:
            parameter: YAMCS parameter object
            msg: ROS message object to populate
            field_mapping: Dictionary mapping YAMCS parameter names to ROS fields
        """
        # Get the parameter name (e.g., "T_IntegerSigned" from "/AllTypes/T_IntegerSigned")
        param_name = parameter.name.split('/')[-1]

        self.get_logger().debug(f'Processing parameter: {param_name}')
        self.get_logger().debug(f'  Full name: {parameter.name}')
        self.get_logger().debug(f'  Value: {parameter.eng_value}')
        self.get_logger().debug(f'  Field mapping keys: {list(field_mapping.keys())}')

        # Check if this parameter is in our field mapping
        if param_name not in field_mapping:
            self.get_logger().debug(f'Parameter {param_name} not directly mapped (checking for aggregate members)')
            # Check for aggregate member access (e.g., "T_StatusAggregate.CurrentDraw")
            for yamcs_field, ros_field in field_mapping.items():
                if yamcs_field.startswith(param_name + '.'):
                    # This is an aggregate type, extract the member
                    member_name = yamcs_field.split('.')[1]
                    try:
                        if hasattr(parameter.eng_value, member_name):
                            value = getattr(parameter.eng_value, member_name)
                            # Handle binary data for aggregate members
                            if isinstance(value, bytes):
                                value = list(value)
                            setattr(msg, ros_field, value)
                    except Exception as e:
                        self.get_logger().error(
                            f'Error mapping aggregate field {yamcs_field} → {ros_field}: {e}'
                        )
            return

        # Map the parameter's engineering value to the ROS message field
        ros_field = field_mapping[param_name]
        try:
            value = parameter.eng_value

            # Special handling for different data types
            import numpy as np

            if isinstance(value, bytes):
                # Binary data: convert bytes to list of uint8
                value = list(value)
            elif param_name == 'T_AbsoluteTime':
                # Absolute time: YAMCS returns datetime object, convert to ROS Time
                from builtin_interfaces.msg import Time
                import datetime

                if isinstance(value, datetime.datetime):
                    # Convert datetime to Unix timestamp
                    timestamp = value.timestamp()
                else:
                    # Already a timestamp
                    timestamp = float(value)

                time_msg = Time()
                time_msg.sec = int(timestamp)
                time_msg.nanosec = int((timestamp - int(timestamp)) * 1e9)
                value = time_msg
            elif param_name == 'T_EnumeratedAlarm':
                # Enumerated type: YAMCS returns string label, but ROS expects uint8 value
                # Map the enum labels to their numeric values
                enum_map = {
                    'STATE_OFF': 0,
                    'STATE_NOMINAL': 1,
                    'STATE_FAULT': 2
                }
                if isinstance(value, str) and value in enum_map:
                    value = enum_map[value]
                    self.get_logger().debug(f'Converted enum to numeric: {value}')
            elif isinstance(value, np.ndarray):
                # Numpy arrays: convert to Python list
                value = value.tolist()
            elif hasattr(value, '__iter__') and not isinstance(value, (str, bytes)):
                # Other iterables: ensure they're lists
                value = list(value)

            # Ensure proper types for ALL ROS message fields based on message definition
            # IMPORTANT: Keep numpy arrays as-is! ROS2 expects them for fixed-size arrays

            # builtin_interfaces/Time - already handled above
            if ros_field == 't_absolute_time':
                pass  # Already converted to Time message

            # uint8[16] array - keep as numpy array
            elif ros_field == 't_binary_blob':
                if isinstance(value, bytes):
                    value = np.frombuffer(value, dtype=np.uint8)
                elif not isinstance(value, np.ndarray):
                    value = np.array(value, dtype=np.uint8)

            # bool
            elif ros_field == 't_boolean_flag':
                value = bool(value)

            # uint8
            elif ros_field == 't_enumerated_alarm':
                value = int(value)

            # float64
            elif ros_field == 't_float_raw64':
                value = float(value)

            # uint16[10] array - keep as numpy array
            elif ros_field == 't_integer_array':
                if not isinstance(value, np.ndarray):
                    value = np.array(value, dtype=np.uint16)

            # int32
            elif ros_field == 't_integer_signed':
                value = int(value)

            # uint32
            elif ros_field == 't_relative_time_raw':
                value = int(value)

            # float32
            elif ros_field == 't_status_aggregate_current_draw':
                value = float(value)

            # bool
            elif ros_field == 't_status_aggregate_heater_enabled':
                value = bool(value)

            # uint8[4] array - keep as numpy array
            elif ros_field == 't_status_aggregate_raw_status_flags':
                if isinstance(value, bytes):
                    value = np.frombuffer(value, dtype=np.uint8)
                elif not isinstance(value, np.ndarray):
                    value = np.array(value, dtype=np.uint8)

            # string
            elif ros_field == 't_string_utf8':
                value = str(value)

            setattr(msg, ros_field, value)
            self.get_logger().debug(f'Set {ros_field} = {value} (type: {type(value).__name__})')
            self.get_logger().debug(f'Mapped {param_name} → {ros_field}: {type(value).__name__}')
        except Exception as e:
            self.get_logger().error(
                f'Error mapping parameter {param_name} → {ros_field}: {e}'
            )
            self.get_logger().error(traceback.format_exc())

    def destroy_node(self):
        """Clean up resources."""
        self.get_logger().info('Shutting down YAMCS-ROS Bridge...')

        # Cancel all YAMCS subscriptions
        for bridge in self.bridges:
            try:
                bridge['subscription'].cancel()
            except Exception as e:
                self.get_logger().error(f'Error canceling subscription: {e}')

        super().destroy_node()


def main(args=None):
    """Main entry point for the YAMCS-ROS bridge."""

    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Bridge YAMCS telemetry to ROS2 messages'
    )
    parser.add_argument(
        '--config',
        type=str,
        required=True,
        help='Path to YAML configuration file'
    )
    parser.add_argument(
        '--yamcs-url',
        type=str,
        help='Override YAMCS URL from config file'
    )

    parsed_args = parser.parse_args()

    # Initialize ROS2
    rclpy.init(args=args)

    node = None
    try:
        # Create and run bridge node
        node = YamcsRosBridge(
            parsed_args.config,
            yamcs_url_override=parsed_args.yamcs_url
        )

        # Spin the node
        rclpy.spin(node)

    except KeyboardInterrupt:
        print('\nShutdown requested by user')
    except Exception as e:
        print(f'Error: {e}')
        traceback.print_exc()
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
