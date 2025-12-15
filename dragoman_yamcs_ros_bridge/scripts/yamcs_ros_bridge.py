#!/usr/bin/env python3
"""
YAMCS-ROS Bridge

Bridges YAMCS telemetry parameters to ROS2 messages based on YAML configuration.

Usage:
    python3 yamcs_ros_bridge.py --config bridge_config.yaml [--yamcs-url localhost:8090]
"""

import argparse
import sys
import traceback
from pathlib import Path
import re
from typing import Any, Dict, List, Optional, Type
from datetime import datetime

import yaml
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from yamcs.client import YamcsClient
import importlib
import numpy as np
from builtin_interfaces.msg import Time


class MessageIntrospector:
    """Automatically discovers ROS message fields and creates YAMCS mappings."""

    @staticmethod
    def get_message_fields(MessageType):
        """
        Extract all field names from a ROS message class.

        Args:
            MessageType: ROS message class

        Returns:
            dict: {field_name: field_type}

        Example:
            {'joint_state': 'float[24]'}
        """
        return MessageType._fields_and_field_types

    @staticmethod
    def to_ros_field_name(name):
        """
        Convert a parameter name to ROS 2 compliant field name.
        Uses the same logic as xtce2msg.py for consistency.

        Args:
            name: Parameter name (e.g., "T_IntegerSigned")

        Returns:
            str: ROS field name (e.g., "t_integer_signed")
        """
        # Insert underscores before uppercase letters that follow lowercase letters or digits
        name = re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', name)

        # Convert to lowercase
        name = name.lower()

        # Replace any non-alphanumeric characters (except underscore) with underscore
        name = re.sub(r'[^a-z0-9_]', '_', name)

        # Remove double underscores
        while '__' in name:
            name = name.replace('__', '_')

        # Remove leading underscores and ensure it starts with a letter
        name = name.lstrip('_')
        if name and not name[0].isalpha():
            name = 'field_' + name

        # Remove trailing underscores
        name = name.rstrip('_')

        return name

    @staticmethod
    def create_field_mapping(MessageType, yamcs_parameters, logger=None, strict=False):
        """
        Automatically map YAMCS parameters to ROS message fields.

        Strategy:
        1. Get all ROS message field names
        2. For each YAMCS parameter, extract the parameter name (last part of path)
        3. Convert YAMCS name to ROS snake_case format
        4. Match against available ROS fields
        5. Handle aggregate types (parameters that map to multiple fields)

        Args:
            MessageType: ROS message class
            yamcs_parameters: List of YAMCS parameter paths
            logger: Optional ROS logger for debug output
            strict: If True, only include parameters that can be mapped (filters out unmappable ones)

        Returns:
            tuple: (field_mapping dict, filtered_parameters list) if strict=True
                   field_mapping dict only if strict=False
        """
        ros_fields = set(MessageType._fields_and_field_types.keys())
        field_mapping = {}
        filtered_parameters = []

        for yamcs_path in yamcs_parameters:
            # Extract parameter name from path: "/Curiosity/joint_state" -> "joint_state"
            param_name = yamcs_path.split('/')[-1]

            # Convert to ROS format: "T_IntegerSigned" -> "t_integer_signed"
            ros_field_name = MessageIntrospector.to_ros_field_name(param_name)

            # Check if this is an aggregate type (maps to multiple fields)
            aggregate_fields = [f for f in ros_fields if f.startswith(ros_field_name + '_')]

            if aggregate_fields:
                # This is an aggregate type - create mappings for each member
                if logger:
                    logger.debug(f"Detected aggregate type '{param_name}' with {len(aggregate_fields)} members")

                for agg_field in aggregate_fields:
                    # Extract member name: "t_status_aggregate_current_draw" -> "CurrentDraw"
                    member_suffix = agg_field[len(ros_field_name)+1:]
                    # Convert to PascalCase: "current_draw" -> "CurrentDraw"
                    yamcs_member = ''.join(word.capitalize() for word in member_suffix.split('_'))

                    # Create aggregate member mapping: "T_StatusAggregate.CurrentDraw" -> "t_status_aggregate_current_draw"
                    aggregate_key = f"{param_name}.{yamcs_member}"
                    field_mapping[aggregate_key] = agg_field

                    if logger:
                        logger.debug(f"  Aggregate member: {aggregate_key} → {agg_field}")

                filtered_parameters.append(yamcs_path)

            elif ros_field_name in ros_fields:
                # Direct mapping found
                field_mapping[param_name] = ros_field_name
                if logger:
                    logger.debug(f"Mapped: {param_name} → {ros_field_name}")
                filtered_parameters.append(yamcs_path)

            elif param_name in ros_fields:
                # Try exact match without conversion
                field_mapping[param_name] = param_name
                if logger:
                    logger.debug(f"Exact match: {param_name} → {param_name}")
                filtered_parameters.append(yamcs_path)

            else:
                # No mapping found
                if logger and not strict:
                    logger.warning(
                        f"Cannot auto-map YAMCS parameter '{param_name}' (from {yamcs_path}). "
                        f"Available ROS fields: {sorted(ros_fields)}"
                    )
                elif logger and strict:
                    logger.debug(
                        f"Skipping unmappable parameter '{param_name}' (from {yamcs_path})"
                    )

        if strict:
            return field_mapping, filtered_parameters
        return field_mapping


class TypeConverter:
    """Handles type conversion from YAMCS parameter values to ROS message field types."""

    # Enum mapping for enumerated types
    ENUM_MAPPINGS = {
        'T_EnumeratedAlarm': {
            'STATE_OFF': 0,
            'STATE_NOMINAL': 1,
            'STATE_FAULT': 2
        }
    }

    @staticmethod
    def convert_value(param_name: str, ros_field: str, value: Any, logger: Optional[Any] = None) -> Any:
        """
        Convert a YAMCS parameter value to the appropriate ROS message field type.

        Args:
            param_name: YAMCS parameter name
            ros_field: ROS message field name
            value: Raw value from YAMCS
            logger: Optional logger for debug output

        Returns:
            Converted value suitable for ROS message field
        """
        # Handle bytes/binary data
        if isinstance(value, bytes):
            return TypeConverter._convert_bytes(ros_field, value)

        # Handle absolute time
        if param_name == 'T_AbsoluteTime':
            return TypeConverter._convert_absolute_time(value)

        # Handle enumerated types
        if param_name == 'T_EnumeratedAlarm':
            return TypeConverter._convert_enum(param_name, value, logger)

        # Handle numpy arrays
        if isinstance(value, np.ndarray):
            return value.tolist()

        # Handle other iterables (but not strings or bytes)
        if hasattr(value, '__iter__') and not isinstance(value, (str, bytes)):
            return list(value)

        # Field-specific conversions
        return TypeConverter._convert_by_field(ros_field, value)

    @staticmethod
    def _convert_bytes(ros_field: str, value: bytes) -> Any:
        """Convert bytes to appropriate format based on ROS field."""
        if ros_field in ('t_binary_blob', 't_status_aggregate_raw_status_flags'):
            return np.frombuffer(value, dtype=np.uint8)
        return list(value)

    @staticmethod
    def _convert_absolute_time(value: Any) -> Time:
        """Convert datetime or timestamp to ROS Time message."""
        if isinstance(value, datetime):
            timestamp = value.timestamp()
        else:
            timestamp = float(value)

        time_msg = Time()
        time_msg.sec = int(timestamp)
        time_msg.nanosec = int((timestamp - int(timestamp)) * 1e9)
        return time_msg

    @staticmethod
    def _convert_enum(param_name: str, value: Any, logger: Optional[Any] = None) -> Any:
        """Convert enumerated string to numeric value."""
        if isinstance(value, str) and param_name in TypeConverter.ENUM_MAPPINGS:
            enum_map = TypeConverter.ENUM_MAPPINGS[param_name]
            if value in enum_map:
                converted = enum_map[value]
                if logger:
                    logger.debug(f'Converted enum to numeric: {converted}')
                return converted
        return value

    @staticmethod
    def _convert_by_field(ros_field: str, value: Any) -> Any:
        """Convert value based on ROS field name."""
        # Boolean fields
        if ros_field in ('t_boolean_flag', 't_status_aggregate_heater_enabled'):
            return bool(value)

        # Integer fields
        if ros_field in ('t_enumerated_alarm', 't_integer_signed', 't_relative_time_raw'):
            return int(value)

        # Float fields
        if ros_field in ('t_float_raw64', 't_status_aggregate_current_draw'):
            return float(value)

        # Array fields
        if ros_field == 't_integer_array':
            if not isinstance(value, np.ndarray):
                return np.array(value, dtype=np.uint16)
            return value

        if ros_field in ('t_binary_blob', 't_status_aggregate_raw_status_flags'):
            if isinstance(value, bytes):
                return np.frombuffer(value, dtype=np.uint8)
            if not isinstance(value, np.ndarray):
                return np.array(value, dtype=np.uint8)
            return value

        # String fields
        if ros_field == 't_string_utf8':
            return str(value)

        # Return as-is for other types
        return value


class MessageTypeLoader:
    """Handles dynamic loading of ROS message types."""

    @staticmethod
    def load_message_type(message_type_str: str) -> Type:
        """
        Dynamically load a ROS message type from a string specification.

        Args:
            message_type_str: Message type in format "package/MessageType" or "package/msg/MessageType"

        Returns:
            The loaded message class

        Raises:
            ValueError: If message type format is invalid or loading fails
        """
        msg_type_parts = message_type_str.split('/')

        if len(msg_type_parts) == 3:
            # Format: "package/msg/MessageType"
            msg_module = msg_type_parts[0]
            msg_class = msg_type_parts[2]
        elif len(msg_type_parts) == 2:
            # Format: "package/MessageType"
            msg_module = msg_type_parts[0]
            msg_class = msg_type_parts[1]
        else:
            raise ValueError(
                f'Invalid message type format: {message_type_str}. '
                f'Expected "package/MessageType" or "package/msg/MessageType"'
            )

        try:
            module = importlib.import_module(f'{msg_module}.msg')
            return getattr(module, msg_class)
        except (ImportError, AttributeError) as e:
            raise ValueError(
                f'Failed to load message type {message_type_str}: {e}'
            )


class YamcsParameterDiscovery:
    """Discovers available parameters from YAMCS server."""

    def __init__(self, yamcs_client: YamcsClient, instance: str, logger: Optional[Any] = None):
        """
        Initialize parameter discovery service.

        Args:
            yamcs_client: YamcsClient instance
            instance: YAMCS instance name
            logger: Optional ROS logger
        """
        self.client = yamcs_client
        self.instance = instance
        self.logger = logger

    def discover_parameters_in_namespace(self, namespace: str) -> List[str]:
        """
        Query YAMCS for all parameters in a namespace.

        Uses YAMCS Python Client API:
            mdb = client.get_mdb(instance)
            parameters = mdb.list_parameters()

        Args:
            namespace: YAMCS namespace (e.g., "/Curiosity")

        Returns:
            list: Full parameter paths (e.g., ["/Curiosity/joint_state"])
        """
        try:
            mdb = self.client.get_mdb(self.instance)
            all_params = mdb.list_parameters()

            # Filter by namespace
            if namespace:
                filtered = [p.qualified_name for p in all_params
                           if p.qualified_name.startswith(namespace)]
                if self.logger:
                    self.logger.debug(
                        f"Discovered {len(filtered)} parameters in namespace '{namespace}'"
                    )
                return filtered

            if self.logger:
                self.logger.info(f"Discovered {len(all_params)} total parameters")
            return [p.qualified_name for p in all_params]

        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to discover parameters: {e}")
            raise

    def discover_parameters_in_packet(self, packet_name: str) -> List[str]:
        """
        Query YAMCS for all parameters in a specific packet/container.

        Since the YAMCS Python Client doesn't expose container introspection directly,
        we use the packet name as a namespace prefix to filter parameters.

        Args:
            packet_name: YAMCS packet/container name (e.g., "IMetro/IMetroTelemetryPacket")
                        This is converted to a namespace by extracting the path prefix.
                        Example: "IMetro/IMetroTelemetryPacket" -> "/IMetro"

        Returns:
            list: Full parameter paths for all parameters in the packet's namespace
        """
        try:
            # Extract namespace from packet name
            # "IMetro/IMetroTelemetryPacket" -> "/IMetro"
            # "AllTypes/AllTelemetryPacket" -> "/AllTypes"
            namespace = '/' + packet_name.split('/')[0]

            if self.logger:
                self.logger.debug(
                    f"Converting packet name '{packet_name}' to namespace '{namespace}'"
                )

            # Use namespace-based discovery
            return self.discover_parameters_in_namespace(namespace)

        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to discover parameters for packet '{packet_name}': {e}")
            raise


class YamcsRosBridge(Node):
    """
    ROS2 node that bridges YAMCS telemetry to ROS2 messages.

    Reads configuration from YAML file to determine which YAMCS parameters
    to subscribe to and which ROS2 messages to publish.
    """

    def __init__(self, config_file: str, yamcs_url_override: Optional[str] = None):
        """
        Initialize the YAMCS-ROS bridge.

        Args:
            config_file: Path to YAML configuration file
            yamcs_url_override: Optional override for YAMCS URL from config
        """
        super().__init__('yamcs_ros_bridge')

        self.get_logger().debug(f'Loading configuration from: {config_file}')

        # Load and validate configuration
        try:
            self.config = self.load_config(config_file)
        except Exception as e:
            self.get_logger().error(f'Failed to load configuration: {e}')
            raise

        # Override YAMCS URL if provided
        if yamcs_url_override:
            self.config['yamcs']['url'] = yamcs_url_override
            self.get_logger().debug(f'Overriding YAMCS URL: {yamcs_url_override}')

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

        # Log summary of created bridges
        self.get_logger().info(f'YAMCS-ROS Bridge initialized with {len(self.bridges)} bridge(s):')
        for bridge in self.bridges:
            config = bridge['config']
            self.get_logger().info(f"  • {config['name']}: {config['ros_topic']}")

    def load_config(self, config_file: str) -> Dict[str, Any]:
        """
        Load and validate YAML configuration file.

        Supports both old (manual) and new (auto-discovery) configuration formats:

        Old format (manual):
            - name: "bridge_name"
              yamcs_parameter: ["/path/to/param"]
              ros_topic: "/topic"
              ros_message_type: "package/msg/Type"
              field_mapping: {param: field}

        New format (auto-discovery):
            - ros_message_type: "package/msg/Type"
              ros_topic: "/topic"
              yamcs_namespace: "/Namespace"  # Optional

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
            # ros_message_type and ros_topic are always required
            if 'ros_message_type' not in bridge:
                raise ValueError(f'Bridge {i} missing required field: ros_message_type')
            if 'ros_topic' not in bridge:
                raise ValueError(f'Bridge {i} missing required field: ros_topic')

            # Check if this is old format (has all manual fields) or new format (auto-discovery)
            has_manual_fields = all(field in bridge for field in ['name', 'yamcs_parameter', 'field_mapping'])
            has_auto_fields = 'yamcs_namespace' in bridge or ('yamcs_parameter' not in bridge and 'field_mapping' not in bridge)

            if not has_manual_fields and not has_auto_fields:
                # Partial configuration - need either all manual fields or use auto-discovery
                self.get_logger().info(
                    f'Bridge {i} will use auto-discovery mode (missing manual configuration fields)'
                )

        return config

    def connect_yamcs(self) -> None:
        """
        Connect to YAMCS server and get processor instance.

        Raises:
            Exception: If connection fails
        """
        yamcs_config = self.config['yamcs']

        self.get_logger().debug(f'Connecting to YAMCS at {yamcs_config["url"]}...')

        self.client = YamcsClient(yamcs_config['url'])
        self.processor = self.client.get_processor(
            yamcs_config['instance'],
            yamcs_config['processor']
        )

        self.get_logger().debug(
            f'Connected to YAMCS instance: {yamcs_config["instance"]}, '
            f'processor: {yamcs_config["processor"]}'
        )

    def setup_bridges(self) -> None:
        """
        Create publishers and YAMCS subscriptions for all configured bridges.
        """
        self.bridges = []

        for bridge_config in self.config['bridges']:
            try:
                bridge = self.create_bridge(bridge_config)
                self.bridges.append(bridge)
                bridge_name = bridge_config.get('name', 'unknown')
                self.get_logger().debug(
                    f'Created bridge "{bridge_name}": '
                    f'{bridge_config["yamcs_parameter"]} → {bridge_config["ros_topic"]}'
                )
            except Exception as e:
                # Get bridge name safely (might not be set yet)
                bridge_name = bridge_config.get('name', bridge_config.get('ros_message_type', 'unknown'))
                self.get_logger().error(
                    f'Failed to create bridge "{bridge_name}": {e}'
                )
                self.get_logger().error(traceback.format_exc())
                raise

    def create_bridge(self, config: Dict[str, Any]) -> Dict[str, Any]:
        """
        Create a single bridge (ROS publisher + YAMCS subscription).

        Supports both manual and auto-discovery modes:
        - Manual: Uses provided yamcs_parameter and field_mapping
        - Auto: Discovers parameters from YAMCS and generates field mapping

        Args:
            config: Bridge configuration dictionary

        Returns:
            dict: Bridge information including publisher and subscription
        """
        # Load ROS message type dynamically
        MessageType = MessageTypeLoader.load_message_type(config['ros_message_type'])
        msg_class = config['ros_message_type'].split('/')[-1]

        # Auto-discover YAMCS parameters if not explicitly provided
        if 'yamcs_parameter' not in config:
            discovery = YamcsParameterDiscovery(
                self.client,
                self.config['yamcs']['instance'],
                self.get_logger()
            )

            # Try packet name first (most specific), then namespace
            if 'yamcs_packet_name' in config:
                packet_name = config['yamcs_packet_name']
                self.get_logger().debug(
                    f"Auto-discovering YAMCS parameters for {config['ros_message_type']} "
                    f"from packet '{packet_name}'"
                )
                yamcs_parameters = discovery.discover_parameters_in_packet(packet_name)
            else:
                namespace = config.get('yamcs_namespace', '')
                self.get_logger().debug(
                    f"Auto-discovering YAMCS parameters for {config['ros_message_type']} "
                    f"in namespace '{namespace}'"
                )
                yamcs_parameters = discovery.discover_parameters_in_namespace(namespace)

            config['yamcs_parameter'] = yamcs_parameters

            self.get_logger().debug(
                f"Discovered {len(yamcs_parameters)} parameters"
            )

        # Auto-generate field mapping using introspection if not provided
        if 'field_mapping' not in config:
            self.get_logger().debug(
                f"Auto-generating field mapping for {config['ros_message_type']}"
            )

            introspector = MessageIntrospector()

            # Use strict mode to filter out unmappable parameters
            field_mapping, filtered_parameters = introspector.create_field_mapping(
                MessageType,
                config['yamcs_parameter'],
                self.get_logger(),
                strict=True
            )

            # Update config with only mappable parameters
            config['yamcs_parameter'] = filtered_parameters
            config['field_mapping'] = field_mapping

            if len(filtered_parameters) == 0:
                raise ValueError(
                    f"No mappable parameters found for {config['ros_message_type']}. "
                    f"ROS message fields: {list(MessageType._fields_and_field_types.keys())}"
                )

            self.get_logger().debug(
                f"Generated {len(field_mapping)} field mappings from {len(filtered_parameters)} parameters"
            )

        # Generate bridge name if not provided
        if 'name' not in config:
            config['name'] = f"{msg_class}_bridge"

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

    def yamcs_callback(self, data: Any, config: Dict[str, Any], publisher: Publisher, MessageType: Type) -> None:
        """
        Handle YAMCS parameter updates and publish to ROS.

        Args:
            data: YAMCS parameter data
            config: Bridge configuration
            publisher: ROS publisher
            MessageType: ROS message class
        """
        try:
            # Create a single message to accumulate all parameter values
            msg = MessageType()

            self.get_logger().debug(
                f'Processing {len(data.parameters)} parameters for {MessageType.__name__}'
            )

            # Map all parameters to the message
            for parameter in data.parameters:
                self.map_fields(parameter, msg, config['field_mapping'])

            # Set timestamp if message has a header
            if hasattr(msg, 'header'):
                msg.header.stamp = self.get_clock().now().to_msg()

            # Publish the complete message
            publisher.publish(msg)

            self.get_logger().debug(
                f'Published to {config["ros_topic"]} with {len(data.parameters)} parameters'
            )

        except Exception as e:
            self.get_logger().error(
                f'Error in callback for {config["name"]}: {e}\n{traceback.format_exc()}'
            )

    def map_fields(self, parameter: Any, msg: Any, field_mapping: Dict[str, str]) -> None:
        """
        Map fields from YAMCS parameter to ROS message.

        Args:
            parameter: YAMCS parameter object
            msg: ROS message object to populate
            field_mapping: Dictionary mapping YAMCS parameter names to ROS fields
        """
        # Get the parameter name (e.g., "T_IntegerSigned" from "/AllTypes/T_IntegerSigned")
        param_name = parameter.name.split('/')[-1]

        self.get_logger().debug(
            f'Mapping parameter: {param_name} (value type: {type(parameter.eng_value).__name__})'
        )

        # Check if this parameter is in our field mapping
        if param_name not in field_mapping:
            # Check for aggregate member access (e.g., "T_StatusAggregate.CurrentDraw")
            self._map_aggregate_fields(parameter, param_name, msg, field_mapping)
            return

        # Map the parameter's engineering value to the ROS message field
        ros_field = field_mapping[param_name]
        try:
            value = TypeConverter.convert_value(
                param_name, ros_field, parameter.eng_value, self.get_logger()
            )
            setattr(msg, ros_field, value)
            self.get_logger().debug(
                f'Mapped {param_name} → {ros_field}: {type(value).__name__}'
            )
        except Exception as e:
            self.get_logger().error(
                f'Error mapping parameter {param_name} → {ros_field}: {e}\n{traceback.format_exc()}'
            )

    def _map_aggregate_fields(
        self, parameter: Any, param_name: str, msg: Any, field_mapping: Dict[str, str]
    ) -> None:
        """
        Map aggregate type fields from YAMCS parameter to ROS message.

        Args:
            parameter: YAMCS parameter object
            param_name: Parameter name
            msg: ROS message object to populate
            field_mapping: Dictionary mapping YAMCS parameter names to ROS fields
        """
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
                        self.get_logger().debug(
                            f'Mapped aggregate {yamcs_field} → {ros_field}'
                        )
                except Exception as e:
                    self.get_logger().error(
                        f'Error mapping aggregate field {yamcs_field} → {ros_field}: {e}'
                    )

    def destroy_node(self) -> None:
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

    # Parse only known args to allow ROS args to pass through
    parsed_args, unknown = parser.parse_known_args()

    # Initialize ROS2 with all arguments (including ROS-specific ones)
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
