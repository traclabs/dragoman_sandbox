#!/usr/bin/env python3
"""
YAMCS-ROS Bridge

Bridges YAMCS telemetry packets to ROS2 messages using packet subscription.

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
import time

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

        This version supports *nested* ROS messages for XTCE aggregate types.

        Supported YAMCS naming patterns:
        - Scalar/normal parameter:
            "/AllTypes/T_IntegerSigned" -> "T_IntegerSigned" -> "t_integer_signed"
        - Aggregate member parameter (common in some MDB exports):
            "/AllTypes/T_StatusAggregate/CurrentDraw" -> "T_StatusAggregate.CurrentDraw"
            "/AllTypes/T_StatusAggregate.CurrentDraw" -> "T_StatusAggregate.CurrentDraw"
          which maps to a *nested* ROS field path like:
            "t_status_aggregate.current_draw"

        Args:
            MessageType: ROS message class
            yamcs_parameters: List of YAMCS parameter paths
            logger: Optional ROS logger for debug output
            strict: If True, only include parameters that can be mapped (filters out unmappable ones)

        Returns:
            tuple: (field_mapping dict, filtered_parameters list) if strict=True
                   field_mapping dict only if strict=False
        """
        ros_fields_and_types = dict(MessageType._fields_and_field_types)
        ros_fields = set(ros_fields_and_types.keys())
        field_mapping: Dict[str, str] = {}
        filtered_parameters: List[str] = []

        def _try_add_mapping(yamcs_key: str, ros_path: str, yamcs_path: str) -> bool:
            # Avoid duplicate entries if the same param is encountered multiple ways
            if yamcs_key in field_mapping:
                return True
            field_mapping[yamcs_key] = ros_path
            filtered_parameters.append(yamcs_path)
            if logger:
                logger.debug(f"Mapped: {yamcs_key} → {ros_path}")
            return True

        for yamcs_path in yamcs_parameters:
            # Normalize and split: "/AllTypes/T_StatusAggregate/CurrentDraw" -> ["AllTypes", "T_StatusAggregate", "CurrentDraw"]
            parts = [p for p in yamcs_path.strip('/').split('/') if p]
            if not parts:
                continue

            leaf = parts[-1]

            # Some YAMCS/MDB setups express members via "/" nesting, so attempt parent.leaf first.
            # Example: ".../T_StatusAggregate/CurrentDraw" -> "T_StatusAggregate.CurrentDraw"
            if len(parts) >= 2:
                parent = parts[-2]
                dotted_candidate = f"{parent}.{leaf}"
                ros_path = MessageIntrospector._yamcs_member_to_ros_path(MessageType, dotted_candidate)
                if ros_path:
                    _try_add_mapping(dotted_candidate, ros_path, yamcs_path)
                    continue

            # Some setups express members via "." already
            if '.' in leaf:
                ros_path = MessageIntrospector._yamcs_member_to_ros_path(MessageType, leaf)
                if ros_path:
                    _try_add_mapping(leaf, ros_path, yamcs_path)
                    continue

            # Scalar / whole-aggregate parameter mapping (aggregate maps to a single nested message field)
            param_name = leaf
            ros_field_name = MessageIntrospector.to_ros_field_name(param_name)

            if ros_field_name in ros_fields:
                _try_add_mapping(param_name, ros_field_name, yamcs_path)
                continue

            if param_name in ros_fields:
                _try_add_mapping(param_name, param_name, yamcs_path)
                continue

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

    @staticmethod
    def _yamcs_member_to_ros_path(MessageType, yamcs_member_name: str) -> Optional[str]:
        """
        Convert a YAMCS aggregate member name (e.g. "T_StatusAggregate.CurrentDraw")
        into a ROS nested field path (e.g. "t_status_aggregate.current_draw"), if valid.
        """
        if '.' not in yamcs_member_name:
            return None

        root_name, member_name = yamcs_member_name.split('.', 1)
        root_field = MessageIntrospector.to_ros_field_name(root_name)
        member_field = MessageIntrospector.to_ros_field_name(member_name)

        root_types = getattr(MessageType, "_fields_and_field_types", {})
        if root_field not in root_types:
            return None

        root_type_str = root_types[root_field]

        # Strip any array suffix (we don't support mapping into arrays of messages here)
        m = re.match(r'^(?P<base>[^\[]+)', str(root_type_str))
        root_base_type = m.group('base') if m else str(root_type_str)

        # Root must be a nested message type (qualified or local/unqualified)
        if not TypeConverter.is_ros_message_type(root_base_type):
            return None

        # If unqualified, infer the package from the parent MessageType
        qualified_type = root_base_type
        if '/' not in root_base_type:
            # e.g. "dragoman_sample_msgs.msg._GatewayTelemetryPacket" -> "dragoman_sample_msgs"
            pkg = getattr(MessageType, "__module__", "").split('.', 1)[0]
            if not pkg:
                return None
            qualified_type = f"{pkg}/msg/{root_base_type}"

        try:
            NestedType = MessageTypeLoader.load_message_type(qualified_type)
        except Exception:
            return None

        nested_fields = getattr(NestedType, "_fields_and_field_types", {})
        if member_field not in nested_fields:
            return None

        return f"{root_field}.{member_field}"


class TypeConverter:
    """Handles type conversion from YAMCS parameter values to ROS message field types."""

    _ROS_INT_TYPES = {
        'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64'
    }
    _ROS_FLOAT_TYPES = {'float32', 'float64'}

    @staticmethod
    def is_ros_message_type(type_str: str) -> bool:
        """
        Return True if a ROS field type string refers to a nested message type.

        Supports both:
        - fully-qualified: "some_pkg/msg/SomeType"
        - local/unqualified: "SomeType"
        - array variants: "SomeType[3]", "some_pkg/msg/SomeType[]"
        """
        if not isinstance(type_str, str):
            return False

        m = re.match(r'^(?P<base>[^\[]+)', type_str)
        base = m.group('base') if m else type_str

        primitives = {
            'bool',
            'byte',
            'char',
            'float32',
            'float64',
            'int8',
            'uint8',
            'int16',
            'uint16',
            'int32',
            'uint32',
            'int64',
            'uint64',
            'string',
            'wstring',
        }

        if base in primitives:
            return False

        # Fully-qualified message type
        if '/' in base:
            return True

        # Unqualified message types are typically PascalCase (primitives are lowercase)
        return bool(base) and base[0].isalpha() and base[0].isupper()

    @staticmethod
    def _parse_ros_type(type_str: str) -> Dict[str, Any]:
        """
        Parse ROS type strings like:
          - "uint32"
          - "float32[7]"
          - "uint8[]"
          - "builtin_interfaces/msg/Time"
        """
        m = re.match(r'^(?P<base>[^\[]+)(?:\[(?P<len>\d*)\])?$', type_str)
        if not m:
            return {'base': type_str, 'is_array': False, 'array_len': None}

        base = m.group('base')
        arr = m.group('len')
        if arr is None:
            return {'base': base, 'is_array': False, 'array_len': None}

        # "[]" -> variable length, "[7]" -> fixed length
        return {
            'base': base,
            'is_array': True,
            'array_len': int(arr) if arr.isdigit() else None
        }

    @staticmethod
    def snake_to_pascal(name: str) -> str:
        return ''.join(word.capitalize() for word in name.split('_') if word)

    @staticmethod
    def _get_member_value(obj: Any, field_name: str) -> Any:
        """
        Try to fetch a member value from a YAMCS engineering value object.

        Supports:
        - dict-like values: value["field"] or value["Field"]
        - attribute values: value.field or value.Field
        """
        if obj is None:
            raise AttributeError("No object")

        # dict lookup
        if isinstance(obj, dict):
            if field_name in obj:
                return obj[field_name]
            pascal = TypeConverter.snake_to_pascal(field_name)
            if pascal in obj:
                return obj[pascal]

        # attribute lookup
        if hasattr(obj, field_name):
            return getattr(obj, field_name)

        pascal = TypeConverter.snake_to_pascal(field_name)
        if hasattr(obj, pascal):
            return getattr(obj, pascal)

        raise AttributeError(f"Missing member {field_name}")

    @staticmethod
    def convert_value(
        param_name: str,
        expected_ros_type: str,
        value: Any,
        logger: Optional[Any] = None
    ) -> Any:
        """
        Convert a YAMCS parameter value to the appropriate ROS message field type.

        Args:
            param_name: YAMCS parameter name (used for special-case conversions like enums)
            expected_ros_type: ROS field type string from _fields_and_field_types
            value: Raw value from YAMCS
            logger: Optional logger for debug output
        """
        if expected_ros_type is None:
            return value

        # Handle nested messages (callers should use populate_message instead)
        if TypeConverter.is_ros_message_type(expected_ros_type):
            return value

        parsed = TypeConverter._parse_ros_type(expected_ros_type)
        base = parsed['base']
        is_array = parsed['is_array']

        # Time messages represented as a message type (but sometimes appear as base types in some codegen)
        if base.endswith('/Time'):
            return TypeConverter._convert_absolute_time(value)

        # bytes -> list of ints for sequences (esp uint8[])
        if isinstance(value, bytes):
            if is_array and base == 'uint8':
                return list(value)
            return list(value)

        # numpy arrays -> python lists (works for both fixed and variable-length arrays)
        if isinstance(value, np.ndarray):
            return value.tolist()

        # Iterables -> list for array fields
        if is_array and hasattr(value, '__iter__') and not isinstance(value, (str, bytes)):
            return list(value)

        # Scalars by base type
        if base == 'bool':
            if isinstance(value, str):
                v = value.strip().lower()
                if v in ('true', 't', 'yes', 'y', '1', 'on'):
                    return True
                if v in ('false', 'f', 'no', 'n', '0', 'off'):
                    return False
            return bool(value)

        if base in TypeConverter._ROS_INT_TYPES:
            if isinstance(value, str):
                # Try parsing numeric strings (supports "0x.." via base=0)
                try:
                    return int(value.strip(), 0)
                except Exception:
                    # Some values might come as "12.0" strings; allow that too
                    return int(float(value.strip()))
            return int(value)

        if base in TypeConverter._ROS_FLOAT_TYPES:
            return float(value)

        if base == 'string':
            return str(value)

        # Unknown: return as-is
        return value

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
    def populate_message(msg: Any, eng_value: Any, logger: Optional[Any] = None, root_param_name: str = "") -> None:
        """
        Populate a (nested) ROS message from a YAMCS engineering value object.

        It tries to match ROS snake_case fields against:
        - same-name members
        - PascalCase members (common for XTCE aggregates)
        """
        fields_and_types = getattr(msg, "_fields_and_field_types", {})
        for field_name, field_type in fields_and_types.items():
            try:
                member_value = TypeConverter._get_member_value(eng_value, field_name)
            except AttributeError:
                continue

            # Nested message field
            if TypeConverter.is_ros_message_type(field_type):
                nested_msg = getattr(msg, field_name)
                TypeConverter.populate_message(
                    nested_msg,
                    member_value,
                    logger=logger,
                    root_param_name=root_param_name
                )
                continue

            try:
                converted = TypeConverter.convert_value(
                    root_param_name or field_name,
                    field_type,
                    member_value,
                    logger=logger
                )
                setattr(msg, field_name, converted)
            except Exception as e:
                if logger:
                    logger.error(f"Error populating nested field '{field_name}' ({field_type}): {e}")


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


class YamcsRosBridge(Node):
    """
    ROS2 node that bridges YAMCS telemetry packets to ROS2 messages.

    Subscribes to YAMCS packets/containers and automatically maps parameters
    to ROS message fields using naming conventions.
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
        self.get_logger().debug(f'YAMCS-ROS Bridge initialized with {len(self.bridges)} bridge(s):')
        for bridge in self.bridges:
            config = bridge['config']
            self.get_logger().debug(f"  • {config['name']}: {config['ros_topic']}")

    def load_config(self, config_file: str) -> Dict[str, Any]:
        """
        Load and validate YAML configuration file.

        Expected configuration format:
            yamcs:
              url: "localhost:8090"
              instance: "dragoman"
              processor: "realtime"

            bridges:
              - ros_message_type: "package/msg/Type"
                ros_topic: "/topic"
                yamcs_packet_name: "Namespace/PacketName"

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
            # Required fields for packet subscription
            if 'ros_message_type' not in bridge:
                raise ValueError(f'Bridge {i} missing required field: ros_message_type')
            if 'ros_topic' not in bridge:
                raise ValueError(f'Bridge {i} missing required field: ros_topic')
            if 'yamcs_packet_name' not in bridge:
                raise ValueError(f'Bridge {i} missing required field: yamcs_packet_name')

        return config

    def connect_yamcs(self, max_retries: int = 10, initial_delay: float = 1.0) -> None:
        """
        Connect to YAMCS server and get processor instance with retry logic.

        Args:
            max_retries: Maximum number of connection attempts (default: 10)
            initial_delay: Initial delay between retries in seconds (default: 1.0)

        Raises:
            Exception: If connection fails after all retries
        """
        yamcs_config = self.config['yamcs']

        for attempt in range(max_retries):
            try:
                self.get_logger().debug(
                    f'Connecting to YAMCS at {yamcs_config["url"]} '
                    f'(attempt {attempt + 1}/{max_retries})...'
                )

                self.client = YamcsClient(yamcs_config['url'])
                self.processor = self.client.get_processor(
                    yamcs_config['instance'],
                    yamcs_config['processor']
                )

                self.get_logger().debug(
                    f'Successfully connected to YAMCS instance: {yamcs_config["instance"]}, '
                    f'processor: {yamcs_config["processor"]}'
                )
                return

            except Exception as e:
                if attempt < max_retries - 1:
                    # Calculate exponential backoff delay
                    delay = initial_delay * (2 ** attempt)
                    self.get_logger().warn(
                        f'Failed to connect to YAMCS: {e}. '
                        f'Retrying in {delay:.1f} seconds...'
                    )
                    time.sleep(delay)
                else:
                    self.get_logger().error(
                        f'Failed to connect to YAMCS after {max_retries} attempts: {e}'
                    )
                    raise

    def setup_bridges(self, max_retries: int = 5, initial_delay: float = 2.0) -> None:
        """
        Create publishers and YAMCS packet subscriptions for all configured bridges with retry logic.

        Args:
            max_retries: Maximum number of retry attempts per bridge (default: 5)
            initial_delay: Initial delay between retries in seconds (default: 2.0)
        """
        self.bridges = []

        for bridge_config in self.config['bridges']:
            bridge_name = bridge_config.get('name', bridge_config['ros_message_type'].split('/')[-1])

            for attempt in range(max_retries):
                try:
                    bridge = self.create_bridge(bridge_config)
                    self.bridges.append(bridge)
                    self.get_logger().debug(
                        f'Created bridge "{bridge_name}": '
                        f'{bridge_config["yamcs_packet_name"]} → {bridge_config["ros_topic"]}'
                    )
                    break  # Success, exit retry loop

                except Exception as e:
                    if attempt < max_retries - 1:
                        delay = initial_delay * (2 ** attempt)
                        self.get_logger().warn(
                            f'Failed to create bridge "{bridge_name}" (attempt {attempt + 1}/{max_retries}): {e}. '
                            f'Retrying in {delay:.1f} seconds...'
                        )
                        time.sleep(delay)
                    else:
                        self.get_logger().error(
                            f'Failed to create bridge "{bridge_name}" after {max_retries} attempts: {e}'
                        )
                        self.get_logger().debug(traceback.format_exc())
                        # Continue with other bridges instead of raising

    def create_bridge(self, config: Dict[str, Any]) -> Dict[str, Any]:
        """
        Create a single bridge (ROS publisher + YAMCS packet subscription).

        Args:
            config: Bridge configuration dictionary with:
                - ros_message_type: ROS message type string
                - ros_topic: ROS topic to publish to
                - yamcs_packet_name: YAMCS packet/container name

        Returns:
            dict: Bridge information including publisher and subscription
        """
        # Load ROS message type dynamically
        MessageType = MessageTypeLoader.load_message_type(config['ros_message_type'])
        msg_class = config['ros_message_type'].split('/')[-1]
        packet_name = config['yamcs_packet_name']

        # Generate bridge name if not provided
        if 'name' not in config:
            config['name'] = f"{msg_class}_bridge"

        self.get_logger().debug(
            f"Creating packet subscription bridge for {config['ros_message_type']} "
            f"(packet: '{packet_name}')"
        )

        # Create ROS publisher
        publisher = self.create_publisher(
            MessageType,
            config['ros_topic'],
            10
        )

        # Get list of parameters for this container from the MDB
        # We'll query these parameters when the container arrives
        parameter_names = self._get_container_parameters(packet_name, MessageType)

        # Create YAMCS packet/container subscription
        subscription = self.processor.create_container_subscription(
            containers=[packet_name],
            on_data=lambda packet: self.yamcs_packet_callback(
                packet, config, publisher, MessageType, parameter_names
            )
        )

        return {
            'config': config,
            'publisher': publisher,
            'subscription': subscription,
            'message_type': MessageType,
            'parameter_names': parameter_names
        }

    def _get_container_parameters(self, container_name: str, MessageType: Type) -> List[str]:
        """
        Get the list of parameter names for a container by inspecting the ROS message type.

        Args:
            container_name: YAMCS container name
            MessageType: ROS message type

        Returns:
            List of fully-qualified parameter names
        """
        # Extract namespace from container name (e.g., "/Curiosity/CuriosityTelemetryPacket" -> "Curiosity")
        parts = [p for p in container_name.strip('/').split('/') if p]
        if not parts:
            return []

        namespace = parts[0] if len(parts) > 1 else ""

        # Get all fields from the ROS message
        fields = MessageType._fields_and_field_types
        parameter_names = []

        for field_name in fields.keys():
            # Convert ROS field name back to YAMCS parameter name
            # e.g., "ccsds_packet_id" -> "/Curiosity/ccsds_packet_id"
            param_name = f"/{namespace}/{field_name}" if namespace else f"/{field_name}"
            parameter_names.append(param_name)

        return parameter_names

    def yamcs_packet_callback(self, packet: Any, config: Dict[str, Any], publisher: Publisher, MessageType: Type, parameter_names: List[str]) -> None:
        """
        Handle YAMCS packet/container updates and publish to ROS.

        Receives complete packets from YAMCS, automatically maps parameters to ROS
        message fields based on naming conventions, and publishes atomically.

        Args:
            packet: YAMCS packet/container data with parameters
            config: Bridge configuration
            publisher: ROS publisher
            MessageType: ROS message class
        """
        try:
            # Create ROS message
            msg = MessageType()

            # Query parameter values from YAMCS for this container
            try:
                # Get current parameter values
                param_values = self.processor.get_parameter_values(
                    parameters=parameter_names,
                    from_cache=True,
                    timeout=1.0
                )

                # Map parameter values to ROS message fields
                for param_name, param_value in zip(parameter_names, param_values):
                    if param_value is not None:
                        self._map_parameter_value_to_message(param_name, param_value, msg)

            except Exception as e:
                self.get_logger().error(f'Failed to query parameters: {e}')

            # Set timestamp from packet generation time if available
            if hasattr(msg, 'header'):
                if hasattr(packet, 'generation_time') and packet.generation_time:
                    # Convert YAMCS generation time to ROS time
                    gen_time = packet.generation_time
                    if isinstance(gen_time, datetime):
                        timestamp = gen_time.timestamp()
                    else:
                        timestamp = float(gen_time)

                    msg.header.stamp.sec = int(timestamp)
                    msg.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
                else:
                    # Fallback to current time
                    msg.header.stamp = self.get_clock().now().to_msg()

            # Publish the complete message
            publisher.publish(msg)

            self.get_logger().debug(
                f'Published packet to {config["ros_topic"]}'
            )

        except Exception as e:
            self.get_logger().error(
                f'Error in packet callback for {config["name"]}: {e}\n{traceback.format_exc()}'
            )

    def _map_parameter_value_to_message(self, param_name: str, param_value: Any, msg: Any) -> None:
        """
        Map a YAMCS ParameterValue to a ROS message field.

        Args:
            param_name: Full YAMCS parameter name (e.g., "/Curiosity/ccsds_packet_id")
            param_value: YAMCS ParameterValue object
            msg: ROS message to populate
        """
        # Extract the leaf name from the full parameter path
        parts = [p for p in str(param_name).strip('/').split('/') if p]
        if not parts:
            return

        leaf = parts[-1]
        # Use eng_value to get engineering/calibrated values (including enum labels as strings)
        value = param_value.eng_value if hasattr(param_value, 'eng_value') else None

        # Try nested field mapping first (e.g., Parent/Member -> parent.member)
        if len(parts) >= 3:  # e.g., /Curiosity/ccsds_packet_id/apid
            parent = parts[-2]
            dotted_name = f"{parent}.{leaf}"
            ros_path = MessageIntrospector._yamcs_member_to_ros_path(type(msg), dotted_name)
            if ros_path:
                try:
                    self._set_field_value(msg, ros_path, value, leaf)
                    return
                except Exception as e:
                    self.get_logger().debug(f'Nested mapping failed: {e}')

        # Try simple field mapping (e.g., ccsds_packet_id -> ccsds_packet_id)
        ros_field_name = MessageIntrospector.to_ros_field_name(leaf)

        if hasattr(msg, ros_field_name):
            try:
                self._set_field_value(msg, ros_field_name, value, leaf)
            except Exception as e:
                self.get_logger().error(f'Could not map {param_name}: {e}')
        else:
            self.get_logger().debug(f'Field {ros_field_name} not found in message')

    def _set_field_value(self, msg: Any, ros_path: str, value: Any, param_name: str) -> None:
        """
        Set a field value in a ROS message, handling nested fields and type conversion.

        Args:
            msg: ROS message object
            ros_path: Field path (e.g., "field" or "parent.child")
            value: Value to set
            param_name: Original parameter name for type conversion
        """
        target_msg, field_name, field_type = self._resolve_ros_field_path(msg, ros_path)

        # Handle nested message fields
        if TypeConverter.is_ros_message_type(field_type):
            nested_msg = getattr(target_msg, field_name)
            TypeConverter.populate_message(
                nested_msg,
                value,
                logger=self.get_logger(),
                root_param_name=param_name
            )
            return

        # Handle scalar fields with type conversion
        converted = TypeConverter.convert_value(
            param_name, field_type, value, logger=self.get_logger()
        )
        setattr(target_msg, field_name, converted)

    def _resolve_ros_field_path(self, msg: Any, ros_path: str):
        """
        Resolve a ROS field path like:
          - "temperature"
          - "ccsds_packet_id.apid"

        Returns:
            (target_msg, leaf_field_name, leaf_field_type_str)
        """
        segments = ros_path.split('.')
        current = msg

        for seg in segments[:-1]:
            if not hasattr(current, seg):
                raise AttributeError(f"Missing intermediate field '{seg}' on {type(current).__name__}")
            current = getattr(current, seg)

        leaf = segments[-1]
        fields_and_types = getattr(current, "_fields_and_field_types", {})
        if leaf not in fields_and_types:
            raise AttributeError(f"Missing leaf field '{leaf}' on {type(current).__name__}")

        return current, leaf, fields_and_types[leaf]

    def destroy_node(self) -> None:
        """Clean up resources."""
        # Cancel all YAMCS subscriptions
        for bridge in self.bridges:
            try:
                bridge['subscription'].cancel()
            except Exception:
                pass  # Silently ignore errors during shutdown

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
        pass  # Silently handle Ctrl+C
    except Exception as e:
        print(f'Error: {e}')
        traceback.print_exc()
        return 1
    finally:
        # Clean up node first
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass  # Ignore errors during shutdown

        # Then shutdown rclpy if still active
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass  # Ignore errors during shutdown

    return 0


if __name__ == '__main__':
    sys.exit(main())
