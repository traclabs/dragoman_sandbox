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
            # e.g. "dragoman_generated_msgs.msg._GatewayTelemetryPacket" -> "dragoman_generated_msgs"
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

    # Enum mapping for enumerated types
    ENUM_MAPPINGS = {
        'T_EnumeratedAlarm': {
            'STATE_OFF': 0,
            'STATE_NOMINAL': 1,
            'STATE_FAULT': 2
        }
    }

    _ROS_INT_TYPES = {
        'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64'
    }
    _ROS_FLOAT_TYPES = {'float32', 'float64'}

    # Common string enums that YAMCS may return for numeric CCSDS-like fields
    # (kept generic; applies when the ROS field is an integer type)
    _COMMON_STRING_TO_INT = {
        # CCSDS primary header "type" bit (common naming)
        'TM': 0,
        'TC': 1,

        # CCSDS packet sequence "sequence flags" (2-bit)
        'CONTINUATION': 0,
        'FIRST': 1,
        'LAST': 2,
        'STANDALONE': 3,
    }

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

        # Enumerated types (YAMCS often provides strings for enums)
        if isinstance(value, str) and param_name in TypeConverter.ENUM_MAPPINGS and base in TypeConverter._ROS_INT_TYPES:
            return TypeConverter._convert_enum(param_name, value, logger)

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
                s = value.strip()

                # Try well-known enum-like strings first (case-insensitive)
                if s in TypeConverter._COMMON_STRING_TO_INT:
                    return TypeConverter._COMMON_STRING_TO_INT[s]
                su = s.upper()
                if su in TypeConverter._COMMON_STRING_TO_INT:
                    return TypeConverter._COMMON_STRING_TO_INT[su]

                # Then try parsing numeric strings (supports "0x.." via base=0)
                try:
                    return int(s, 0)
                except Exception:
                    # Some values might come as "12.0" strings; allow that too
                    return int(float(s))

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

        Supports:
        - direct scalar mapping: "T_IntegerSigned" -> "t_integer_signed"
        - whole-aggregate mapping: "T_StatusAggregate" -> "t_status_aggregate" (nested msg)
        - member mapping (if YAMCS exposes aggregate members as separate parameters):
            "T_StatusAggregate.CurrentDraw" -> "t_status_aggregate.current_draw"
            ".../T_StatusAggregate/CurrentDraw" -> same (key is synthesized)
        """
        full_name = getattr(parameter, "name", "")
        parts = [p for p in str(full_name).strip('/').split('/') if p]

        leaf = parts[-1] if parts else str(full_name)
        dotted = f"{parts[-2]}.{parts[-1]}" if len(parts) >= 2 else None

        self.get_logger().debug(
            f'Mapping parameter: {full_name} (leaf: {leaf}, value type: {type(parameter.eng_value).__name__})'
        )

        # Try multiple keys for robustness across YAMCS naming styles
        mapping_key = None
        for candidate in (dotted, leaf, full_name):
            if candidate and candidate in field_mapping:
                mapping_key = candidate
                break

        if mapping_key is None:
            return

        ros_path = field_mapping[mapping_key]

        try:
            target_msg, field_name, field_type = self._resolve_ros_field_path(msg, ros_path)

            # Whole-aggregate or nested member field
            if TypeConverter.is_ros_message_type(field_type):
                nested_msg = getattr(target_msg, field_name)
                TypeConverter.populate_message(
                    nested_msg,
                    parameter.eng_value,
                    logger=self.get_logger(),
                    root_param_name=leaf
                )
                self.get_logger().debug(
                    f'Mapped nested {mapping_key} → {ros_path}'
                )
                return

            converted = TypeConverter.convert_value(
                leaf, field_type, parameter.eng_value, logger=self.get_logger()
            )
            setattr(target_msg, field_name, converted)
            self.get_logger().debug(
                f'Mapped {mapping_key} → {ros_path}: {type(converted).__name__}'
            )
        except Exception as e:
            self.get_logger().error(
                f'Error mapping parameter {mapping_key} → {ros_path}: {e}\n{traceback.format_exc()}'
            )

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
