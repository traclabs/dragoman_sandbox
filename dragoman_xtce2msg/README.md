# dragoman_xtce2msg

XTCE to ROS 2 Message Converter - A tool for converting XTCE (XML Telemetric and Command Exchange) telemetry definitions into ROS 2 message files.

## Overview

This package provides utilities to parse XTCE XML files and automatically generate corresponding ROS 2 `.msg` files. It handles various XTCE data types including integers, floats, strings, arrays, aggregates, and time types, mapping them to appropriate ROS 2 message field types.

## Features

- **Container-based Generation**: Generates separate message files for each non-abstract XTCE container
- **Type Resolution**: Automatically resolves XTCE parameter types to ROS 2 primitives
- **Aggregate Flattening**: Flattens aggregate types into individual message fields
- **Array Support**: Handles fixed and variable-length arrays
- **CCSDS Header Handling**: Recognizes and optionally includes CCSDS packet headers
- **ROS 2 Compliance**: Ensures all field names comply with ROS 2 naming conventions

## Usage

### Batch Generation

Use the provided shell script to generate all ROS 2 messages from XTCE files at once:

```bash
ros2 run dragoman_xtce2msg generate_msgs.sh
```

This script will automatically:
- Locate the installed XTCE files from `dragoman_sample_xtce`
- Locate the message output directory in `dragoman_generated_msgs`
- Generate ROS 2 message files for all XTCE definitions
- Provide progress feedback during generation

### Command Line (Individual Files)

Convert a single XTCE file to ROS 2 messages:

```bash
ros2 run dragoman_xtce2msg xtce_to_ros <input_xtce_file.xml> <output_directory>
```

Example:
```bash
ros2 run dragoman_xtce2msg xtce_to_ros telemetry.xml ./msg/
```

This will:
1. Parse the XTCE file
2. Extract all containers and parameters
3. Generate a `.msg` file for each non-abstract container
4. Place all generated files in the specified output directory

### Programmatic Usage

You can also use the package modules directly in your Python code:

```python
from dragoman_xtce2msg import parse_xtce_file, parse_containers, generate_ros_msg
import xml.etree.ElementTree as ET

# Parse XTCE file
tree = ET.parse('telemetry.xml')
root = tree.getroot()

# Extract metadata
parameter_metadata, type_definitions = parse_xtce_file(root)
containers = parse_containers(root)

# Generate messages programmatically
# ... (see xtce_to_ros.py for full example)
```

## Package Structure

```
dragoman_xtce2msg/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package manifest
├── README.md                   # This file
├── dragoman_xtce2msg/         # Python package
│   ├── __init__.py            # Package exports
│   ├── xtce_model.py          # XTCE parsing and type resolution
│   ├── ros_generator.py       # ROS message file generation
│   └── xtce_to_ros.py         # Main conversion logic
└── scripts/
    └── xtce_to_ros            # Executable entry point
```

## Supported XTCE Types

| XTCE Type | ROS 2 Type | Notes |
|-----------|------------|-------|
| IntegerParameterType | int8, int16, int32, int64 | Based on bit size |
| EnumeratedParameterType | uint8, uint16, uint32, uint64 | Based on bit size |
| FloatParameterType | float32, float64 | Based on bit size |
| BooleanParameterType | bool | |
| StringParameterType | string, string[N] | Fixed or variable length |
| BinaryParameterType | uint8[N], uint8[] | Fixed or variable length |
| ArrayParameterType | type[N] | Fixed-size arrays |
| AggregateParameterType | Flattened fields | Members become individual fields |
| AbsoluteTimeParameterType | builtin_interfaces/Time | |
| RelativeTimeParameterType | builtin_interfaces/Duration | |

## Field Name Conversion

The tool automatically converts XTCE parameter names to ROS 2 compliant field names:

- Converts PascalCase/CamelCase to snake_case
- Ensures names start with a lowercase letter
- Removes invalid characters
- Prevents double underscores and trailing underscores

Examples:
- `T_AbsoluteTime` → `t_absolute_time`
- `StatusAggregate_CurrentDraw` → `status_aggregate_current_draw`

## Migration Notes

This package was created by extracting XTCE conversion functionality from the `dragoman_sandbox` package. The following files were moved:

- `xtce_to_ros.py` - Main conversion script
- `xtce_model.py` - XTCE parsing and type resolution
- `ros_generator.py` - ROS message file generation

The functionality remains identical, but imports have been updated to use the new package structure.

## Dependencies

- Python 3
- ROS 2 (rclpy)
- Standard Python libraries: xml.etree.ElementTree, datetime, sys, os, re

## License

TODO

## Maintainer

TRACLabs Robotics <robotics@traclabs.com>
