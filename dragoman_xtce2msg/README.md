# dragoman_xtce2msg

Converts XTCE (XML Telemetric and Command Exchange) telemetry definitions into ROS 2 message files.

## Usage

### Batch Generation
```bash
ros2 run dragoman_xtce2msg generate_msgs.sh
```

**Output Location:** Message files are generated in the `dragoman_generated_msgs` package's `msg/` directory:
```
$(ros2 pkg prefix --share dragoman_generated_msgs)/msg/
```

### Single File Conversion
```bash
ros2 run dragoman_xtce2msg xtce_to_ros <input_xtce_file.xml> <output_directory>
```

## Features

- Generates separate `.msg` files for each non-abstract XTCE container
- Resolves XTCE parameter types to ROS 2 primitives
- Flattens aggregate types into individual fields
- Handles fixed and variable-length arrays
- Converts all field names to snake_case (e.g., `MyParameter` → `my_parameter`)
- Supports CCSDS packet headers

## Supported XTCE Types

| XTCE Type | ROS 2 Type |
|-----------|------------|
| IntegerParameterType | int8, int16, int32, int64 |
| EnumeratedParameterType | uint8, uint16, uint32, uint64 |
| FloatParameterType | float32, float64 |
| BooleanParameterType | bool |
| StringParameterType | string, string[N] |
| BinaryParameterType | uint8[N], uint8[] |
| ArrayParameterType | type[N] |
| AggregateParameterType | Individual fields* |
| AbsoluteTimeParameterType | builtin_interfaces/Time |
| RelativeTimeParameterType | builtin_interfaces/Duration |

\* Aggregates (structs) are flattened into individual fields with prefixed names. Example: `StatusData` with members `voltage` and `current` becomes `status_data_voltage` and `status_data_current`.
