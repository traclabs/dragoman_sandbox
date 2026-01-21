# dragoman_xtce2msg

Converts XTCE (XML Telemetric and Command Exchange) telemetry definitions into compiled ROS 2 message types.

## Quick Start

### CMakeLists.txt

```cmake
find_package(ament_cmake REQUIRED)
find_package(dragoman_xtce2msg REQUIRED)

xtce_generate_messages(
  FILES
    xtce/MySpacecraft.xtce
)

ament_package()
```

### package.xml

**Required dependencies** for packages using `xtce_generate_messages()`:

```xml
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>dragoman_xtce2msg</depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
```

These dependencies are required for any ROS 2 message package and cannot be inherited from `dragoman_xtce2msg`.

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
| AggregateParameterType | Separate message type |
| AbsoluteTimeParameterType | builtin_interfaces/Time |
| RelativeTimeParameterType | builtin_interfaces/Duration |

\* Aggregates (structs) are separated into individual message types.
