# dragoman_fsw_sim

Flight software simulators for YAMCS telemetry and command handling.

## Simulators

**Standalone (No ROS):**
- `all_types_simulator.py` - All XTCE parameter types
- `multipacket_simulator.py` - Multi-APID telemetry
- `imetro_demo_dummy_simulator.py` - Mock iMetro telemetry

**ROS2-Integrated:**
- `curiosity_simple_simulator.py` - Curiosity rover
- `imetro_demo_robot_simple_simulator.py` - iMetro robot

**Utilities:**
- `srdf_parser.py` - SRDF parser
- `*_construct_definitions.py` - CCSDS packet structures
- `clr_trajectory_router.py` - CLR trajectory routing

## Usage

**Standalone Simulators:**
```bash
$(ros2 pkg prefix dragoman_fsw_sim --share)/scripts/all_types_simulator.py
$(ros2 pkg prefix dragoman_fsw_sim --share)/scripts/multipacket_simulator.py
$(ros2 pkg prefix dragoman_fsw_sim --share)/scripts/imetro_demo_dummy_simulator.py
```

**ROS2 Launches:**
```bash
ros2 launch dragoman_fsw_sim curiosity_simulation.launch.py
ros2 launch dragoman_fsw_sim imetro_robot_simple_demo.launch.py
```

All simulators support command-line options. Use `--help` to see available options.

See [main README](../README.md) for details.
