# dragoman_fsw_sim

Flight software simulators and launch files for dragoman telemetry and command handling.

## Contents

**Simulators:**
- `curiosity_simple_simulator.py` - Curiosity rover telemetry and commands
- `imetro_demo_dummy_simulator.py` - iMetro demo with hardcoded telemetry
- `imetro_demo_robot_simple_simulator.py` - iMetro demo with real robot integration
- `all_types_simulator.py` - Demonstrates all XTCE parameter types
- `multipacket_simulator.py` - Multi-packet telemetry with different APIDs

**Launch Files:**
- `curiosity_simulation.launch.py` - Curiosity rover with Gazebo
- `imetro_dummy_demo.launch.py` - iMetro dummy demo
- `imetro_robot_simple_demo.launch.py` - iMetro robot demo
- `test_clr.launch.py` - CLR robot with MuJoCo
- `view_clr.launch.py` - CLR robot visualization with RViz

**Utilities:**
- `clr_trajectory_router.py` - CLR robot trajectory routing
- `curiosity_xtce_construct_generator.py` - Curiosity XTCE constructs
- `imetro_xtce_construct_generator.py` - iMetro XTCE constructs
- `srdf_parser.py` - SRDF file parser

See the [main README](../README.md) for detailed demo instructions.
