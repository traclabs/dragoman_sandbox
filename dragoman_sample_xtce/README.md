# dragoman_sample_xtce

Python scripts to generate XTCE files for dragoman telemetry definitions.

## Generate All XTCE Files

Generates to `xtce/` directory:

```bash
ros2 run dragoman_sample_xtce generate_xtces.sh
```

## Generate Individual Files

```bash
# iMetro demo
ros2 run dragoman_sample_xtce pymdb_generate_imetro_demo_xtce.py

# Curiosity rover
ros2 run dragoman_sample_xtce pymdb_generate_curiosity_xtce.py

# All parameter types
ros2 run dragoman_sample_xtce pymdb_generate_all_types.py

# Multiple packets
ros2 run dragoman_sample_xtce pymdb_generate_multi_packet.py
```

## Install

After generating, build the package to install the XTCE files:

```bash
colcon build --packages-select dragoman_sample_xtce
```
