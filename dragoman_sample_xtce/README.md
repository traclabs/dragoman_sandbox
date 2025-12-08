# dragoman_sample_xtce

Sample XTCE generation scripts for dragoman telemetry definitions.

## Overview

This package contains Python scripts that generate XTCE (XML Telemetric and Command Exchange) files for various dragoman demonstrations and test scenarios. These XTCE files define the telemetry and command structures used by YAMCS.

## Generate XTCE Files

## Batch Generation

Use the provided shell script to generate all XTCE files at once:

```bash
./generate_xtces.sh
```

If you don't want to run the script, you can generate each XTCE file individually as described below.

### Generate iMetro Demo XTCE

Generate the XTCE file used for the iMetro demo:

```bash
ros2 run dragoman_sample_xtce pymdb_generate_imetro_demo_xtce.py
```

This will generate the XTCE file in the `install/share/dragoman_sample_xtce/xtce` directory.

**Note:** For the XTCE to be used by YAMCS, you'll need to copy it to the appropriate YAMCS project directory (e.g., `dragoman_yamcs_project/src/main/yamcs/mdb`).

### Generate Curiosity XTCE

Generate the XTCE file for Curiosity rover telemetry:

```bash
ros2 run dragoman_sample_xtce pymdb_generate_curiosity_xtce.py
```

### Generate All Types XTCE

Generate an XTCE file demonstrating all supported parameter types:

```bash
ros2 run dragoman_sample_xtce pymdb_generate_all_types.py
```

### Generate Multi-Packet XTCE

Generate an XTCE file with multiple packet definitions (different APIDs):

```bash
ros2 run dragoman_sample_xtce pymdb_generate_multi_packet.py
```

## Output Location

Generated XTCE files are placed in xtce/, and can be installed by running:
```bash
colcon build --packages-select dragoman_sample_xtce
```

The installed files will be located in `install/share/dragoman_sample_xtce/xtce/`.

## Installing the XTCE files

After generating XTCE files, run the following command to install them:

```bash
colcon build --packages-select dragoman_sample_xtce
```

## Using Generated XTCE Files in YAMCS

To use the generated XTCE files in a YAMCS project, update the YAMCS configuration to point to the installed XTCE files. For example, in `dragoman_yamcs_project/dragoman_yamcs_project/src/main/yamcs/etc/yamcs.dragoman.yaml`, set the file paths to:

```bash
mdb:
  - type: xtce
    args:
      file: /dragoman/install/dragoman_sample_xtce/share/dragoman_sample_xtce/xtce/AllTypes.xtce
  - type: xtce
    args:
      file: /dragoman/install/dragoman_sample_xtce/share/dragoman_sample_xtce/xtce/Curiosity.xtce
  - type: xtce
    args:
      file: /dragoman/install/dragoman_sample_xtce/share/dragoman_sample_xtce/xtce/IMetro.xtce
  - type: xtce
    args:
      file: /dragoman/install/dragoman_sample_xtce/share/dragoman_sample_xtce/xtce/MultiPacket.xtce
```

To install the YAMCS project, run:

```bash
colcon build --packages-select dragoman_yamcs_project
```

## Dependencies

- `rclpy` - ROS2 Python client library
- `pymdb` - Python library for generating XTCE files (Mission Database)
