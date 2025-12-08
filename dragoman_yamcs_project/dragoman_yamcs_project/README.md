# Dragoman YAMCS Project

YAMCS (Yet Another Mission Control System) server configurations and XTCE definitions for dragoman telemetry and command handling.

## Overview

This package contains YAMCS project configurations for different mission scenarios:
- **imetro** - iMetro robot demonstration configuration
- **curiosity** - Curiosity rover simulation configuration

## Build

Compile the YAMCS project:

```bash
cd src/dragoman_sandbox/dragoman_yamcs_project/dragoman_yamcs_project
mvn compile
```

## Run YAMCS Server

### Run iMetro YAMCS

Start the YAMCS server with iMetro configuration:

```bash
ros2 run dragoman_sandbox imetro_yamcs
```

Or manually:
```bash
cd src/dragoman_sandbox/dragoman_yamcs_project/dragoman_yamcs_project
mvn yamcs:run -Dinstance=imetro
```

### Run Curiosity YAMCS

Start the YAMCS server with Curiosity configuration:

```bash
ros2 run dragoman_sandbox curiosity_yamcs
```

Or manually:
```bash
cd src/dragoman_sandbox/dragoman_yamcs_project/dragoman_yamcs_project
mvn yamcs:run -Dinstance=curiosity
```

## Access YAMCS Web Interface

After starting YAMCS, open a web browser and navigate to:

```
http://localhost:8090/
```

You should see the YAMCS Mission Control interface with the selected instance loaded.

## XTCE Files

XTCE (XML Telemetric and Command Exchange) files define the telemetry and command structures. They are located in:

```
src/main/yamcs/mdb/
```

To update XTCE files, generate them using [`dragoman_sample_xtce`](../../dragoman_sample_xtce/README.md) and copy them to the MDB directory.

## Dependencies

- Maven 3.x
- Java 11 or higher
- YAMCS server libraries (managed by Maven)
