# Dragoman YAMCS Project

YAMCS (Yet Another Mission Control System) server configurations and XTCE definitions for dragoman telemetry and command handling.

## Run YAMCS Server

Start the YAMCS server:

```bash
ros2 run dragoman_sandbox dragoman_yamcs
```

Access the web interface at [`http://localhost:8090/`](http://localhost:8090/)

## XTCE Files

XTCE files define telemetry and command structures. Generate new files using [`dragoman_sample_xtce`](../../dragoman_sample_xtce/README.md), then manually add them to the `mdb` section in [`yamcs.dragoman.yaml`](src/main/yamcs/etc/yamcs.dragoman.yaml).
