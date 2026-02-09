# dragoman_sample_msgs

ROS 2 messages auto-generated from XTCE telemetry specifications.

## Usage

**Python:**
```python
from dragoman_sample_msgs.msg import IMetroTelemetryPacket
```

**C++:**
```cpp
#include "dragoman_sample_msgs/msg/imetro_telemetry_packet.hpp"
```

## Available Messages

- **IMetro.xtce** → IMetroTelemetryPacket
- **Gateway.xtce** → GatewayTelemetryPacket
- **Curiosity.xtce** → CuriosityTelemetryPacket
- **AllTypes.xtce** → AllTelemetryPacket
- **MultiPacket.xtce** → TemperaturePacket, VoltagePacket
- **MobileServicingSytem.xtce** → MobileServicingSystemPacket
- **LunarExploration.xtce** → LunarExplorationPacket

## Updating Messages

```bash
# 1. Modify XTCE generation scripts in scripts/ directory
# 2. Regenerate XTCE files
./rosws/src/dragoman_sandbox/dragoman_sample_msgs/scripts/generate_xtces.sh

# 3. Rebuild
colcon build
```

## XTCE Generation Scripts

The `scripts/` directory contains Python scripts to generate sample XTCE files:

- `generate_xtces.sh` - Generate all XTCE files at once
- `pymdb_generate_imetro_demo_xtce.py` - Generate IMetro.xtce
- `pymdb_generate_curiosity_xtce.py` - Generate Curiosity.xtce
- `pymdb_generate_gateway_demo_xtce.py` - Generate Gateway.xtce
- `pymdb_generate_all_types.py` - Generate AllTypes.xtce
- `pymdb_generate_multi_packet.py` - Generate MultiPacket.xtce
- `cfs_msg_hdr.py` - Helper module for cFS message headers
