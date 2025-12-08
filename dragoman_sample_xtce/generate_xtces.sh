#!/bin/bash

python3 scripts/pymdb_generate_imetro_demo_xtce.py ./xtce/IMetro.xtce
python3 scripts/pymdb_generate_curiosity_xtce.py ./xtce/Curiosity.xtce
python3 scripts/pymdb_generate_all_types.py ./xtce/AllTypes.xtce
python3 scripts/pymdb_generate_multi_packet.py ./xtce/MultiPacket.xtce
