#!/bin/bash

SCRIPT_DIR=$(dirname "$(realpath "$0")")
XTCE_DIR=${SCRIPT_DIR}/../xtce

echo "========================================"
echo "  XTCE Generator"
echo "========================================"
echo ""

echo "→ Generating CCSDSHeader.xtce..."
ros2 run dragoman_sample_xtce pymdb_generate_ccsds_header.py ${XTCE_DIR}/CCSDSHeader.xtce
echo ""

echo "→ Generating Cfs.xtce..."
ros2 run dragoman_sample_xtce pymdb_generate_cfs.py ${XTCE_DIR}/Cfs.xtce
echo ""

echo "→ Generating IMetro.xtce..."
ros2 run dragoman_sample_xtce pymdb_generate_imetro_demo_xtce.py ${XTCE_DIR}/IMetro.xtce
echo ""

echo "→ Generating Curiosity.xtce..."
ros2 run dragoman_sample_xtce pymdb_generate_curiosity_xtce.py ${XTCE_DIR}/Curiosity.xtce
echo ""

echo "→ Generating Gateway.xtce..."
ros2 run dragoman_sample_xtce pymdb_generate_gateway_demo_xtce.py ${XTCE_DIR}/Gateway.xtce
echo ""

echo "→ Generating AllTypes.xtce..."
ros2 run dragoman_sample_xtce pymdb_generate_all_types.py ${XTCE_DIR}/AllTypes.xtce
echo ""

echo "→ Generating MultiPacket.xtce..."
ros2 run dragoman_sample_xtce pymdb_generate_multi_packet.py ${XTCE_DIR}/MultiPacket.xtce
echo ""

echo "========================================"
echo "  ✓ All XTCE files generated"
echo "========================================"
