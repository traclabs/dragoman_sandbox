#!/bin/bash

SCRIPT_DIR=$(dirname "$(realpath "$0")")
# Output to dragoman_sample_msgs/xtce directory (source space)
OUTPUT_DIR=${SCRIPT_DIR}/../xtce

echo "========================================"
echo "  XTCE Generator"
echo "========================================"
echo ""
echo "Output directory: ${OUTPUT_DIR}"
echo ""

# Create output directory if it doesn't exist
mkdir -p ${OUTPUT_DIR}

echo "→ Generating IMetro.xtce..."
${SCRIPT_DIR}/pymdb_generate_imetro_demo_xtce.py ${OUTPUT_DIR}/IMetro.xtce
echo ""

echo "→ Generating Curiosity.xtce..."
${SCRIPT_DIR}/pymdb_generate_curiosity_xtce.py ${OUTPUT_DIR}/Curiosity.xtce
echo ""

echo "→ Generating Gateway.xtce..."
${SCRIPT_DIR}/pymdb_generate_gateway_demo_xtce.py ${OUTPUT_DIR}/Gateway.xtce
echo ""

echo "→ Generating MobileServicingSystem.xtce..."
${SCRIPT_DIR}/pymdb_generate_mobile_servicing_system_demo_xtce.py ${OUTPUT_DIR}/MobileServicingSystem.xtce
echo ""

echo "→ Generating LunarExploration.xtce..."
${SCRIPT_DIR}/pymdb_generate_lunar_exploration_demo_xtce.py ${OUTPUT_DIR}/LunarExploration.xtce
echo ""


echo "→ Generating AllTypes.xtce..."
${SCRIPT_DIR}/pymdb_generate_all_types.py ${OUTPUT_DIR}/AllTypes.xtce
echo ""

echo "→ Generating MultiPacket.xtce..."
${SCRIPT_DIR}/pymdb_generate_multi_packet.py ${OUTPUT_DIR}/MultiPacket.xtce
echo ""

echo "========================================"
echo "  ✓ All XTCE files generated"
echo "  Location: ${OUTPUT_DIR}"
echo "========================================"
