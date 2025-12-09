#!/bin/bash

XTCE_DIR=$(ros2 pkg prefix --share dragoman_sample_xtce)/xtce
MSG_DIR=$(ros2 pkg prefix --share dragoman_generated_msgs)/msg

echo "========================================"
echo "  XTCE to ROS2 Message Generator"
echo "========================================"
echo ""

echo "→ Processing IMetro.xtce..."
ros2 run dragoman_xtce2msg xtce2msg ${XTCE_DIR}/IMetro.xtce ${MSG_DIR}
echo ""

echo "→ Processing Curiosity.xtce..."
ros2 run dragoman_xtce2msg xtce2msg ${XTCE_DIR}/Curiosity.xtce ${MSG_DIR}
echo ""

echo "→ Processing AllTypes.xtce..."
ros2 run dragoman_xtce2msg xtce2msg ${XTCE_DIR}/AllTypes.xtce ${MSG_DIR}
echo ""

echo "→ Processing MultiPacket.xtce..."
ros2 run dragoman_xtce2msg xtce2msg ${XTCE_DIR}/MultiPacket.xtce ${MSG_DIR}
echo ""

echo "========================================"
echo "  ✓ All XTCE files processed"
echo "========================================"
