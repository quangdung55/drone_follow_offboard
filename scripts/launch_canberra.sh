#!/bin/bash
# Launch Smart Follow Node with location-specific altitude limits
# Location: Canberra, Australia (approximate elevation: 584m MSL)

# Calculate safe MSL limit based on terrain
TERRAIN_ELEVATION_MSL=584  # meters above sea level
MAX_AGL=120                 # maximum altitude above ground (FAA limit)
MAX_MSL=$((TERRAIN_ELEVATION_MSL + MAX_AGL))  # 584 + 120 = 704m

echo "========================================="
echo "Smart Follow Node - Canberra Launch"
echo "========================================="
echo "Terrain Elevation: ${TERRAIN_ELEVATION_MSL}m MSL"
echo "Max AGL Altitude:  ${MAX_AGL}m"
echo "Max MSL Altitude:  ${MAX_MSL}m"
echo "========================================="
echo ""

# Source ROS 2 workspace
source /home/xb/ardu_ws/install/setup.bash

# Launch with proper altitude limits
ros2 run drone_offboard smart_follow_node --ros-args \
    -p follow_dist:=8.0 \
    -p follow_height:=5.0 \
    -p terrain_follow_enable:=false \
    -p adaptive_distance_enable:=true \
    -p filter_enable:=true \
    -p heading_blend_enable:=false \
    -p jitter_correction_enable:=true

echo ""
echo "Node stopped. Check diagnostics:"
echo "  ros2 topic echo /smart_follow/diagnostics"
