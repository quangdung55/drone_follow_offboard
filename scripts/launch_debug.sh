#!/bin/bash
# Launch Smart Follow Node with DEBUG logging for troubleshooting

echo "========================================="
echo "Smart Follow Node - DEBUG Mode"
echo "========================================="
echo "Log level: DEBUG"
echo "Use this to diagnose why drone is not following"
echo "========================================="
echo ""

# Source ROS 2 workspace
source /home/xb/ardu_ws/install/setup.bash

# Launch with DEBUG log level
ros2 run drone_offboard smart_follow_node --ros-args \
    -p follow_dist:=8.0 \
    -p follow_height:=5.0 \
    -p terrain_follow_enable:=false \
    -p adaptive_distance_enable:=true \
    -p filter_enable:=true \
    --log-level debug

echo ""
echo "Debug session ended."
