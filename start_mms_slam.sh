#!/bin/bash
# Quick start script for MMS_SLAM
# Author: AI Assistant
# Date: 2025-11-01

echo "========================================"
echo "    Starting MMS_SLAM System"
echo "========================================"
echo ""
echo "Features enabled:"
echo "  ✅ 3D SLAM mapping"
echo "  ✅ Real-time odometry"
echo "  ✅ Intel RealSense L515 live data"
echo "  ✅ RViz visualization"
echo ""
echo "Features disabled (require mmdet):"
echo "  ❌ Object detection"
echo "  ❌ Dynamic object filtering"
echo ""
echo "Starting in 3 seconds..."
sleep 3

cd ~/catkin_ws
source devel/setup.bash
roslaunch mms_slam mms_slam_mapping.launch

