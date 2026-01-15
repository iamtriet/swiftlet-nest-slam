#!/bin/bash
# Quick status check for MMS_SLAM
# Run this while MMS_SLAM is running

source ~/catkin_ws/devel/setup.bash

echo "=========================================="
echo "MMS_SLAM Status Check"
echo "=========================================="
echo ""

echo "1. Checking ROS nodes..."
rosnode list | grep mms_slam

echo ""
echo "2. Checking key topics..."
echo "   /map:"
rostopic info /map

echo ""
echo "   /odom:"
rostopic info /odom

echo ""
echo "   /filtered_points_static:"
rostopic info /filtered_points_static

echo ""
echo "3. Checking TF frames..."
rosrun tf tf_echo map camera_depth_optical_frame

echo ""
echo "=========================================="

