#!/bin/bash
# MMS_SLAM Debug Script
# This will show what's happening with topics and transforms

echo "=========================================="
echo "  MMS_SLAM Debugging Information"
echo "=========================================="
echo ""

echo "1. Checking if L515 camera is publishing..."
echo "   Point cloud topic:"
rostopic hz /camera/depth/color/points --count 5 2>&1 | head -5
echo ""

echo "2. Checking TF tree..."
rosrun tf view_frames
echo "   TF tree saved to: frames.pdf"
echo ""

echo "3. Checking what's being published to /laser_cloud_surround..."
rostopic hz /laser_cloud_surround --count 5 2>&1 | head -5
echo ""

echo "4. Checking frame IDs in point cloud..."
rostopic echo /camera/depth/color/points/header -n 1
echo ""

echo "5. Available topics:"
rostopic list | grep -E "cloud|laser|camera"
echo ""

echo "=========================================="
echo "  Debug Info Complete"
echo "=========================================="


