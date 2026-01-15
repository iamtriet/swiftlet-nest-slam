#!/bin/bash

echo "============================================"
echo "🔍 Checking TF Transforms for Segmentation"
echo "============================================"
echo ""

# Check if ROS is running
if ! rostopic list > /dev/null 2>&1; then
    echo "❌ ROS is not running!"
    exit 1
fi

echo "✅ ROS is running"
echo ""

# Check available frames
echo "📡 Available TF frames:"
echo "───────────────────────"
timeout 2 rosrun tf tf_echo map camera_color_optical_frame 2>&1 | grep -E "(At time|Waiting)" | head -5

echo ""
echo "🔗 Checking transform chain:"
echo "───────────────────────────"

# Check map -> base_link
echo -n "map -> base_link: "
if timeout 2 rosrun tf tf_echo map base_link 2>&1 | grep -q "At time"; then
    echo "✅ OK"
else
    echo "❌ MISSING"
fi

# Check base_link -> camera_link
echo -n "base_link -> camera_link: "
if timeout 2 rosrun tf tf_echo base_link camera_link 2>&1 | grep -q "At time"; then
    echo "✅ OK"
else
    echo "❌ MISSING (needed for SLAM mode)"
fi

# Check camera_link -> camera_color_optical_frame
echo -n "camera_link -> camera_color_optical_frame: "
if timeout 2 rosrun tf tf_echo camera_link camera_color_optical_frame 2>&1 | grep -q "At time"; then
    echo "✅ OK"
else
    echo "❌ MISSING"
fi

# Check full chain
echo ""
echo -n "Full chain (map -> camera_color_optical_frame): "
if timeout 2 rosrun tf tf_echo map camera_color_optical_frame 2>&1 | grep -q "At time"; then
    echo "✅ OK - Segmentation should work!"
    echo ""
    timeout 2 rosrun tf tf_echo map camera_color_optical_frame 2>&1 | head -10
else
    echo "❌ BROKEN - Segmentation won't work!"
    echo ""
    echo "💡 Solution: Make sure static transform is published:"
    echo "   rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link camera_link"
fi

echo ""
echo "============================================"
