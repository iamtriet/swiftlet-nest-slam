#!/bin/bash

echo "======================================"
echo "🔍 Semantic SLAM System Diagnostics"
echo "======================================"
echo ""

# Check ROS topics
echo "📡 ROS Topics:"
echo "-----------------------------------"
if timeout 2 rostopic list | grep -E "(map|segmentation)" > /dev/null 2>&1; then
    echo "✅ SLAM Map:"
    rostopic info /map 2>/dev/null | grep "Type\|Publishers"
    echo ""
    echo "✅ Segmentation Topics:"
    rostopic list | grep segmentation
    echo ""
    rostopic info /segmentation/object_clouds 2>/dev/null | grep "Type\|Publishers"
    echo ""
    rostopic info /segmentation/boxes_3d 2>/dev/null | grep "Type\|Publishers"
else
    echo "❌ ROS topics not available. Is the system running?"
fi

echo ""
echo "🤖 Running Nodes:"
echo "-----------------------------------"
rosnode list 2>/dev/null | grep -E "(ssl_slam|yolo|segmentation|realsense)" || echo "❌ No nodes found"

echo ""
echo "📊 Topic Rates (5 second sample):"
echo "-----------------------------------"
echo "SLAM Map rate:"
timeout 5 rostopic hz /map 2>/dev/null | grep "average rate" || echo "  ⚠️  No data"

echo "Segmentation rate:"
timeout 5 rostopic hz /segmentation/object_clouds 2>/dev/null | grep "average rate" || echo "  ⚠️  No data (normal if no objects detected)"

echo ""
echo "======================================"
