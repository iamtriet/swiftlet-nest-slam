#!/bin/bash

# Test YOLO Segmentation Only (NO SLAM)
# Use this to verify segmentation works before running full system

echo "=================================================="
echo "🧪 Testing YOLO Segmentation (Without SLAM)"
echo "=================================================="
echo ""
echo "This will:"
echo "  ✅ Start RealSense camera"
echo "  ✅ Start YOLO segmentation"
echo "  ✅ Show colored point clouds"
echo "  ❌ NO SLAM (for testing only)"
echo ""

# Check model
MODEL_PATH="$HOME/catkin_ws/yolov8n-seg.pt"
if [ ! -f "$MODEL_PATH" ]; then
    echo "📥 Downloading YOLOv8 segmentation model..."
    cd $HOME/catkin_ws
    python3 -c "from ultralytics import YOLO; model = YOLO('yolov8n-seg.pt')"
fi

echo "🌟 Launching segmentation test..."
source $HOME/catkin_ws/devel/setup.bash

# Kill existing
pkill -9 -f "yolo|realsense|rviz" 2>/dev/null
sleep 2

# Launch
roslaunch ssl_slam test_segmentation_only.launch

