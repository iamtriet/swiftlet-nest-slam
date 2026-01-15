#!/bin/bash

# Multi-modal Semantic SLAM Launcher
# Combines SSL_SLAM + YOLOv8 Segmentation for temporary object point clouds

echo "=================================================="
echo "🚀 Starting Multi-modal Semantic SLAM System"
echo "=================================================="
echo ""
echo "Features:"
echo "  ✅ SSL_SLAM for 3D mapping (permanent gray map)"
echo "  ✅ YOLOv8 Instance Segmentation (temporary colored point clouds)"
echo "  ✅ 3D Bounding Boxes for detected objects"
echo "  ✅ Point clouds appear/disappear with detections"
echo ""
echo "📦 Checking YOLO Segmentation Model..."

# Check if yolov8n-seg.pt exists
MODEL_PATH="$HOME/catkin_ws/yolov8n-seg.pt"
if [ ! -f "$MODEL_PATH" ]; then
    echo "⚠️  YOLOv8 Segmentation model not found!"
    echo "📥 Downloading yolov8n-seg.pt..."
    cd $HOME/catkin_ws
    python3 -c "from ultralytics import YOLO; model = YOLO('yolov8n-seg.pt')"
    echo "✅ Model downloaded successfully!"
else
    echo "✅ Model found: $MODEL_PATH"
fi

echo ""
echo "🔧 Checking CUDA availability..."
python3 -c "import torch; print('✅ CUDA available!' if torch.cuda.is_available() else '⚠️  CUDA not available, using CPU')"

echo ""
echo "🌟 Launching system..."
echo ""

# Source ROS workspace
source $HOME/catkin_ws/devel/setup.bash

# Kill any existing processes
echo "🧹 Cleaning up existing processes..."
pkill -9 -f "ssl_slam|yolo|realsense|rviz" 2>/dev/null
sleep 2

# Launch the system
roslaunch ssl_slam ssl_slam_L515_semantic.launch

