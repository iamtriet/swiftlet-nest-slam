#!/bin/bash
# Start L515 Infrared Camera and Image Capture Tool

echo "📷 Starting L515 Infrared Camera Data Collection"
echo "════════════════════════════════════════════════════════════════"
echo "   Camera: Intel RealSense L515 (Infrared mode)"
echo "   Press 'c' to capture images"
echo "   Press 'q' to quit"
echo "════════════════════════════════════════════════════════════════"

# Check if RealSense camera is connected
if ! rs-enumerate-devices &> /dev/null; then
    echo "❌ Error: No RealSense camera detected!"
    echo "   Please connect your L515 camera and try again."
    exit 1
fi

echo "✅ RealSense camera detected"
echo ""
echo "🚀 Starting camera and capture tool..."
echo ""

# Launch RealSense camera with infrared stream only
roslaunch realsense2_camera rs_camera.launch \
    enable_color:=false \
    enable_depth:=false \
    enable_infra:=true \
    infra_width:=640 \
    infra_height:=480 \
    infra_fps:=30 &

# Wait for camera to initialize
sleep 3

# Run capture tool
cd ~/catkin_ws
python3 capture_ir_images.py

# Kill background processes
rosnode kill -a 2>/dev/null

echo ""
echo "⏹️  Data collection stopped"
