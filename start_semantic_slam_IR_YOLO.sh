#!/bin/bash
# Start SSL_SLAM with L515 in INFRARED mode + YOLO segmentation

echo "🌙 Starting SSL_SLAM with INFRARED Camera + YOLO Segmentation"
echo "════════════════════════════════════════════════════════════════"
echo "📹 Camera: Intel RealSense L515 (Infrared)"
echo "🤖 AI: YOLOv8n-seg (Enhanced for IR)"
echo "🗺️  SLAM: SSL_SLAM (3D Mapping)"
echo "💡 Perfect for dark bird house environments"
echo "════════════════════════════════════════════════════════════════"
echo

# Check if ROS is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS not sourced. Sourcing workspace..."
    source /opt/ros/noetic/setup.bash
    source devel/setup.bash
fi

# Check if camera is connected
echo "🔍 Checking for RealSense camera..."
if ! lsusb | grep -q "Intel"; then
    echo "❌ Error: RealSense camera not detected!"
    echo "   Please connect the L515 camera and try again."
    exit 1
fi
echo "✅ RealSense camera detected"
echo

# Kill any existing nodes
echo "🧹 Cleaning up old nodes..."
rosnode kill -a 2>/dev/null
pkill -9 -f ssl_slam 2>/dev/null
pkill -9 -f realsense2_camera 2>/dev/null
pkill -9 -f yolo 2>/dev/null
pkill -9 -f rviz 2>/dev/null
sleep 3
echo

echo "🚀 Launching SSL_SLAM with IR camera + YOLO detection..."
echo "════════════════════════════════════════════════════════════════"
echo "📊 Features:"
echo "   • Infrared camera for dark environments"
echo "   • YOLOv8 segmentation with IR enhancement (CLAHE + gamma)"
echo "   • 3D point cloud mapping (white visualization)"
echo "   • 3D bounding boxes for detected objects"
echo "   • Real-time SLAM"
echo "   • Works in complete darkness"
echo "════════════════════════════════════════════════════════════════"
echo
echo "⏳ Starting system (this may take 10-15 seconds)..."
echo
echo "📝 Note: Currently using default YOLOv8n-seg model."
echo "   To use your custom trained swiftlet nest model:"
echo "   1. Train model using train_swiftlet_model.py"
echo "   2. Replace model in launch file:"
echo "      <param name=\"model\" value=\"your_model_best.pt\"/>"
echo "════════════════════════════════════════════════════════════════"
echo

# Launch IR + YOLO configuration
roslaunch ssl_slam ssl_slam_L515_semantic_IR_YOLO.launch

# If launch exits
echo
echo "⏹️  SSL_SLAM stopped"
