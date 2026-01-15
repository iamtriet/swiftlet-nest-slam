#!/bin/bash
# Start SSL_SLAM with L515 in INFRARED mode + YOLO segmentation + IMPROVED TRACKING

echo "🌙 Starting SSL_SLAM with INFRARED Camera + YOLO Segmentation + TRACKING"
echo "════════════════════════════════════════════════════════════════"
echo "📹 Camera: Intel RealSense L515 (Infrared)"
echo "🤖 AI: YOLOv8n-seg (Enhanced for IR)"
echo "🗺️  SLAM: SSL_SLAM (3D Mapping)"
echo "🎯 Tracking: Map-based persistent object tracking"
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

echo ""
echo "✅ Launching Semantic SLAM System..."
echo "� Camera: L515 Infrared Mode with Enhancement"
echo "🎯 YOLO: Custom Swiftlet Nest Detection (best.pt)"
echo "🔍 Tracking: DeepSORT + Enhanced 3D Position Memory (Kalman Filter)"
echo "🎨 Visualization: RViz with rainbow depth SLAM map"
echo ""
echo "Press Ctrl+C to stop..."
echo ""
echo "⏳ Starting system (this may take 10-15 seconds)..."
echo
echo "📝 Tracking improvements:"
echo "   • DeepSORT: Deep learning-based multi-object tracking"
echo "   • Appearance features: MobileNet CNN embeddings"
echo "   • Kalman filtering: Motion prediction"
echo "   • Hungarian algorithm: Optimal track-detection matching"
echo "   • Handles occlusions, ID switches, camera rotation"
echo "   • 60 frame persistence, 3 frame confirmation"
echo "════════════════════════════════════════════════════════════════"
echo

# Launch IR + YOLO + TRACKING configuration
roslaunch ssl_slam ssl_slam_L515_semantic_IR_YOLO_tracking.launch

# If launch exits
echo
echo "⏹️  SSL_SLAM stopped"
