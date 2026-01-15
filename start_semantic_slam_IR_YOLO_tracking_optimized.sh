#!/bin/bash
# Start SSL_SLAM with L515 in INFRARED mode + YOLO segmentation + OPTIMIZED TRACKING

echo "⚡ Starting OPTIMIZED SSL_SLAM with INFRARED Camera + YOLO + TRACKING"
echo "════════════════════════════════════════════════════════════════"
echo "📹 Camera: Intel RealSense L515 (Infrared)"
echo "🤖 AI: YOLOv8 (Enhanced for IR)"
echo "🗺️  SLAM: SSL_SLAM (3D Mapping)"
echo "🎯 Tracking: DeepSORT + Global Object Memory"
echo "⚡ Performance: VECTORIZED + OPTIMIZED"
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
echo "✅ Launching OPTIMIZED Semantic SLAM System..."
echo "📹 Camera: L515 Infrared Mode with Enhancement"
echo "🎯 YOLO: Custom Swiftlet Nest Detection (best.pt)"
echo "🔍 Tracking: DeepSORT + Global Memory"
echo "⚡ Optimizations:"
echo "   • Vectorized point cloud extraction (3-5x faster)"
echo "   • Optimized bbox calculation"
echo "   • Reduced memory allocations"
echo "   • Smart batch processing"
echo "   • Optional frame skipping"
echo ""
echo "📊 Expected Performance Gain: 20-40% faster FPS"
echo ""
echo "Press Ctrl+C to stop..."
echo ""
echo "════════════════════════════════════════════════════════════════"
echo

# Launch OPTIMIZED IR + YOLO + TRACKING configuration
roslaunch ssl_slam ssl_slam_L515_semantic_IR_YOLO_tracking_optimized.launch

# If launch exits
echo
echo "⏹️  Optimized SSL_SLAM stopped"
