#!/bin/bash
# Start SSL_SLAM with L515 in INFRARED mode (for dark environments)

echo "🌙 Starting SSL_SLAM with INFRARED Camera"
echo "════════════════════════════════════════════════════════════════"
echo "📹 Camera: Intel RealSense L515 (Infrared)"
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
pkill -9 -f rviz 2>/dev/null
sleep 3
echo

echo "🚀 Launching SSL_SLAM with IR camera..."
echo "════════════════════════════════════════════════════════════════"
echo "📊 Features:"
echo "   • Infrared camera for dark environments"
echo "   • 3D point cloud mapping"
echo "   • Real-time SLAM"
echo "   • Works in complete darkness"
echo "════════════════════════════════════════════════════════════════"
echo
echo "⏳ Starting system (this may take 10-15 seconds)..."
echo

# Launch IR configuration (without YOLO)
roslaunch ssl_slam ssl_slam_L515_semantic_IR.launch

# If launch exits
echo
echo "⏹️  SSL_SLAM stopped"
