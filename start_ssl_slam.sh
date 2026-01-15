#!/bin/bash

# SSL_SLAM Quick Start Script
# Author: Setup Assistant
# Date: October 31, 2025

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  SSL_SLAM with Intel RealSense L515${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if camera is connected
echo -e "${YELLOW}[1/3]${NC} Checking for Intel RealSense L515..."
if rs-enumerate-devices 2>&1 | grep -q "L515"; then
    echo -e "${GREEN}✓ Intel RealSense L515 detected!${NC}"
    SERIAL=$(rs-enumerate-devices 2>&1 | grep "Serial Number" | awk '{print $4}')
    echo -e "   Serial Number: ${SERIAL}"
else
    echo -e "${RED}✗ Intel RealSense L515 not detected!${NC}"
    echo -e "   Please connect your camera and try again."
    exit 1
fi

# Source the workspace
echo ""
echo -e "${YELLOW}[2/3]${NC} Setting up ROS environment..."
cd /home/iamtriet/catkin_ws
source devel/setup.bash

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Workspace sourced successfully${NC}"
else
    echo -e "${RED}✗ Failed to source workspace${NC}"
    exit 1
fi

# Launch SSL_SLAM
echo ""
echo -e "${YELLOW}[3/3]${NC} Launching SSL_SLAM with L515 live data..."
echo ""
echo -e "${GREEN}Starting in 3 seconds...${NC}"
echo -e "Press ${RED}Ctrl+C${NC} to abort"
sleep 3

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Launching ROS...${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

roslaunch ssl_slam ssl_slam_L515.launch

