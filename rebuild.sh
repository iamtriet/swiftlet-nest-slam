#!/bin/bash

# SSL_SLAM Rebuild Script
# Quick rebuild after code modifications

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  SSL_SLAM Rebuild Script${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Navigate to workspace
cd /home/iamtriet/catkin_ws

# Option to clean build
read -p "Clean build? (removes build/ and devel/) [y/N]: " clean_build
if [[ $clean_build =~ ^[Yy]$ ]]; then
    echo -e "${YELLOW}Cleaning build directories...${NC}"
    rm -rf build devel
    echo -e "${GREEN}✓ Clean complete${NC}"
fi

# Source ROS
echo ""
echo -e "${YELLOW}Sourcing ROS Noetic...${NC}"
source /opt/ros/noetic/setup.bash

# Build
echo ""
echo -e "${YELLOW}Building workspace...${NC}"
echo -e "${BLUE}----------------------------------------${NC}"

catkin_make -j4

if [ $? -eq 0 ]; then
    echo ""
    echo -e "${BLUE}----------------------------------------${NC}"
    echo -e "${GREEN}✓ Build successful!${NC}"
    echo ""
    
    # Source the workspace
    source devel/setup.bash
    echo -e "${GREEN}✓ Workspace sourced${NC}"
    echo ""
    
    echo -e "${YELLOW}You can now run:${NC}"
    echo -e "  ${BLUE}roslaunch ssl_slam ssl_slam_L515.launch${NC}"
    echo -e ""
    echo -e "Or use the quick start script:"
    echo -e "  ${BLUE}./start_ssl_slam.sh${NC}"
else
    echo ""
    echo -e "${RED}✗ Build failed!${NC}"
    echo -e "${YELLOW}Check the error messages above.${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}========================================${NC}"

