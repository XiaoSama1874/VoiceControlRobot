#!/bin/bash

# ROSBridge Server Startup Script
# This script starts the ROSBridge WebSocket server for external communication

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== ROSBridge Server Startup Script ===${NC}"
echo ""

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}Warning: ROS2 environment not detected.${NC}"
    echo "Attempting to source ROS2 setup..."
    
    # Try common ROS2 installation paths
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
        echo -e "${GREEN}Sourced ROS2 Jazzy${NC}"
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo -e "${GREEN}Sourced ROS2 Humble${NC}"
    elif [ -f "/opt/ros/iron/setup.bash" ]; then
        source /opt/ros/iron/setup.bash
        echo -e "${GREEN}Sourced ROS2 Iron${NC}"
    else
        echo -e "${RED}Error: Could not find ROS2 installation.${NC}"
        echo "Please source ROS2 setup.bash manually:"
        echo "  source /opt/ros/<distro>/setup.bash"
        exit 1
    fi
fi

# Check if rosbridge_server is installed
if ! ros2 pkg list | grep -q rosbridge_server; then
    echo -e "${RED}Error: rosbridge_server package not found.${NC}"
    echo ""
    echo "Please install it with:"
    echo "  sudo apt install ros-${ROS_DISTRO}-rosbridge-suite"
    exit 1
fi

# Default values
PORT=${1:-9090}
ADDRESS=${2:-0.0.0.0}

echo -e "${GREEN}Starting ROSBridge WebSocket Server...${NC}"
echo "  Port: ${PORT}"
echo "  Address: ${ADDRESS}"
echo ""

# Start ROSBridge server
ros2 launch rosbridge_server rosbridge_websocket.launch.py \
    port:=${PORT} \
    address:=${ADDRESS}

