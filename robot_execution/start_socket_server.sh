#!/bin/bash
# Script to start the Socket Robot Server on Raspberry Pi

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default configuration
HOST="${1:-0.0.0.0}"
PORT="${2:-5005}"

echo -e "${BLUE}=== Starting Socket Robot Server ===${NC}"
echo -e "${BLUE}Host: ${HOST}${NC}"
echo -e "${BLUE}Port: ${PORT}${NC}"

# Source ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}Warning: ROS2 environment not detected.${NC}"
    echo "Attempting to source ROS2 setup..."
    
    # Try common ROS2 installation paths
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
        echo -e "${GREEN}Sourced ROS2 Jazzy environment.${NC}"
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo -e "${GREEN}Sourced ROS2 Humble environment.${NC}"
    elif [ -f "/opt/ros/iron/setup.bash" ]; then
        source /opt/ros/iron/setup.bash
        echo -e "${GREEN}Sourced ROS2 Iron environment.${NC}"
    else
        echo -e "${RED}Error: No supported ROS2 environment found.${NC}"
        echo "Please install ROS2 Jazzy, Humble, or Iron, or source it manually:"
        echo "  source /opt/ros/<distro>/setup.bash"
        exit 1
    fi
else
    echo -e "${GREEN}ROS2 environment already sourced: $ROS_DISTRO${NC}"
fi

# Check if xarm_automatic nodes are running
echo -e "${BLUE}Checking if xarm_automatic nodes are running...${NC}"
if ! ros2 node list | grep -q "command_xarm\|xarm_kinematics"; then
    echo -e "${YELLOW}Warning: xarm_automatic nodes not detected.${NC}"
    echo -e "${YELLOW}Starting xarm_automatic in background...${NC}"
    
    # Start xarm_automatic in background
    ros2 launch xarmrob xarm_automatic.launch.py > /tmp/xarm_automatic.log 2>&1 &
    XARM_PID=$!
    echo "xarm_automatic PID: $XARM_PID"
    
    # Wait a bit for nodes to start
    sleep 3
    
    # Check again
    if ros2 node list | grep -q "command_xarm\|xarm_kinematics"; then
        echo -e "${GREEN}✓ xarm_automatic nodes started successfully${NC}"
    else
        echo -e "${RED}✗ Failed to start xarm_automatic nodes${NC}"
        echo "Please start them manually:"
        echo "  ros2 launch xarmrob xarm_automatic.launch.py"
        exit 1
    fi
else
    echo -e "${GREEN}✓ xarm_automatic nodes are running${NC}"
fi

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Start socket server
echo -e "${BLUE}Starting socket robot server...${NC}"
cd "$SCRIPT_DIR"
python3 socket_server.py "$HOST" "$PORT"

