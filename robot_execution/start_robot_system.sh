#!/bin/bash

# Complete Robot System Startup Script
# This script starts all required ROS2 nodes for the robot system:
# 1. ROSBridge WebSocket Server
# 2. Robot Control Nodes (xarm_automatic)
# 3. Robot Command Receiver

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Robot System Startup Script ===${NC}"
echo ""

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}Warning: ROS2 environment not detected.${NC}"
    echo "Attempting to source ROS2 setup..."
    
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
        exit 1
    fi
fi

# Default values
ROSBridge_PORT=${1:-9090}
ROSBridge_ADDRESS=${2:-0.0.0.0}

echo -e "${BLUE}Configuration:${NC}"
echo "  ROSBridge Port: ${ROSBridge_PORT}"
echo "  ROSBridge Address: ${ROSBridge_ADDRESS}"
echo ""

# Function to check if a node is running
check_node() {
    local node_name=$1
    if ros2 node list 2>/dev/null | grep -q "$node_name"; then
        return 0
    else
        return 1
    fi
}

# Function to wait for a node
wait_for_node() {
    local node_name=$1
    local max_wait=10
    local count=0
    
    echo -e "${YELLOW}Waiting for $node_name...${NC}"
    while [ $count -lt $max_wait ]; do
        if check_node "$node_name"; then
            echo -e "${GREEN}✓ $node_name is running${NC}"
            return 0
        fi
        sleep 1
        count=$((count + 1))
    done
    
    echo -e "${RED}✗ $node_name did not start within ${max_wait}s${NC}"
    return 1
}

# Start ROSBridge server in background
echo -e "${BLUE}[1/3] Starting ROSBridge WebSocket Server...${NC}"
ros2 launch rosbridge_server rosbridge_websocket.launch.py \
    port:=${ROSBridge_PORT} \
    address:=${ROSBridge_ADDRESS} > /tmp/rosbridge.log 2>&1 &
ROSBridge_PID=$!

sleep 2
if check_node "rosbridge_websocket"; then
    echo -e "${GREEN}✓ ROSBridge server started (PID: $ROSBridge_PID)${NC}"
else
    echo -e "${RED}✗ Failed to start ROSBridge server${NC}"
    cat /tmp/rosbridge.log
    exit 1
fi

# Start robot control nodes
echo -e "${BLUE}[2/3] Starting Robot Control Nodes...${NC}"
if [ -f "${SCRIPT_DIR}/xarm_automatic.launch.py" ]; then
    ros2 launch xarmrob xarm_automatic.launch.py > /tmp/xarm.log 2>&1 &
    XARM_PID=$!
    sleep 3
    if check_node "command_arm" || check_node "xarm_kinematics"; then
        echo -e "${GREEN}✓ Robot control nodes started (PID: $XARM_PID)${NC}"
    else
        echo -e "${YELLOW}⚠ Robot control nodes may not be fully started${NC}"
    fi
else
    echo -e "${YELLOW}⚠ xarm_automatic.launch.py not found, skipping${NC}"
fi

# Start robot command receiver
echo -e "${BLUE}[3/3] Starting Robot Command Receiver...${NC}"
if [ -f "${SCRIPT_DIR}/robot_command_receiver.py" ]; then
    cd "${SCRIPT_DIR}"
    python3 robot_command_receiver.py > /tmp/robot_receiver.log 2>&1 &
    RECEIVER_PID=$!
    sleep 2
    if check_node "robot_command_receiver"; then
        echo -e "${GREEN}✓ Robot command receiver started (PID: $RECEIVER_PID)${NC}"
    else
        echo -e "${YELLOW}⚠ Robot command receiver may not be fully started${NC}"
        echo "Check log: /tmp/robot_receiver.log"
    fi
else
    echo -e "${RED}✗ robot_command_receiver.py not found${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}=== All nodes started successfully ===${NC}"
echo ""
echo "Running nodes:"
ros2 node list
echo ""
echo "Active topics:"
ros2 topic list | grep -E "(robot|endpoint|gripper)" || echo "  (No robot topics found yet)"
echo ""
echo -e "${YELLOW}Log files:${NC}"
echo "  ROSBridge: /tmp/rosbridge.log"
echo "  Robot Control: /tmp/xarm.log"
echo "  Command Receiver: /tmp/robot_receiver.log"
echo ""
echo -e "${YELLOW}To stop all nodes, press Ctrl+C or run:${NC}"
echo "  kill $ROSBridge_PID $XARM_PID $RECEIVER_PID"
echo ""

# Wait for Ctrl+C
trap "echo ''; echo -e '${YELLOW}Shutting down...${NC}'; kill $ROSBridge_PID $XARM_PID $RECEIVER_PID 2>/dev/null; exit" INT TERM

# Keep script running
wait

