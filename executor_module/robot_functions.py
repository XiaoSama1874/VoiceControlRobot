"""
Robot Functions Module

This module contains the atomic robot control functions.
move_home, move, and grasp communicate with Raspberry Pi via TCP socket.
see function uses local implementation (no camera available yet).
"""

from typing import Dict, Any, Optional, Tuple
import time
import config
from .socket_client import get_socket_client


def move_home() -> Dict[str, Any]:
    """
    Move the robot arm to home position.
    
    Returns:
        Dictionary with status and debug information
    """
    print("[robot_functions] move_home() called")
    
    # Get home position coordinates
    home_pos = config.ROBOT_HOME_POSITION
    
    # Build JSON command (same format as move)
    json_command = {
        "action": "move_home",
        "action_description": "Move to home position",
        "x": home_pos["x"],
        "y": home_pos["y"],
        "z": home_pos["z"]
    }
    
    # Send command via socket
    socket_client = get_socket_client()
    response = socket_client.send_command(json_command)
    
    if response is None:
        return {
            "status": "error",
            "action": "move_home",
            "message": "Failed to communicate with robot server",
            "position": home_pos.copy()
        }
    
    # Check server response status
    server_status = response.get("status", "error")
    if server_status == "success" or server_status is True:
        result = {
            "status": "success",
            "action": "move_home",
            "message": response.get("message", "Robot arm moved to home position"),
            "position": home_pos.copy()
        }
    else:
        result = {
            "status": "error",
            "action": "move_home",
            "message": response.get("message", "Server returned error status"),
            "position": home_pos.copy()
        }
    
    print(f"[robot_functions] move_home() completed: {result}")
    return result


def move(x: Optional[float] = None, y: Optional[float] = None, z: Optional[float] = None) -> Dict[str, Any]:
    """
    Move the robot arm to specified coordinates.
    
    Args:
        x: X coordinate (None if unknown, will use default or previous value)
        y: Y coordinate (None if unknown, will use default or previous value)
        z: Z coordinate (None if unknown, will use default or previous value)
    
    Returns:
        Dictionary with status, coordinates, and debug information
    """
    print(f"[robot_functions] move(x={x}, y={y}, z={z}) called")
    
    # Use default values if None (in real implementation, might use current position or default)
    final_x = x if x is not None else config.ROBOT_DEFAULT_POSITION["x"]
    final_y = y if y is not None else config.ROBOT_DEFAULT_POSITION["y"]
    final_z = z if z is not None else config.ROBOT_DEFAULT_POSITION["z"]
    
    # Build JSON command
    json_command = {
        "action": "move",
        "action_description": "Move to position",
        "x": final_x,
        "y": final_y,
        "z": final_z
    }
    
    # Send command via socket
    socket_client = get_socket_client()
    response = socket_client.send_command(json_command)
    
    if response is None:
        return {
            "status": "error",
            "action": "move",
            "message": "Failed to communicate with robot server",
            "position": {"x": final_x, "y": final_y, "z": final_z}
        }
    
    # Check server response status
    server_status = response.get("status", "error")
    if server_status == "success" or server_status is True:
        result = {
            "status": "success",
            "action": "move",
            "message": response.get("message", f"Robot arm moved to position ({final_x}, {final_y}, {final_z})"),
            "position": {"x": final_x, "y": final_y, "z": final_z}
        }
    else:
        result = {
            "status": "error",
            "action": "move",
            "message": response.get("message", "Server returned error status"),
            "position": {"x": final_x, "y": final_y, "z": final_z}
        }
    
    print(f"[robot_functions] move() completed: {result}")
    return result


def grasp(grasp: bool) -> Dict[str, Any]:
    """
    Grasp or release the end effector.
    
    Args:
        grasp: True to grasp, False to release
    
    Returns:
        Dictionary with status and debug information
    """
    action_str = "grasp" if grasp else "release"
    print(f"[robot_functions] grasp(grasp={grasp}) called - {action_str}")
    
    # Build JSON command
    json_command = {
        "action": "grasp",
        "action_description": "Grasp" if grasp else "Release",
        "grasp": grasp
    }
    
    # Send command via socket
    socket_client = get_socket_client()
    response = socket_client.send_command(json_command)
    
    if response is None:
        return {
            "status": "error",
            "action": "grasp",
            "grasp_state": grasp,
            "message": "Failed to communicate with robot server"
        }
    
    # Check server response status
    server_status = response.get("status", "error")
    if server_status == "success" or server_status is True:
        result = {
            "status": "success",
            "action": "grasp",
            "grasp_state": grasp,
            "message": response.get("message", f"End effector {'grasped' if grasp else 'released'} successfully")
        }
    else:
        result = {
            "status": "error",
            "action": "grasp",
            "grasp_state": grasp,
            "message": response.get("message", "Server returned error status")
        }
    
    print(f"[robot_functions] grasp() completed: {result}")
    return result


def see(target: str) -> Dict[str, Any]:
    """
    Use vision to identify a target object and get its coordinates.
    
    Args:
        target: Target object description (e.g., "red_square", "blue_cube")
    
    Returns:
        Dictionary with status, target information, and coordinates
    """
    print(f"[robot_functions] see(target='{target}') called")
    
    print(f"[robot_functions] Using vision to identify '{target}'...")
    
    # Simulate execution time
    time.sleep(config.ROBOT_SEE_DELAY)
    
    # Return fixed coordinates for now (in real implementation, this would come from vision system)
    # Use coordinates from config (only VISION_TARGET_COORDINATES, bin coordinates are handled separately)
    coordinates = config.VISION_TARGET_COORDINATES.get(target.lower(), config.ROBOT_DEFAULT_POSITION.copy())
    
    result = {
        "status": "success",
        "action": "see",
        "target": target,
        "message": f"Identified '{target}' at coordinates ({coordinates['x']}, {coordinates['y']}, {coordinates['z']})",
        "coordinates": coordinates,
        "position": coordinates  # Alias for compatibility
    }
    
    print(f"[robot_functions] see() completed: {result}")
    return result

