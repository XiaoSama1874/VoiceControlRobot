"""
Robot Functions Module

This module contains the atomic robot control functions.
move_home, move, and grasp communicate with Raspberry Pi via TCP socket or ROSBridge.
see function uses local implementation (no camera available yet).

Debug Mode:
When ROBOT_DEBUG_MODE is enabled in config.py, functions will return mock responses
instead of communicating with the real robot server. This is useful for testing
without hardware or network connectivity.

Communication Modes:
- "socket": TCP socket communication (legacy)
- "rosbridge": ROSBridge WebSocket communication (recommended)
"""

from typing import Dict, Any, Optional, Tuple
import time
import config

# Import communication clients based on configuration
if config.ROBOT_COMMUNICATION_MODE == "rosbridge":
    try:
        from .rosbridge_client import get_rosbridge_client
        _get_client = get_rosbridge_client
        print("[robot_functions] Using ROSBridge communication mode")
    except ImportError:
        print("[robot_functions] Warning: roslibpy not available, falling back to socket")
        from .socket_client import get_socket_client
        _get_client = get_socket_client
else:
    from .socket_client import get_socket_client
    _get_client = get_socket_client
    print("[robot_functions] Using Socket communication mode")


def _get_mock_response(json_command: Dict[str, Any]) -> Dict[str, Any]:
    """
    Generate a mock server response for debug mode.
    
    Args:
        json_command: The command that would be sent to the server
        
    Returns:
        Mock response dictionary matching server response format
    """
    action = json_command.get("action", "unknown")
    
    # Simulate network delay if enabled
    if config.ROBOT_DEBUG_SIMULATE_DELAY:
        time.sleep(config.ROBOT_DEBUG_DELAY_TIME)
    
    # Generate appropriate mock response based on action
    if action == "move":
        return {
            "status": "success",
            "message": f"Robot arm moved to position ({json_command.get('x')}, {json_command.get('y')}, {json_command.get('z')})"
        }
    elif action == "grasp":
        grasp_state = json_command.get("grasp", False)
        action_desc = "grasped" if grasp_state else "released"
        return {
            "status": "success",
            "message": f"End effector {action_desc} successfully"
        }
    else:
        return {
            "status": "error",
            "message": f"Unknown action: {action}"
        }


def move_home() -> Dict[str, Any]:
    """
    Move the robot arm to home position.
    
    This function converts move_home to a move command with home coordinates.
    The actual move_home action is handled by calling move() with home position.
    
    Returns:
        Dictionary with status and debug information
    """
    print("[robot_functions] move_home() called - converting to move() with home coordinates")
    
    # Get home position coordinates
    home_pos = config.ROBOT_HOME_POSITION
    
    # Convert move_home to move command
    print(f"[robot_functions] [DEBUG] Converting move_home to move({home_pos['x']}, {home_pos['y']}, {home_pos['z']})")
    result = move(x=home_pos["x"], y=home_pos["y"], z=home_pos["z"])
    
    # Update action name in result for consistency
    if result.get("action") == "move":
        result["action"] = "move_home"
        # Update message if needed
        if "moved to position" in result.get("message", ""):
            result["message"] = result["message"].replace("moved to position", "moved to home position")
    
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
    print(f"[robot_functions] [DEBUG] Built JSON command: {json_command}")
    
    # Check if debug mode is enabled
    print(f"[robot_functions] [DEBUG] DEBUG_MODE = {config.ROBOT_DEBUG_MODE}")
    if config.ROBOT_DEBUG_MODE:
        print("[robot_functions] [DEBUG MODE] Using mock server response")
        print(f"[robot_functions] [DEBUG] Command that would be sent: {json_command}")
        response = _get_mock_response(json_command)
        print(f"[robot_functions] [DEBUG] Mock response received: {response}")
    else:
        # Send command via configured communication method
        comm_mode = config.ROBOT_COMMUNICATION_MODE
        print(f"[robot_functions] [DEBUG] Using real {comm_mode} communication")
        print(f"[robot_functions] [DEBUG] Getting {comm_mode} client...")
        client = _get_client()
        print(f"[robot_functions] [DEBUG] Sending command via {comm_mode}...")
        response = client.send_command(json_command)
        print(f"[robot_functions] [DEBUG] {comm_mode.capitalize()} response received: {response}")
    
    if response is None:
        print(f"[robot_functions] [DEBUG] ✗ Response is None - communication failed")
        result = {
            "status": "error",
            "action": "move",
            "message": "Failed to communicate with robot server",
            "position": {"x": final_x, "y": final_y, "z": final_z}
        }
        print(f"[robot_functions] [DEBUG] Returning error result: {result}")
        return result
    
    # Check server response status
    server_status = response.get("status", "error")
    print(f"[robot_functions] [DEBUG] Server response status: {server_status}")
    
    if server_status == "success" or server_status is True:
        result = {
            "status": "success",
            "action": "move",
            "message": response.get("message", f"Robot arm moved to position ({final_x}, {final_y}, {final_z})"),
            "position": {"x": final_x, "y": final_y, "z": final_z}
        }
        print(f"[robot_functions] [DEBUG] ✓ Success result: {result}")
    else:
        result = {
            "status": "error",
            "action": "move",
            "message": response.get("message", "Server returned error status"),
            "position": {"x": final_x, "y": final_y, "z": final_z}
        }
        print(f"[robot_functions] [DEBUG] ✗ Error result: {result}")
    
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
    print(f"[robot_functions] [DEBUG] Built JSON command: {json_command}")
    
    # Check if debug mode is enabled
    print(f"[robot_functions] [DEBUG] DEBUG_MODE = {config.ROBOT_DEBUG_MODE}")
    if config.ROBOT_DEBUG_MODE:
        print("[robot_functions] [DEBUG MODE] Using mock server response")
        print(f"[robot_functions] [DEBUG] Command that would be sent: {json_command}")
        response = _get_mock_response(json_command)
        print(f"[robot_functions] [DEBUG] Mock response received: {response}")
    else:
        # Send command via configured communication method
        comm_mode = config.ROBOT_COMMUNICATION_MODE
        print(f"[robot_functions] [DEBUG] Using real {comm_mode} communication")
        print(f"[robot_functions] [DEBUG] Getting {comm_mode} client...")
        client = _get_client()
        print(f"[robot_functions] [DEBUG] Sending command via {comm_mode}...")
        response = client.send_command(json_command)
        print(f"[robot_functions] [DEBUG] {comm_mode.capitalize()} response received: {response}")
    
    if response is None:
        print(f"[robot_functions] [DEBUG] ✗ Response is None - communication failed")
        result = {
            "status": "error",
            "action": "grasp",
            "grasp_state": grasp,
            "message": "Failed to communicate with robot server"
        }
        print(f"[robot_functions] [DEBUG] Returning error result: {result}")
        return result
    
    # Check server response status
    server_status = response.get("status", "error")
    print(f"[robot_functions] [DEBUG] Server response status: {server_status}")
    
    if server_status == "success" or server_status is True:
        result = {
            "status": "success",
            "action": "grasp",
            "grasp_state": grasp,
            "message": response.get("message", f"End effector {'grasped' if grasp else 'released'} successfully")
        }
        print(f"[robot_functions] [DEBUG] ✓ Success result: {result}")
    else:
        result = {
            "status": "error",
            "action": "grasp",
            "grasp_state": grasp,
            "message": response.get("message", "Server returned error status")
        }
        print(f"[robot_functions] [DEBUG] ✗ Error result: {result}")
    
    print(f"[robot_functions] grasp() completed: {result}")
    return result


def see(target: str) -> Dict[str, Any]:
    """
    Use vision to identify a target object and get its coordinates.
    
    Args:
        target: Target object description (e.g., "red_object", "green_object", "blue_object")
    
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

