"""
Configuration file for Voice Control Robot

This file contains all hyperparameters and configuration values used across modules.
"""

# ============================================================================
# Voice Module Configuration
# ============================================================================

# Command Detection Keywords
VOICE_START_KEYWORD = "hi robot"  # Keyword to start command recording
VOICE_STOP_KEYWORD = "command end"  # Keyword to stop command recording

# Speech Recognition
VOICE_LANGUAGE = "en-US"  # Language code for speech recognition

# Audio Processing
VOICE_SILENCE_DURATION = 1.0  # Duration of silence to consider end of phrase (seconds)
VOICE_PHRASE_TIME_LIMIT = 5.0  # Maximum time to wait for a phrase (seconds)
VOICE_ENERGY_THRESHOLD = 4000  # Energy threshold for speech recognition (adjust based on ambient noise)
VOICE_DYNAMIC_ENERGY_THRESHOLD = True  # Enable dynamic energy threshold adjustment
VOICE_AMBIENT_NOISE_DURATION = 0.5  # Duration to listen for ambient noise adjustment (seconds)
VOICE_COMMAND_END_SILENCE_DURATION = 2.0  # Duration of silence to consider command ended (seconds)

# ============================================================================
# LLM Module Configuration
# ============================================================================

# OpenAI API Settings
LLM_MODEL = "gpt-5.1"  # OpenAI model to use
LLM_TEMPERATURE = 0.7  # Sampling temperature (0.0 to 2.0)
LLM_MAX_TOKENS = 2000  # Maximum tokens in response
LLM_MAX_RETRIES = 3  # Maximum number of retry attempts for API calls

# ============================================================================
# Executor Module Configuration
# ============================================================================

# Unit System
# All coordinates and distances are in meters (m)
ROBOT_UNIT = "m"  # Unit for all robot coordinates and distances

# Robot Home Position (in m)
# Default position where the robot arm returns to
# Matches pick_and_place.py: [0.1697, 0.0, 0.2]
ROBOT_HOME_POSITION = {
    "x": 0.1697,   # m
    "y": 0.0,      # m
    "z": 0.2       # m
}

# Vision Target Fixed Coordinates (placeholder values, in m)
# These are used when vision system is not available
# Updated to match pick_and_place.py coordinates
VISION_TARGET_COORDINATES = {
    "cube": {"x": 0.207, "y": 0.00, "z": 0.070},      # m - Generic object
    "red_object": {"x": 0.2271, "y": 0.1712, "z": 0.0372},   # m - Matches pick_and_place.py
    "green_object": {"x": 0.2271, "y": -0.1448, "z": 0.0360}, # m - Matches pick_and_place.py
    "blue_object": {"x": 0.15, "y": 0.08, "z": 0.02},  # m - Blue object coordinates
}

# Bin/Drop-off Location Coordinates (in m)
# Predefined locations where objects can be placed
# All bins are configurable independently
# Updated to match pick_and_place.py coordinates
BIN_COORDINATES = {
    "bin": {"x": 0.1, "y": -0.2, "z": 0.3},  # m - Default bin location
    "red_bin": {"x": 0.1503, "y": 0.0, "z": 0.0708},  # m - Matches pick_and_place.py
    "green_bin": {"x": 0.2021, "y": 0.0, "z": 0.0708},  # m - Matches pick_and_place.py
}

# Default coordinates when move() is called with None values (in m)
ROBOT_DEFAULT_POSITION = {
    "x": 0.0,  # m
    "y": 0.0,  # m
    "z": 0.0   # m
}

# Robot Function Execution Times (simulation delays)
ROBOT_MOVE_HOME_DELAY = 0.5  
ROBOT_MOVE_DELAY = 0.5   
ROBOT_GRASP_DELAY = 0.3  
ROBOT_SEE_DELAY = 0.5   

# Robot Communication Configuration
# Choose communication mode: "socket" or "rosbridge"
ROBOT_COMMUNICATION_MODE = "socket"  # Options: "socket" or "rosbridge"

# Socket Communication Configuration (for backward compatibility)
# Raspberry Pi robot server connection settings
ROBOT_SOCKET_HOST = "192.168.1.76"  # Default host (overridden by ME578_RPI_IP_ADDR env var)
ROBOT_SOCKET_PORT = 5005  # Port number for robot server
ROBOT_SOCKET_TIMEOUT = 30.0  # Connection and receive timeout (seconds)
ROBOT_SOCKET_RETRY_ATTEMPTS = 3  # Number of retry attempts for connection/communication
ROBOT_SOCKET_RETRY_DELAY = 1.0  # Delay between retry attempts (seconds)

# ROSBridge Communication Configuration
# ROSBridge WebSocket server connection settings
ROSBridge_HOST = "192.168.1.76"  # ROSBridge server host (overridden by ME578_RPI_IP_ADDR env var)
ROSBridge_PORT = 9090  # ROSBridge WebSocket port (default: 9090)
ROSBridge_TIMEOUT = 30.0  # Connection and response timeout (seconds)

# Debug Mode Configuration
# When enabled, robot functions will use mock responses instead of real socket communication
ROBOT_DEBUG_MODE = False  # Set to True to enable mock server responses
ROBOT_DEBUG_SIMULATE_DELAY = True  # Simulate network delay in debug mode (seconds)
ROBOT_DEBUG_DELAY_TIME = 0.1  # Delay time for mock responses (seconds)

# Relative Movement Configuration
ROBOT_RELATIVE_MOVE_DEFAULT_DISTANCE = 0.05  # Default distance for relative movement (m) (5cm)

# Direction to Axis Mapping for Relative Movement
# Coordinate System: Base center (0,0,0) as origin
# - X-axis: Positive direction points forward (front of robot)
# - Y-axis: Positive direction points left (left side of robot)
# - Z-axis: Positive direction points upward (perpendicular to base plane)
# Directions are relative to the robot's current position
ROBOT_DIRECTION_MAPPING = {
    "forward": {"axis": "x", "sign": 1},    # Forward → +X axis (front)
    "backward": {"axis": "x", "sign": -1},   # Backward → -X axis (back)
    "back": {"axis": "x", "sign": -1},      # Back (alias for backward) → -X axis
    "left": {"axis": "y", "sign": 1},       # Left → +Y axis (left side)
    "right": {"axis": "y", "sign": -1},     # Right → -Y axis (right side)
    "up": {"axis": "z", "sign": 1},         # Up → +Z axis (upward)
    "down": {"axis": "z", "sign": -1}       # Down → -Z axis (downward)
}

# ============================================================================
# Valid Actions
# ============================================================================

VALID_ROBOT_ACTIONS = {
    "move_home",
    "move",
    "grasp",
    "see"
}
