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

# Robot Home Position
ROBOT_HOME_POSITION = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0
}

# Vision Target Fixed Coordinates (placeholder values)
# These are used when vision system is not available
VISION_TARGET_COORDINATES = {
    "cube": {"x": 10.0, "y": 5.0, "z": 2.0},
    "red_cube": {"x": 10.0, "y": 5.0, "z": 2.0},
    "blue_cube": {"x": 15.0, "y": 8.0, "z": 2.0},
    "green_cube": {"x": 12.0, "y": 3.0, "z": 2.0},
}

# Default coordinates when move() is called with None values
ROBOT_DEFAULT_POSITION = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0
}

# Robot Function Execution Times (simulation delays)
ROBOT_MOVE_HOME_DELAY = 0.5  # seconds
ROBOT_MOVE_DELAY = 0.5  # seconds
ROBOT_GRASP_DELAY = 0.3  # seconds
ROBOT_SEE_DELAY = 0.5  # seconds

# Relative Movement Configuration
ROBOT_RELATIVE_MOVE_DEFAULT_DISTANCE = 20.0  # Default distance for relative movement (cm)

# Direction to Axis Mapping for Relative Movement
# Directions are relative to the robot's current position
ROBOT_DIRECTION_MAPPING = {
    "right": {"axis": "x", "sign": 1},      # Right → x axis positive
    "left": {"axis": "x", "sign": -1},      # Left → x axis negative
    "forward": {"axis": "y", "sign": 1},    # Forward → y axis positive
    "backward": {"axis": "y", "sign": -1},  # Backward → y axis negative
    "back": {"axis": "y", "sign": -1},      # Back (alias for backward)
    "up": {"axis": "z", "sign": 1},         # Up → z axis positive
    "down": {"axis": "z", "sign": -1}       # Down → z axis negative
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
