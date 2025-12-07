# VoiceControlRobot

Voice-controlled robot system for 639 Introduction to Robotics final project.

## Project Overview

This project implements a voice-controlled robot system that:
1. Listens to voice commands continuously
2. Uses LLM (GPT-5) to parse natural language commands into structured execution plans
3. Executes plans using vision recognition and robotic arm control
4. Maintains context between commands for relative movements

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         User Voice Input                                │
│                    (Natural Language Commands)                           │
└────────────────────────────┬────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      Voice Module                                       │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐      │
│  │ Speech           │→ │ Command          │→ │ Voice           │      │
│  │ Recognizer       │  │ Detector         │  │ Listener        │      │
│  │ (Google STT API) │  │ (Keywords +      │  │ (Orchestrator)  │      │
│  │                  │  │  Silence Detect) │  │                 │      │
│  └──────────────────┘  └──────────────────┘  └────────┬─────────┘      │
└───────────────────────────────────────────────────────┼─────────────────┘
                                                        │ Text Command
                                                        ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      LLM Module                                         │
│  ┌──────────────────┐  ┌──────────────────┐                            │
│  │ LLM Planner      │→ │ Plan Parser      │                            │
│  │ (OpenAI GPT-5.1) │  │ (JSON Validator) │                            │
│  │                  │  │                  │                            │
│  │ - System Prompt  │  │ - Parse JSON     │                            │
│  │ - API Call       │  │ - Validate Plan │                            │
│  │ - Retry Logic    │  │ - Extract Tasks  │                            │
│  └──────────────────┘  └────────┬─────────┘                            │
└──────────────────────────────────┼─────────────────────────────────────┘
                                    │ Execution Plan (JSON)
                                    │ {tasks: [{action, parameters, ...}]}
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                    Executor Module                                      │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐     │
│  │ Executor         │→ │ Context Manager  │  │ Task Sequencer   │     │
│  │ (Plan Runner)    │  │                  │  │                  │     │
│  │                  │  │ - Position State │  │ - Execute Tasks  │     │
│  │ - Validate Plan  │  │ - Vision Results  │  │ - Error Handle  │     │
│  │ - Execute Tasks  │  │ - Grasp State    │  │ - Retry Logic    │     │
│  └────────┬─────────┘  └──────────────────┘  └────────┬─────────┘     │
│           │                                            │                │
│           └──────────────────┬─────────────────────────┘                │
│                              │                                          │
│                              ▼                                          │
│              ┌───────────────────────────────────────┐                  │
│              │      Robot Functions                  │                  │
│              │  ┌──────────┐  ┌──────────┐           │                  │
│              │  │ move()   │  │ grasp()  │           │                  │
│              │  │ move_    │  │ see()    │           │                  │
│              │  │ home()   │  │          │           │                  │
│              │  └────┬─────┘  └────┬─────┘           │                  │
│              └──────┼─────────────┼─────────────────┘                  │
│                     │             │                                     │
│                     └──────┬──────┘                                     │
│                            │                                            │
│                            ▼                                            │
│              ┌───────────────────────────────────────┐                  │
│              │      Socket Client                     │                  │
│              │  ┌──────────────────────────────────┐  │                  │
│              │  │ - TCP Connection Management      │  │                  │
│              │  │ - JSON Serialization/Deserial.   │  │                  │
│              │  │ - Auto Reconnection              │  │                  │
│              │  │ - Error Handling & Retry         │  │                  │
│              │  │ - Debug Mode Support             │  │                  │
│              │  └──────────────┬───────────────────┘  │                  │
│              └─────────────────┼───────────────────────┘                  │
└───────────────────────────────┼──────────────────────────────────────────┘
                                 │ TCP Socket (Port 5005)
                                 │ JSON Messages
                                 ▼
                    ┌────────────────────────────────────┐
                    │   Raspberry Pi Robot Server        │
                    │   (ROS Node)                       │
                    │                                    │
                    │  ┌──────────────┐  ┌──────────┐  │
                    │  │ Trajectory   │  │ Vision   │  │
                    │  │ Controller   │  │ System   │  │
                    │  │ (MoveIt)     │  │ (Camera)  │  │
                    │  └──────┬───────┘  └────┬─────┘  │
                    │         │               │         │
                    └─────────┼───────────────┼─────────┘
                              │               │
                              ▼               ▼
                    ┌────────────────────────────────────┐
                    │      Robot Hardware                │
                    │  ┌──────────┐  ┌──────────┐      │
                    │  │ Robotic  │  │ Camera   │      │
                    │  │ Arm      │  │ (Vision) │      │
                    │  │          │  │          │      │
                    │  │ - Joints │  │ - Object │      │
                    │  │ - Gripper│  │   Detect │      │
                    │  └──────────┘  └──────────┘      │
                    └────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────┐
│                    Configuration Layer (config.py)                      │
│  - Voice Settings (keywords, language, timeouts)                        │
│  - LLM Settings (model, temperature, tokens)                            │
│  - Robot Settings (home position, coordinates, distances)               │
│  - Socket Settings (host, port, timeout, retries)                       │
│  - Debug Mode Settings                                                  │
└─────────────────────────────────────────────────────────────────────────┘
```

## Supported Capabilities

### 1. Voice Recognition
- **Continuous Listening**: Monitors microphone input continuously
- **Speech-to-Text**: Uses Google Speech Recognition API (English)
- **Command Detection**: 
  - Start keyword: "hi robot" (configurable)
  - Stop keyword: "command end" (configurable)
  - **Silence Detection**: Automatically ends command after 2 seconds of silence (configurable)
- **Multi-segment Commands**: Supports commands split across multiple speech segments

### 2. Natural Language Understanding
- **LLM Integration**: Uses OpenAI GPT-5 to understand natural language commands
- **Plan Generation**: Converts commands into structured JSON execution plans
- **Smart Planning**: Automatically adds necessary steps (e.g., move_home before see())

### 3. Relative Movement
- **Direction-based Movement**: Supports relative movement commands
  - Directions: `right`, `left`, `forward`, `backward`, `back`, `up`, `down`
  - **Direction Mapping** (based on robot coordinate system):
    - `forward` → +X axis (move forward)
    - `backward` or `back` → -X axis (move backward)
    - `left` → +Y axis (move to the left)
    - `right` → -Y axis (move to the right)
    - `up` → +Z axis (move upward)
    - `down` → -Z axis (move downward)
  - **Default Distance**: 0.05m (5cm, configurable in `config.py`)
  - **Units**: All coordinates and distances are in meters (m)
  - **Context Preservation**: Maintains robot position between commands
  - Example: "move forward" followed by "move forward" will move 0.05m + 0.05m from initial position
  - **Usage Examples**:
  - "move the robot arm to the right"
  - "grab the red cube and move it forward"
  - "move forward 0.3m" (custom distance)

### 4. Vision Recognition
- **Object Detection**: Identifies targets using vision system
- **Automatic Home Positioning**: Automatically moves to home position before vision recognition
- **Coordinate Extraction**: Returns target coordinates for subsequent movements
- **Supported Targets**: red_cube, blue_cube, green_cube (configurable in `config.py`)

### 5. Context Management
- **Position Tracking**: Maintains robot position across multiple commands
- **State Persistence**: Preserves grasp state, vision results, and position between executions
- **Data Flow**: Automatically passes vision coordinates to subsequent move actions

### 6. Error Handling
- **Robust Execution**: Handles errors gracefully with detailed feedback
- **Retry Mechanism**: LLM plan generation includes retry logic
- **Validation**: Plans are validated before execution

## Project Structure

```
VoiceControlRobot/
├── config.py                    # Centralized configuration file
│                                # - Voice settings (keywords, language, timeouts)
│                                # - LLM settings (model, temperature, tokens)
│                                # - Robot settings (home position, relative move distance)
│                                # - Direction mappings
│
├── main_voice.py               # Main entry point for voice-controlled system
│
├── requirements.txt            # Python dependencies
│
├── voice_module/               # Voice recognition and command detection
│   ├── __init__.py
│   ├── speech_recognizer.py    # Google Speech Recognition wrapper
│   ├── command_detector.py     # Command boundary detection (keywords + silence)
│   └── voice_listener.py      # Main listening orchestrator
│
├── llm_module/                 # LLM-based plan generation
│   ├── __init__.py
│   ├── llm_planner.py         # GPT-5 integration for plan generation
│   └── plan_parser.py         # JSON plan parsing and validation
│
├── executor_module/            # Plan execution and robot control
│   ├── __init__.py
│   ├── executor.py            # Main executor (runs plans, manages context)
│   └── robot_functions.py     # Atomic robot functions (move, grasp, see, move_home)
│
└── test/                       # Unit tests
    ├── __init__.py
    ├── test_voice_module.py   # Voice module tests
    ├── test_llm_module.py     # LLM module tests
    ├── test_executor_module.py # Executor module tests
    ├── test_plan_parser.py    # Plan parser tests
    ├── test_full_pipeline.py  # End-to-end pipeline tests
    └── llm_plans/             # Generated execution plans from tests
        └── *.txt              # Saved LLM-generated plans
```

### Directory Descriptions

#### `config.py`
Central configuration file containing all hyperparameters:
- Voice module settings (keywords, language, timeouts, energy thresholds)
- LLM module settings (model, temperature, max tokens, retries)
- Robot settings (home position, default positions, execution delays)
- Relative movement settings (default distance, direction mappings)
- Valid robot actions

#### `voice_module/`
Handles all voice-related functionality:
- **speech_recognizer.py**: Wraps Google Speech Recognition API
- **command_detector.py**: Detects command boundaries using start/stop keywords and silence detection
- **voice_listener.py**: Orchestrates continuous listening, recognition, and command detection

#### `llm_module/`
Converts natural language commands into execution plans:
- **llm_planner.py**: Interfaces with OpenAI GPT-5 to generate execution plans
- **plan_parser.py**: Parses and validates JSON execution plans from LLM

#### `executor_module/`
Executes plans and controls the robot:
- **executor.py**: Main execution engine that:
  - Runs tasks in sequence
  - Manages execution context (position, grasp state, vision results)
  - Handles relative movement calculations
  - Ensures home position before vision recognition
  - Preserves context between command executions
- **robot_functions.py**: Placeholder implementations for robot atomic functions:
  - `move_home()`: Move to home position
  - `move(x, y, z)`: Move to absolute coordinates
  - `grasp(grasp)`: Grasp (True) or release (False)
  - `see(target)`: Vision recognition

#### `test/`
Comprehensive unit tests for all modules:
- Individual module tests
- Integration tests
- Full pipeline tests
- Generated LLM plans saved in `llm_plans/` subdirectory

## Installation

1. Install dependencies:
```bash
pip install -r requirements.txt
```

Note: On macOS, you may need to install PortAudio first:
```bash
brew install portaudio
```

2. Set up environment variables:
```bash
cp .env.example .env
# Edit .env and add your OpenAI API key
```

## Usage

### Running the Voice-Controlled System

```bash
python main_voice.py
```

**Instructions:**
1. Say "hi robot" to start a command
2. Say your command (e.g., "pick up the red cube and move it to the right")
3. Say "command end" to end the command, OR wait 2 seconds of silence
4. The system will generate an execution plan and execute it
5. Press Ctrl+C to exit

### Example Commands

- **Simple movement**: "move the robot arm forward"
- **Relative movement**: "move the robot arm to the right 0.3m"
- **Object manipulation**: "pick up the red cube"
- **Complex task**: "grab the blue cube and move it forward, then release it"

## Testing

### Running Tests

Run all unit tests:
```bash
python -m unittest discover test
```

Run with verbose output:
```bash
python -m unittest discover test -v
```

Run specific test file:
```bash
python -m unittest test.test_llm_module
python -m unittest test.test_executor_module
python -m unittest test.test_voice_module
python -m unittest test.test_plan_parser
python -m unittest test.test_full_pipeline
```

### Test Files

1. **test/test_llm_module.py** - Tests for LLM plan generation
2. **test/test_executor_module.py** - Tests for plan execution
3. **test/test_voice_module.py** - Tests for voice module
4. **test/test_plan_parser.py** - Tests for plan parsing
5. **test/test_full_pipeline.py** - End-to-end pipeline tests

### Note on Voice Tests

Tests that require microphone input are marked with `@unittest.skip` by default. To run them, remove the decorator.

## Configuration

All configuration is centralized in `config.py`. Key settings:

### Voice Module
- `VOICE_START_KEYWORD`: Command start keyword (default: "hi robot")
- `VOICE_STOP_KEYWORD`: Command stop keyword (default: "command end")
- `VOICE_COMMAND_END_SILENCE_DURATION`: Silence timeout in seconds (default: 2.0)
- `VOICE_LANGUAGE`: Speech recognition language (default: "en-US")

### LLM Module
- `LLM_MODEL`: OpenAI model to use (default: "gpt-5.1")
- `LLM_TEMPERATURE`: Sampling temperature (default: 0.7)
- `LLM_MAX_TOKENS`: Maximum response tokens (default: 2000)

### Robot Settings
- `ROBOT_HOME_POSITION`: Home position coordinates in m (default: {0.1, 0.0, 0.25})
- `ROBOT_RELATIVE_MOVE_DEFAULT_DISTANCE`: Default relative move distance in m (default: 0.05)
- `ROBOT_UNIT`: Unit system for all coordinates (default: "m")
- `ROBOT_DIRECTION_MAPPING`: Direction to axis mapping for relative movement

## Execution Plan Format

Execution plans are JSON structures with the following format:

```json
{
  "tasks": [
    {
      "task_id": 0,
      "action": "move_home",
      "description": "Move to home position for vision recognition"
    },
    {
      "task_id": 1,
      "action": "see",
      "description": "Identify the red cube using vision",
      "parameters": {"target": "red_cube"}
    },
    {
      "task_id": 2,
      "action": "move",
      "description": "Move arm to red cube location",
      "parameters": {"x": null, "y": null, "z": null}
    },
    {
      "task_id": 3,
      "action": "grasp",
      "description": "Grasp the red cube",
      "parameters": {"grasp": true}
    },
    {
      "task_id": 4,
      "action": "move",
      "description": "Move robot arm to the right",
      "parameters": {"relative": true, "direction": "right", "distance": 0.2}
    }
  ]
}
```

### Supported Actions

1. **move_home** - Move robot arm to home position (no parameters)
2. **move** - Move robot arm:
   - Absolute: `{"x": value, "y": value, "z": value}`
   - Relative: `{"relative": true, "direction": "right|left|forward|backward|back|up|down", "distance": value}`
3. **grasp** - Grasp or release: `{"grasp": true|false}`
4. **see** - Vision recognition: `{"target": "target_name"}`
   - Used for identifying objects (e.g., "red_cube", "blue_cube")
   - **Note**: Bin location is NOT identified through vision (see Predefined Locations below)

### Predefined Locations (Configuration Values)

- **bin**: A drop-off location at coordinates `(0.1, -0.2, 0.3)` m (from configuration)
  - Bin coordinates are predefined in `config.py` and do NOT require vision recognition
  - When moving to bin, use absolute coordinates: `{"x": 0.1, "y": -0.2, "z": 0.3}`
  - Do NOT use `see("bin")` - bin location is known from configuration
  - Used for commands like "put in bin", "place in bin", "drop in bin"
  - Example: "pick up the red cube and put it in the bin"

### Coordinate System

The robot arm uses a right-handed coordinate system with the robot base center as the origin:

- **Origin**: The center of the robot base is at coordinates `(0, 0, 0)`
- **X-axis**: Positive X direction points **forward** (toward the front of the robot arm)
- **Y-axis**: Positive Y direction points **left** (toward the left side of the robot arm)
- **Z-axis**: Positive Z direction points **upward** (perpendicular to the base plane)
- **End Effector Position**: The coordinates `(x, y, z)` represent the center position of the robot gripper/end effector
- **Units**: All coordinates and distances are in **meters (m)**
- **Default Home Position**: `(0.1, 0.0, 0.25)` m (configurable in `config.py`)

**Direction Mapping for Relative Movement**:
- `forward` → +X axis (move forward)
- `backward` or `back` → -X axis (move backward)
- `left` → +Y axis (move to the left)
- `right` → -Y axis (move to the right)
- `up` → +Z axis (move upward)
- `down` → -Z axis (move downward)

## Key Features

### Automatic Home Positioning
- Vision recognition (`see()`) automatically moves robot to home position if not already there
- LLM is instructed to add `move_home()` before `see()` actions

### Context Preservation
- Robot position is maintained between command executions
- Relative movements are calculated based on the last known position
- Vision results and grasp state are preserved in execution context

### Smart Plan Generation
- LLM automatically adds necessary steps (e.g., move_home before see)
- Handles relative movement commands intelligently
- Supports bin/drop-off operations (e.g., "put in bin", "place in bin")
- Validates plans before execution

### Bin/Drop-off Operations
- Predefined bin location at `(0.1, -0.2, 0.3)` m (from configuration in `config.py`)
- Bin coordinates are configuration values, NOT obtained through vision recognition
- Supports commands like "pick up the red cube and put it in the bin"
- Automatically releases gripper after moving to bin location


## Communication Protocol

The system communicates with the Raspberry Pi robot server via **TCP Socket** using **JSON** messages. The communication is persistent (single connection maintained) with automatic reconnection on failures.

### Connection Details

- **Protocol**: TCP Socket
- **Host**: Configurable via `ROBOT_SOCKET_HOST` in `config.py` or `ME578_RPI_IP_ADDR` environment variable
- **Port**: `5005` (configurable via `ROBOT_SOCKET_PORT` in `config.py`)
- **Connection Type**: Persistent (single connection reused for all commands)
- **Timeout**: `5.0` seconds (configurable via `ROBOT_SOCKET_TIMEOUT`)
- **Retry Mechanism**: Automatic reconnection with configurable retry attempts and delays

### Command Format (Client → Server)

All commands are sent as JSON objects with the following structure:

#### 1. Move Home Command

```json
{
  "action": "move_home",
  "action_description": "Move to home position",
  "x": 0.1,
  "y": 0.0,
  "z": 0.25
}
```

**Fields**:
- `action` (string): Always `"move_home"`
- `action_description` (string): Human-readable description
- `x`, `y`, `z` (float): Home position coordinates in meters (m)

#### 2. Move Command

```json
{
  "action": "move",
  "action_description": "Move to position",
  "x": 0.15,
  "y": 0.05,
  "z": 0.30
}
```

**Fields**:
- `action` (string): Always `"move"`
- `action_description` (string): Human-readable description
- `x`, `y`, `z` (float): Target position coordinates in meters (m)

#### 3. Grasp Command

```json
{
  "action": "grasp",
  "action_description": "Grasp",
  "grasp": true
}
```

**Fields**:
- `action` (string): Always `"grasp"`
- `action_description` (string): Human-readable description (`"Grasp"` or `"Release"`)
- `grasp` (boolean): `true` to close gripper, `false` to open gripper

### Response Format (Server → Client)

The server responds with a JSON object indicating success or failure:

#### Success Response

```json
{
  "status": "success",
  "message": "Robot arm moved to position (0.15, 0.05, 0.30)"
}
```

#### Error Response

```json
{
  "status": "error",
  "message": "Invalid coordinates or robot error description"
}
```

**Fields**:
- `status` (string): `"success"` or `"error"` (or boolean `true`/`false` for success)
- `message` (string): Human-readable status message

### Communication Flow

1. **Connection**: Client establishes TCP connection to server (automatic on first command)
2. **Command**: Client sends JSON command as UTF-8 encoded string
3. **Processing**: Server processes command and executes robot action
4. **Response**: Server sends JSON response with status
5. **Persistence**: Connection remains open for subsequent commands
6. **Reconnection**: If connection fails, client automatically reconnects and retries

### Error Handling

- **Connection Failure**: Client automatically retries connection (configurable attempts)
- **Timeout**: If no response received within timeout period, client reconnects and retries
- **Invalid JSON**: Server returns error response, client handles gracefully
- **Server Disconnect**: Client detects disconnect and automatically reconnects on next command

### Example Communication Sequence

```
Client → Server: {"action": "move_home", "action_description": "Move to home position", "x": 0.1, "y": 0.0, "z": 0.25}
Server → Client: {"status": "success", "message": "Robot arm moved to home position"}

Client → Server: {"action": "move", "action_description": "Move to position", "x": 0.15, "y": 0.05, "z": 0.30}
Server → Client: {"status": "success", "message": "Robot arm moved to position (0.15, 0.05, 0.30)"}

Client → Server: {"action": "grasp", "action_description": "Grasp", "grasp": true}
Server → Client: {"status": "success", "message": "End effector grasped successfully"}
```

### Configuration

All communication settings are configurable in `config.py`:

- `ROBOT_SOCKET_HOST`: Server IP address (default: `"10.141.25.190"`)
- `ROBOT_SOCKET_PORT`: Server port (default: `5005`)
- `ROBOT_SOCKET_TIMEOUT`: Connection and receive timeout in seconds (default: `5.0`)
- `ROBOT_SOCKET_RETRY_ATTEMPTS`: Number of retry attempts (default: `3`)
- `ROBOT_SOCKET_RETRY_DELAY`: Delay between retries in seconds (default: `1.0`)

**Note**: The host can be overridden by setting the `ME578_RPI_IP_ADDR` environment variable.

## Development Notes

- Robot functions (`move`, `grasp`, `see`, `move_home`) are placeholder implementations
- Actual hardware integration will be provided by team members
- All configuration is centralized in `config.py` for easy modification
- Comprehensive unit tests ensure system reliability

## License

See LICENSE file for details.
