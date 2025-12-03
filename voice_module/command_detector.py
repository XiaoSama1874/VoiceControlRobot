"""
Command Detector Module

This module detects command boundaries using keywords "Hi Robot." and "Stop."
"""

from typing import Optional, Callable
from enum import Enum
import time
import config


class CommandState(Enum):
    """State machine for command detection."""
    WAITING_FOR_START = "waiting_for_start"
    RECORDING_COMMAND = "recording_command"
    COMMAND_COMPLETE = "command_complete"


class CommandDetector:
    """Detects command boundaries using start and stop keywords."""
    
    def __init__(
        self,
        start_keyword: str = None,
        stop_keyword: str = None,
        on_command_detected: Optional[Callable[[str], None]] = None
    ):
        """
        Initialize the command detector.
        
        Args:
            start_keyword: Keyword to start command recording (default: from config)
            stop_keyword: Keyword to stop command recording (default: from config)
            on_command_detected: Callback function called when a complete command is detected
        """
        self.start_keyword = (start_keyword or config.VOICE_START_KEYWORD).lower().strip()
        self.stop_keyword = (stop_keyword or config.VOICE_STOP_KEYWORD).lower().strip()
        self.on_command_detected = on_command_detected
        self.state = CommandState.WAITING_FOR_START
        self.current_command_parts = []
        self.last_activity_time: Optional[float] = None  # Timestamp of last text received
        self.silence_timeout = config.VOICE_COMMAND_END_SILENCE_DURATION
        
    def process_text(self, text: str) -> Optional[str]:
        """
        Process recognized text and detect command boundaries.
        
        Args:
            text: Recognized text from speech recognizer
            
        Returns:
            Complete command text if detected, None otherwise
        """
        if not text:
            return None
        
        # Update last activity time
        self.last_activity_time = time.time()
            
        text_lower = text.lower().strip()
        print(f"[CommandDetector] Processing text: '{text_lower}'")
        
        # Check for start keyword
        if self.state == CommandState.WAITING_FOR_START:
            print(f"[CommandDetector] Checking for start keyword: '{self.start_keyword}'")
            if self.start_keyword in text_lower:
                # Extract text after start keyword
                start_idx = text_lower.find(self.start_keyword)
                remaining_text = text[start_idx + len(self.start_keyword):].strip()
                if remaining_text:
                    self.current_command_parts.append(remaining_text)
                self.state = CommandState.RECORDING_COMMAND
                print(f"[CommandDetector] ✓✓ START keyword '{self.start_keyword}' detected!")
                print(f"[CommandDetector] → State changed: WAITING_FOR_START → RECORDING_COMMAND")
                print(f"[CommandDetector] → Starting to record command...")
                return None
            else:
                print(f"[CommandDetector] → Waiting for start keyword '{self.start_keyword}' (current state: WAITING_FOR_START)")
        
        # Check for stop keyword
        elif self.state == CommandState.RECORDING_COMMAND:
            # Check if start keyword appears again (user wants to restart)
            if self.start_keyword in text_lower:
                print(f"[CommandDetector] ⚠ START keyword '{self.start_keyword}' detected again during recording!")
                print(f"[CommandDetector] → Resetting and restarting command recording...")
                
                # Reset and start fresh
                self.current_command_parts = []
                
                # Extract text after start keyword
                start_idx = text_lower.find(self.start_keyword)
                remaining_text = text[start_idx + len(self.start_keyword):].strip()
                if remaining_text:
                    self.current_command_parts.append(remaining_text)
                
                # State remains RECORDING_COMMAND (already in this state)
                print(f"[CommandDetector] → Command recording restarted")
                return None
            
            if self.stop_keyword in text_lower:
                # Extract text before stop keyword
                stop_idx = text_lower.find(self.stop_keyword)
                if stop_idx > 0:
                    text_before_stop = text[:stop_idx].strip()
                    if text_before_stop:
                        self.current_command_parts.append(text_before_stop)
                
                # Complete command detected
                complete_command = " ".join(self.current_command_parts).strip()
                print(f"[CommandDetector] ✓✓ STOP keyword '{self.stop_keyword}' detected!")
                print(f"[CommandDetector] → State changed: RECORDING_COMMAND → WAITING_FOR_START")
                print(f"[CommandDetector] → Complete command assembled: '{complete_command}'")
                print(f"[CommandDetector] → Command parts collected: {len(self.current_command_parts)} segment(s)")
                
                self.current_command_parts = []
                self.state = CommandState.WAITING_FOR_START
                self.last_activity_time = None  # Reset activity time
                
                # Call callback if provided
                if self.on_command_detected and complete_command:
                    print(f"[CommandDetector] → Triggering callback with complete command...")
                    self.on_command_detected(complete_command)
                
                return complete_command
            else:
                # Continue recording command
                self.current_command_parts.append(text)
                print(f"[CommandDetector] → Recording command segment: '{text}'")
                print(f"[CommandDetector] → Current command parts: {len(self.current_command_parts)} segment(s)")
                return None
        
        return None
    
    def check_silence_timeout(self) -> Optional[str]:
        """
        Check if silence timeout has been reached and complete command if so.
        
        Returns:
            Complete command text if timeout reached and command was being recorded, None otherwise
        """
        if self.state != CommandState.RECORDING_COMMAND:
            return None
        
        if self.last_activity_time is None:
            return None
        
        elapsed_silence = time.time() - self.last_activity_time
        
        if elapsed_silence >= self.silence_timeout:
            # Silence timeout reached, complete the command
            complete_command = " ".join(self.current_command_parts).strip()
            
            if complete_command:
                print(f"[CommandDetector] ✓✓ Silence timeout ({self.silence_timeout}s) reached!")
                print(f"[CommandDetector] → State changed: RECORDING_COMMAND → WAITING_FOR_START")
                print(f"[CommandDetector] → Complete command assembled (via silence timeout): '{complete_command}'")
                print(f"[CommandDetector] → Command parts collected: {len(self.current_command_parts)} segment(s)")
                
                self.current_command_parts = []
                self.state = CommandState.WAITING_FOR_START
                self.last_activity_time = None
                
                # Call callback if provided
                if self.on_command_detected:
                    print(f"[CommandDetector] → Triggering callback with complete command (silence timeout)...")
                    self.on_command_detected(complete_command)
                
                return complete_command
        
        return None
    
    def reset(self):
        """Reset the detector to initial state."""
        self.state = CommandState.WAITING_FOR_START
        self.current_command_parts = []
        self.last_activity_time = None
        print("[CommandDetector] Reset to initial state.")

