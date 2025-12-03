"""
Voice Listener Module

This module provides continuous voice listening with silence detection and command detection.
"""

import speech_recognition as sr
import threading
from typing import Optional, Callable
from .speech_recognizer import SpeechRecognizer
from .command_detector import CommandDetector
import config


class VoiceListener:
    """Continuous voice listener with command detection."""
    
    def __init__(
        self,
        on_command_detected: Optional[Callable[[str], None]] = None,
        language: str = None,
        start_keyword: str = None,
        stop_keyword: str = None,
        silence_duration: float = None,
        phrase_time_limit: float = None
    ):
        """
        Initialize the voice listener.
        
        Args:
            on_command_detected: Callback function called when a complete command is detected
            language: Language code for speech recognition (default: from config)
            start_keyword: Keyword to start command recording (default: from config)
            stop_keyword: Keyword to stop command recording (default: from config)
            silence_duration: Duration of silence to consider end of phrase (default: from config)
            phrase_time_limit: Maximum time to wait for a phrase (default: from config)
        """
        self.speech_recognizer = SpeechRecognizer(language=language or config.VOICE_LANGUAGE)
        self.command_detector = CommandDetector(
            start_keyword=start_keyword or config.VOICE_START_KEYWORD,
            stop_keyword=stop_keyword or config.VOICE_STOP_KEYWORD,
            on_command_detected=on_command_detected
        )
        self.microphone = sr.Microphone()
        self.is_listening = False
        self.listening_thread: Optional[threading.Thread] = None
        self.silence_check_thread: Optional[threading.Thread] = None
        self.silence_duration = silence_duration or config.VOICE_SILENCE_DURATION
        self.phrase_time_limit = phrase_time_limit or config.VOICE_PHRASE_TIME_LIMIT
        self.command_end_silence_duration = config.VOICE_COMMAND_END_SILENCE_DURATION
        
    def start_listening(self):
        """Start continuous voice listening in a separate thread."""
        if self.is_listening:
            print("[VoiceListener] Already listening.")
            return
        
        # Adjust for ambient noise
        self.speech_recognizer.adjust_for_ambient_noise(
            self.microphone, 
            duration=config.VOICE_AMBIENT_NOISE_DURATION
        )
        
        self.is_listening = True
        self.listening_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.listening_thread.start()
        
        # Start silence check thread
        self.silence_check_thread = threading.Thread(target=self._silence_check_loop, daemon=True)
        self.silence_check_thread.start()
        
        print("[VoiceListener] Started listening. Say 'Hi Robot.' to begin a command.")
        
    def stop_listening(self):
        """Stop continuous voice listening."""
        if not self.is_listening:
            return
        
        self.is_listening = False
        if self.listening_thread:
            self.listening_thread.join(timeout=2.0)
        if self.silence_check_thread:
            self.silence_check_thread.join(timeout=2.0)
        print("[VoiceListener] Stopped listening.")
        
    def _listen_loop(self):
        """Main listening loop running in a separate thread."""
        try:
            with self.microphone as source:
                while self.is_listening:
                    try:
                        # Listen for audio with silence detection
                        audio = self.speech_recognizer.recognizer.listen(
                            source,
                            timeout=None,
                            phrase_time_limit=self.phrase_time_limit
                        )
                        
                        # Recognize speech
                        print("[VoiceListener] Processing audio...")
                        text = self.speech_recognizer.recognize(audio)
                        
                        if text:
                            print(f"[VoiceListener] ✓ Speech recognized: '{text}'")
                            
                            # Process text through command detector
                            command = self.command_detector.process_text(text)
                            
                            if command:
                                print(f"[VoiceListener] ✓✓ Complete command detected: '{command}'")
                            else:
                                print(f"[VoiceListener] → Command not yet complete, waiting for more input...")
                        else:
                            print("[VoiceListener] ⚠ Could not recognize speech (may be noise or unclear audio)")
                                
                    except sr.WaitTimeoutError:
                        # Timeout waiting for audio, continue listening
                        continue
                    except Exception as e:
                        print(f"[VoiceListener] Error in listening loop: {e}")
                        continue
                        
        except Exception as e:
            print(f"[VoiceListener] Fatal error in listening thread: {e}")
            self.is_listening = False
    
    def _silence_check_loop(self):
        """Background thread that checks for silence timeout."""
        import time
        check_interval = 0.5  # Check every 0.5 seconds
        
        while self.is_listening:
            try:
                # Check if silence timeout has been reached
                command = self.command_detector.check_silence_timeout()
                if command:
                    print(f"[VoiceListener] ✓✓ Command completed via silence detection: '{command}'")
                
                time.sleep(check_interval)
            except Exception as e:
                print(f"[VoiceListener] Error in silence check loop: {e}")
                time.sleep(check_interval)
    
    def reset_detector(self):
        """Reset the command detector to initial state."""
        self.command_detector.reset()

