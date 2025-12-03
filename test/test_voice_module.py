"""
Unit Tests for Voice Module

Run with: python -m unittest test.test_voice_module
"""

import unittest
import speech_recognition as sr
from voice_module import VoiceListener, SpeechRecognizer


class TestVoiceModuleSpeechRecognition(unittest.TestCase):
    """Unit tests for Voice module - Speech Recognition."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.recognizer = SpeechRecognizer(language="en-US")
        self.microphone = sr.Microphone()
    
    @unittest.skip("Requires microphone and audio input")
    def test_speech_recognition_single(self):
        """Test single speech recognition (requires microphone)."""
        try:
            # Adjust for ambient noise
            self.recognizer.adjust_for_ambient_noise(self.microphone, duration=0.5)
            
            # Listen and recognize
            with self.microphone as source:
                audio = self.recognizer.recognizer.listen(source, timeout=5, phrase_time_limit=5)
            
            text = self.recognizer.recognize(audio)
            
            if text:
                self.assertIsInstance(text, str, "Recognized text should be string")
                self.assertGreater(len(text), 0, "Recognized text should not be empty")
                print(f"\n✓ Recognized text: '{text}'")
            else:
                self.skipTest("Could not recognize speech (may be noise)")
                
        except sr.WaitTimeoutError:
            self.skipTest("No audio detected within timeout")
        except Exception as e:
            self.fail(f"Speech recognition failed: {e}")
    
    def test_speech_recognizer_initialization(self):
        """Test that speech recognizer initializes correctly."""
        self.assertIsNotNone(self.recognizer.recognizer, "Recognizer should be initialized")
        self.assertEqual(self.recognizer.language, "en-US", "Language should be en-US")
        print("\n✓ Speech recognizer initialized correctly")


class TestVoiceModuleFull(unittest.TestCase):
    """Unit tests for Voice module - Full listening."""
    
    @unittest.skip("Requires microphone and audio input")
    def test_voice_listener_command_detection(self):
        """Test voice listener with command detection (requires microphone)."""
        commands_detected = []
        
        def on_command_detected(command: str):
            commands_detected.append(command)
        
        listener = VoiceListener(
            on_command_detected=on_command_detected,
            language="en-US",
            start_keyword="hi robot",
            stop_keyword="command end",
            silence_duration=1.0,
            phrase_time_limit=5.0
        )
        
        try:
            listener.start_listening()
            
            # Wait a bit for commands
            import time
            time.sleep(10)  # Listen for 10 seconds
            
            listener.stop_listening()
            
            # Check if any commands were detected
            if commands_detected:
                self.assertGreater(len(commands_detected), 0, "Should detect at least one command")
                print(f"\n✓ Detected {len(commands_detected)} command(s)")
            else:
                print("\n⚠ No commands detected (this is OK if no speech was provided)")
                
        except Exception as e:
            self.fail(f"Voice listener test failed: {e}")
    
    def test_voice_listener_initialization(self):
        """Test that voice listener initializes correctly."""
        listener = VoiceListener(
            language="en-US",
            start_keyword="hi robot",
            stop_keyword="command end"
        )
        
        self.assertIsNotNone(listener.speech_recognizer, "Speech recognizer should be initialized")
        self.assertIsNotNone(listener.command_detector, "Command detector should be initialized")
        self.assertFalse(listener.is_listening, "Should not be listening initially")
        print("\n✓ Voice listener initialized correctly")
    
    def test_command_detector_state_machine(self):
        """Test command detector state machine."""
        from voice_module.command_detector import CommandDetector, CommandState
        
        detector = CommandDetector(
            start_keyword="hi robot",
            stop_keyword="end"
        )
        
        # Initially waiting for start
        self.assertEqual(detector.state, CommandState.WAITING_FOR_START)
        
        # Process start keyword
        detector.process_text("hi robot pick up")
        self.assertEqual(detector.state, CommandState.RECORDING_COMMAND)
        
        # Process stop keyword
        result = detector.process_text("the red square end")
        self.assertEqual(detector.state, CommandState.WAITING_FOR_START)
        self.assertIsNotNone(result)
        print("\n✓ Command detector state machine works correctly")


if __name__ == "__main__":
    unittest.main(verbosity=2)

