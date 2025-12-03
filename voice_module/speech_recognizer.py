"""
Speech Recognizer Module

This module provides speech-to-text functionality using Google Speech Recognition API.
"""

import speech_recognition as sr
from typing import Optional
import config


class SpeechRecognizer:
    """Speech recognizer using Google Speech Recognition API (English)."""
    
    def __init__(self, language: str = None):
        """
        Initialize the speech recognizer.
        
        Args:
            language: Language code for speech recognition (default: from config)
        """
        self.recognizer = sr.Recognizer()
        self.language = language or config.VOICE_LANGUAGE
        self.recognizer.energy_threshold = config.VOICE_ENERGY_THRESHOLD
        self.recognizer.dynamic_energy_threshold = config.VOICE_DYNAMIC_ENERGY_THRESHOLD
        
    def recognize(self, audio_data: sr.AudioData) -> Optional[str]:
        """
        Recognize speech from audio data.
        
        Args:
            audio_data: AudioData object from microphone input
            
        Returns:
            Recognized text string, or None if recognition fails
        """
        try:
            print(f"[SpeechRecognizer] Calling Google Speech Recognition API (language: {self.language})...")
            # Use Google Speech Recognition API
            text = self.recognizer.recognize_google(
                audio_data, 
                language=self.language
            )
            print(f"[SpeechRecognizer] ✓ Recognition successful: '{text}'")
            return text
        except sr.UnknownValueError:
            # Speech was unintelligible
            print("[SpeechRecognizer] ⚠ Could not understand audio (UnknownValueError)")
            return None
        except sr.RequestError as e:
            # API was unreachable or unresponsive
            print(f"[SpeechRecognizer] ✗ Error with speech recognition service: {e}")
            return None
        except Exception as e:
            print(f"[SpeechRecognizer] ✗ Unexpected error during speech recognition: {e}")
            return None
    
    def adjust_for_ambient_noise(self, source: sr.Microphone, duration: float = None):
        """
        Adjust recognizer sensitivity for ambient noise.
        
        Args:
            source: Microphone source
            duration: Duration in seconds to listen for ambient noise (default: from config)
        """
        if duration is None:
            duration = config.VOICE_AMBIENT_NOISE_DURATION
        
        print("Adjusting for ambient noise... Please wait.")
        try:
            with source as mic:
                self.recognizer.adjust_for_ambient_noise(mic, duration=duration)
            print(f"Energy threshold set to: {self.recognizer.energy_threshold}")
        except Exception as e:
            print(f"Error adjusting for ambient noise: {e}")

