"""
ROSBridge Client Module

This module provides a ROSBridge client for communicating with the Raspberry Pi robot server.
It uses roslibpy to connect to ROSBridge WebSocket server and communicates via ROS topics.
"""

import json
import time
import threading
from typing import Dict, Any, Optional
import os
import config

try:
    import roslibpy
except ImportError:
    roslibpy = None
    print("[ROSBridgeClient] Warning: roslibpy not installed. Install with: pip install roslibpy")


class ROSBridgeClient:
    """Manages ROSBridge connection to robot server via ROS topics."""
    
    def __init__(self):
        """Initialize ROSBridge client (does not connect immediately)."""
        self.ros_client: Optional[Any] = None
        self.host = os.getenv("ME578_RPI_IP_ADDR", config.ROSBridge_HOST)
        self.port = config.ROSBridge_PORT
        self.timeout = config.ROSBridge_TIMEOUT
        self.retry_attempts = config.ROBOT_SOCKET_RETRY_ATTEMPTS
        self.retry_delay = config.ROBOT_SOCKET_RETRY_DELAY
        self._connected = False
        
        # Topic names
        self.command_topic = "/robot_commands"
        self.response_topic = "/robot_command_response"
        
        # Response handling
        self.response_event = threading.Event()
        self.last_response: Optional[Dict[str, Any]] = None
        self.response_lock = threading.Lock()
        
        # Publisher and subscriber
        self.command_publisher: Optional[Any] = None
        self.response_subscriber: Optional[Any] = None
    
    def connect(self) -> bool:
        """
        Establish connection to ROSBridge server.
        
        Returns:
            True if connection successful, False otherwise
        """
        if roslibpy is None:
            print("[ROSBridgeClient] ✗ roslibpy not available. Cannot connect.")
            return False
        
        print(f"[ROSBridgeClient] [DEBUG] connect() called - Current state: connected={self._connected}")
        
        if self._connected and self.ros_client is not None:
            try:
                # Test if connection is still alive
                if self.ros_client.is_connected:
                    print(f"[ROSBridgeClient] [DEBUG] ✓ Existing connection is alive")
                    return True
                else:
                    print(f"[ROSBridgeClient] [DEBUG] ✗ Existing connection is dead")
                    self._close_connection()
            except Exception as e:
                print(f"[ROSBridgeClient] [DEBUG] ✗ Error checking connection: {e}")
                self._close_connection()
        
        print(f"[ROSBridgeClient] [DEBUG] Starting new connection attempt...")
        print(f"[ROSBridgeClient] [DEBUG] Connection parameters: host={self.host}, port={self.port}, timeout={self.timeout}s")
        
        for attempt in range(self.retry_attempts):
            try:
                print(f"[ROSBridgeClient] [DEBUG] Attempt {attempt + 1}/{self.retry_attempts}: Creating ROSBridge client...")
                print(f"[ROSBridgeClient] Connecting to ROSBridge at ws://{self.host}:{self.port} (attempt {attempt + 1}/{self.retry_attempts})...")
                
                # Create ROSBridge client
                self.ros_client = roslibpy.Ros(host=self.host, port=self.port)
                
                # Connect (non-blocking)
                self.ros_client.connect()
                
                # Wait for connection to be established
                connect_start = time.time()
                while not self.ros_client.is_connected:
                    if (time.time() - connect_start) >= self.timeout:
                        raise ConnectionError("Failed to establish ROSBridge connection within timeout")
                    time.sleep(0.1)
                
                connect_time = time.time() - connect_start
                print(f"[ROSBridgeClient] [DEBUG] ✓ Connection established in {connect_time:.3f}s")
                
                # Set up publisher and subscriber
                self.command_publisher = roslibpy.Topic(self.ros_client, self.command_topic, 'std_msgs/String')
                self.command_publisher.advertise()
                
                self.response_subscriber = roslibpy.Topic(self.ros_client, self.response_topic, 'std_msgs/String')
                self.response_subscriber.subscribe(self._on_response_received)
                
                self._connected = True
                print(f"[ROSBridgeClient] ✓ Connected successfully to ROSBridge at {self.host}:{self.port}")
                print(f"[ROSBridgeClient] ✓ Publisher: {self.command_topic}")
                print(f"[ROSBridgeClient] ✓ Subscriber: {self.response_topic}")
                return True
                
            except Exception as e:
                print(f"[ROSBridgeClient] [DEBUG] ✗ Connection error: {type(e).__name__}: {e}")
                print(f"[ROSBridgeClient] ✗ Connection failed (attempt {attempt + 1}/{self.retry_attempts}): {e}")
                self._close_connection()
                
                if attempt < self.retry_attempts - 1:
                    print(f"[ROSBridgeClient] [DEBUG] Waiting {self.retry_delay}s before retry...")
                    time.sleep(self.retry_delay)
        
        print(f"[ROSBridgeClient] [DEBUG] ✗✗✗ All connection attempts exhausted")
        print(f"[ROSBridgeClient] ✗✗✗ Failed to connect after {self.retry_attempts} attempts")
        return False
    
    def _on_response_received(self, message: Dict[str, Any]):
        """
        Callback for receiving response messages.
        
        Args:
            message: ROS message containing response JSON string
        """
        try:
            response_str = message.get('data', '')
            print(f"[ROSBridgeClient] [DEBUG] Response received: {response_str}")
            
            # Parse JSON response
            response_json = json.loads(response_str)
            
            with self.response_lock:
                self.last_response = response_json
                self.response_event.set()
                
        except json.JSONDecodeError as e:
            print(f"[ROSBridgeClient] [DEBUG] ✗ Invalid JSON in response: {e}")
            with self.response_lock:
                self.last_response = {"status": "error", "message": f"Invalid JSON response: {e}"}
                self.response_event.set()
        except Exception as e:
            print(f"[ROSBridgeClient] [DEBUG] ✗ Error processing response: {e}")
            with self.response_lock:
                self.last_response = {"status": "error", "message": f"Error processing response: {e}"}
                self.response_event.set()
    
    def send_command(self, json_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Send JSON command and receive response.
        
        Args:
            json_data: Dictionary to send as JSON
            
        Returns:
            Response dictionary if successful, None if failed
        """
        print(f"[ROSBridgeClient] [DEBUG] ===== send_command() START =====")
        print(f"[ROSBridgeClient] [DEBUG] Command data: {json_data}")
        
        # Ensure connection
        if not self.is_connected():
            print(f"[ROSBridgeClient] [DEBUG] Not connected. Attempting to connect...")
            if not self.connect():
                print(f"[ROSBridgeClient] [DEBUG] ✗ Failed to connect, cannot send command.")
                print(f"[ROSBridgeClient] [DEBUG] ===== send_command() FAILED (No Connection) =====")
                return None
        
        # Reset response state
        with self.response_lock:
            self.last_response = None
            self.response_event.clear()
        
        # Try to send command with retry on failure
        for attempt in range(self.retry_attempts):
            try:
                print(f"[ROSBridgeClient] [DEBUG] --- Attempt {attempt + 1}/{self.retry_attempts} ---")
                
                # Serialize JSON command
                json_str = json.dumps(json_data, ensure_ascii=False)
                print(f"[ROSBridgeClient] [DEBUG] Serialized JSON: {json_str}")
                print(f"[ROSBridgeClient] Sending command: {json_str}")
                
                # Publish command
                send_start = time.time()
                message = roslibpy.Message({'data': json_str})
                self.command_publisher.publish(message)
                send_time = time.time() - send_start
                print(f"[ROSBridgeClient] [DEBUG] ✓ Command published in {send_time:.3f}s")
                
                # Wait for response with timeout
                print(f"[ROSBridgeClient] [DEBUG] Waiting for response (timeout: {self.timeout}s)...")
                response_received = self.response_event.wait(timeout=self.timeout)
                
                if not response_received:
                    print(f"[ROSBridgeClient] [DEBUG] ✗ Response timeout after {self.timeout}s")
                    if attempt < self.retry_attempts - 1:
                        print(f"[ROSBridgeClient] [DEBUG] Retrying (attempt {attempt + 2}/{self.retry_attempts})...")
                        time.sleep(self.retry_delay)
                        continue
                    print(f"[ROSBridgeClient] [DEBUG] ===== send_command() FAILED (Timeout) =====")
                    return None
                
                # Get response
                with self.response_lock:
                    response = self.last_response.copy() if self.last_response else None
                
                if response:
                    print(f"[ROSBridgeClient] [DEBUG] ✓✓✓ Command completed successfully")
                    print(f"[ROSBridgeClient] [DEBUG] Response: {response}")
                    print(f"[ROSBridgeClient] [DEBUG] ===== send_command() SUCCESS =====")
                    return response
                else:
                    print(f"[ROSBridgeClient] [DEBUG] ✗ No response received")
                    if attempt < self.retry_attempts - 1:
                        print(f"[ROSBridgeClient] [DEBUG] Retrying (attempt {attempt + 2}/{self.retry_attempts})...")
                        time.sleep(self.retry_delay)
                        continue
                    print(f"[ROSBridgeClient] [DEBUG] ===== send_command() FAILED (No Response) =====")
                    return None
                    
            except Exception as e:
                print(f"[ROSBridgeClient] [DEBUG] ✗ Error during send_command (attempt {attempt + 1}/{self.retry_attempts}): {e}")
                import traceback
                traceback.print_exc()
                
                # Check if connection is still alive
                if self.ros_client is None or not self.ros_client.is_connected:
                    print(f"[ROSBridgeClient] [DEBUG] Connection lost, attempting reconnect...")
                    self._close_connection()
                    if attempt < self.retry_attempts - 1:
                        if self.connect():
                            continue
                
                if attempt < self.retry_attempts - 1:
                    print(f"[ROSBridgeClient] [DEBUG] Retrying (attempt {attempt + 2}/{self.retry_attempts})...")
                    time.sleep(self.retry_delay)
                    continue
                
                print(f"[ROSBridgeClient] [DEBUG] ===== send_command() FAILED (Exception) =====")
                return None
        
        print(f"[ROSBridgeClient] [DEBUG] ✗✗✗ All {self.retry_attempts} attempts failed.")
        print(f"[ROSBridgeClient] [DEBUG] ===== send_command() FAILED (Max Retries) =====")
        return None
    
    def is_connected(self) -> bool:
        """
        Check if client is connected.
        
        Returns:
            True if connected, False otherwise
        """
        if self.ros_client is None:
            return False
        try:
            return self.ros_client.is_connected and self._connected
        except:
            return False
    
    def disconnect(self):
        """Disconnect from ROSBridge server."""
        self._close_connection()
    
    def _close_connection(self):
        """Close ROSBridge connection."""
        try:
            if self.response_subscriber is not None:
                self.response_subscriber.unsubscribe()
                self.response_subscriber = None
            
            if self.command_publisher is not None:
                self.command_publisher.unadvertise()
                self.command_publisher = None
            
            if self.ros_client is not None:
                if self.ros_client.is_connected:
                    self.ros_client.terminate()
                self.ros_client = None
        except Exception as e:
            print(f"[ROSBridgeClient] [DEBUG] Error closing connection: {e}")
        finally:
            self._connected = False
            print(f"[ROSBridgeClient] [DEBUG] Connection closed")


# Singleton instance
_rosbridge_client_instance: Optional[ROSBridgeClient] = None


def get_rosbridge_client() -> ROSBridgeClient:
    """
    Get singleton ROSBridge client instance.
    
    Returns:
        ROSBridgeClient instance
    """
    global _rosbridge_client_instance
    if _rosbridge_client_instance is None:
        _rosbridge_client_instance = ROSBridgeClient()
    return _rosbridge_client_instance

