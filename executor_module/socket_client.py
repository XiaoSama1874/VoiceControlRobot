"""
Socket Client Module

This module provides a TCP socket client for communicating with the Raspberry Pi robot server.
It manages a persistent connection and handles automatic reconnection.
"""

import socket
import json
import time
import os
from typing import Dict, Any, Optional
import config


class SocketClient:
    """Manages persistent TCP connection to robot server."""
    
    def __init__(self):
        """Initialize socket client (does not connect immediately)."""
        self.sock: Optional[socket.socket] = None
        self.host = os.getenv("ME578_RPI_IP_ADDR", config.ROBOT_SOCKET_HOST)
        self.port = config.ROBOT_SOCKET_PORT
        self.timeout = config.ROBOT_SOCKET_TIMEOUT
        self.retry_attempts = config.ROBOT_SOCKET_RETRY_ATTEMPTS
        self.retry_delay = config.ROBOT_SOCKET_RETRY_DELAY
        self._connected = False
    
    def connect(self) -> bool:
        """
        Establish connection to robot server.
        
        Returns:
            True if connection successful, False otherwise
        """
        print(f"[SocketClient] [DEBUG] connect() called - Current state: connected={self._connected}, sock={self.sock is not None}")
        
        if self._connected and self.sock is not None:
            try:
                # Test if connection is still alive
                print(f"[SocketClient] [DEBUG] Testing existing connection...")
                self.sock.settimeout(0.1)
                self.sock.recv(1, socket.MSG_PEEK)
                self.sock.settimeout(self.timeout)
                print(f"[SocketClient] [DEBUG] ✓ Existing connection is alive")
                return True
            except (socket.error, OSError) as e:
                # Connection is dead, close it
                print(f"[SocketClient] [DEBUG] ✗ Existing connection is dead: {e}")
                self._close_socket()
        
        print(f"[SocketClient] [DEBUG] Starting new connection attempt...")
        print(f"[SocketClient] [DEBUG] Connection parameters: host={self.host}, port={self.port}, timeout={self.timeout}s")
        
        for attempt in range(self.retry_attempts):
            try:
                print(f"[SocketClient] [DEBUG] Attempt {attempt + 1}/{self.retry_attempts}: Creating socket...")
                print(f"[SocketClient] Connecting to {self.host}:{self.port} (attempt {attempt + 1}/{self.retry_attempts})...")
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(self.timeout)
                print(f"[SocketClient] [DEBUG] Socket created, attempting connection...")
                connect_start = time.time()
                self.sock.connect((self.host, self.port))
                connect_time = time.time() - connect_start
                self._connected = True
                print(f"[SocketClient] [DEBUG] ✓ Connection established in {connect_time:.3f}s")
                print(f"[SocketClient] ✓ Connected successfully to {self.host}:{self.port}")
                return True
            except socket.timeout:
                print(f"[SocketClient] [DEBUG] ✗ Connection timeout after {self.timeout}s")
                print(f"[SocketClient] ✗ Connection timeout (attempt {attempt + 1}/{self.retry_attempts})")
                self._close_socket()
            except (socket.error, OSError) as e:
                print(f"[SocketClient] [DEBUG] ✗ Connection error: {type(e).__name__}: {e}")
                print(f"[SocketClient] ✗ Connection failed (attempt {attempt + 1}/{self.retry_attempts}): {e}")
                self._close_socket()
            
            if attempt < self.retry_attempts - 1:
                print(f"[SocketClient] [DEBUG] Waiting {self.retry_delay}s before retry...")
                time.sleep(self.retry_delay)
        
        print(f"[SocketClient] [DEBUG] ✗✗✗ All connection attempts exhausted")
        print(f"[SocketClient] ✗✗✗ Failed to connect after {self.retry_attempts} attempts")
        return False
    
    def send_command(self, json_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Send JSON command and receive response.
        
        Args:
            json_data: Dictionary to send as JSON
            
        Returns:
            Response dictionary if successful, None if failed
        """
        print(f"[SocketClient] [DEBUG] ===== send_command() START =====")
        print(f"[SocketClient] [DEBUG] Command data: {json_data}")
        
        # Ensure connection
        if not self.is_connected():
            print(f"[SocketClient] [DEBUG] Not connected, attempting connection...")
            if not self.connect():
                print(f"[SocketClient] [DEBUG] ✗ Connection failed, aborting send_command")
                return None
        else:
            print(f"[SocketClient] [DEBUG] ✓ Already connected")
        
        # Try to send command with retry on failure
        for attempt in range(self.retry_attempts):
            try:
                print(f"[SocketClient] [DEBUG] --- Attempt {attempt + 1}/{self.retry_attempts} ---")
                
                # Send JSON command
                json_str = json.dumps(json_data, ensure_ascii=False)
                json_bytes = json_str.encode('utf-8')
                json_size = len(json_bytes)
                
                print(f"[SocketClient] [DEBUG] Serialized JSON: {json_str}")
                print(f"[SocketClient] [DEBUG] JSON size: {json_size} bytes")
                print(f"[SocketClient] Sending command: {json_str}")
                
                send_start = time.time()
                self.sock.sendall(json_bytes)
                send_time = time.time() - send_start
                print(f"[SocketClient] [DEBUG] ✓ Data sent in {send_time:.3f}s ({json_size} bytes)")
                
                # Receive response
                print(f"[SocketClient] [DEBUG] Waiting for response (timeout: {self.timeout}s)...")
                recv_start = time.time()
                response_data = self.sock.recv(1024)
                recv_time = time.time() - recv_start
                response_size = len(response_data)
                
                print(f"[SocketClient] [DEBUG] Response received in {recv_time:.3f}s ({response_size} bytes)")
                
                if not response_data:
                    print("[SocketClient] [DEBUG] ✗ Empty response - server closed connection")
                    print("[SocketClient] ✗ Server closed connection")
                    self._close_socket()
                    if attempt < self.retry_attempts - 1:
                        print(f"[SocketClient] [DEBUG] Attempting reconnection for retry...")
                        if self.connect():
                            continue
                    return None
                
                # Parse JSON response
                response_str = response_data.decode('utf-8')
                print(f"[SocketClient] [DEBUG] Decoded response string: {response_str}")
                print(f"[SocketClient] Received response: {response_str}")
                
                parse_start = time.time()
                response = json.loads(response_str)
                parse_time = time.time() - parse_start
                print(f"[SocketClient] [DEBUG] ✓ JSON parsed in {parse_time:.3f}s")
                print(f"[SocketClient] [DEBUG] Parsed response: {response}")
                
                total_time = time.time() - send_start
                print(f"[SocketClient] [DEBUG] ✓✓✓ Command completed successfully in {total_time:.3f}s total")
                print(f"[SocketClient] [DEBUG] ===== send_command() SUCCESS =====")
                return response
                
            except socket.timeout:
                print(f"[SocketClient] [DEBUG] ✗ Receive timeout after {self.timeout}s")
                print(f"[SocketClient] ✗ Receive timeout (attempt {attempt + 1}/{self.retry_attempts})")
                if attempt < self.retry_attempts - 1:
                    print(f"[SocketClient] [DEBUG] Closing socket and attempting reconnection...")
                    self._close_socket()
                    if self.connect():
                        print(f"[SocketClient] [DEBUG] Reconnected, retrying command...")
                        continue
                return None
            except (socket.error, OSError) as e:
                print(f"[SocketClient] [DEBUG] ✗ Socket error: {type(e).__name__}: {e}")
                print(f"[SocketClient] ✗ Socket error (attempt {attempt + 1}/{self.retry_attempts}): {e}")
                self._close_socket()
                if attempt < self.retry_attempts - 1:
                    print(f"[SocketClient] [DEBUG] Attempting reconnection...")
                    if self.connect():
                        print(f"[SocketClient] [DEBUG] Reconnected, retrying command...")
                        continue
                return None
            except json.JSONDecodeError as e:
                print(f"[SocketClient] [DEBUG] ✗ JSON decode error: {e}")
                print(f"[SocketClient] [DEBUG] Error position: line {e.lineno}, column {e.colno}")
                print(f"[SocketClient] ✗ Invalid JSON response: {e}")
                raw_response = response_data.decode('utf-8', errors='ignore')
                print(f"[SocketClient] [DEBUG] Raw response (first 200 chars): {raw_response[:200]}")
                print(f"[SocketClient] Raw response: {raw_response}")
                return None
            except Exception as e:
                print(f"[SocketClient] [DEBUG] ✗ Unexpected error: {type(e).__name__}: {e}")
                import traceback
                print(f"[SocketClient] [DEBUG] Traceback:\n{traceback.format_exc()}")
                print(f"[SocketClient] ✗ Unexpected error: {e}")
                return None
        
        print(f"[SocketClient] [DEBUG] ✗✗✗ All send attempts exhausted")
        print(f"[SocketClient] [DEBUG] ===== send_command() FAILED =====")
        return None
    
    def disconnect(self):
        """Close connection to server."""
        print(f"[SocketClient] [DEBUG] disconnect() called")
        self._close_socket()
        print("[SocketClient] Disconnected")
    
    def is_connected(self) -> bool:
        """
        Check if connected to server.
        
        Returns:
            True if connected, False otherwise
        """
        result = self._connected and self.sock is not None
        print(f"[SocketClient] [DEBUG] is_connected() = {result} (connected={self._connected}, sock={self.sock is not None})")
        return result
    
    def _close_socket(self):
        """Close socket connection (internal method)."""
        print(f"[SocketClient] [DEBUG] _close_socket() called")
        if self.sock is not None:
            try:
                print(f"[SocketClient] [DEBUG] Closing socket...")
                self.sock.close()
                print(f"[SocketClient] [DEBUG] ✓ Socket closed")
            except Exception as e:
                print(f"[SocketClient] [DEBUG] ⚠ Error closing socket: {e}")
            self.sock = None
        self._connected = False
        print(f"[SocketClient] [DEBUG] Connection state reset: connected=False")


# Global socket client instance (singleton pattern)
_socket_client: Optional[SocketClient] = None


def get_socket_client() -> SocketClient:
    """
    Get global socket client instance.
    
    Returns:
        SocketClient instance
    """
    global _socket_client
    if _socket_client is None:
        _socket_client = SocketClient()
    return _socket_client

