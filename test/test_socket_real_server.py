"""
Integration Tests for Socket Client with Real Raspberry Pi Server

This test class directly tests communication with the real robot server.
It will attempt to connect and send commands to the server configured in config.py.

Run with: python -m unittest test.test_socket_real_server
"""

import unittest
import os

from executor_module.socket_client import SocketClient, get_socket_client
from executor_module.robot_functions import move_home, move, grasp
import config


class TestRealServerConnection(unittest.TestCase):
    """Integration tests with real Raspberry Pi server."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.client = SocketClient()
        print(f"\n[Test] Testing connection to {self.client.host}:{self.client.port}")
        print(f"[Test] Using config: ROBOT_SOCKET_HOST={config.ROBOT_SOCKET_HOST}, PORT={config.ROBOT_SOCKET_PORT}")
    
    def tearDown(self):
        """Clean up after tests."""
        if self.client.is_connected():
            self.client.disconnect()
    
    def test_connect_to_server(self):
        """Test connecting to real server."""
        print("\n[Test] === Testing Connection ===")
        result = self.client.connect()
        
        if result:
            print(f"[Test] ✓ Successfully connected to {self.client.host}:{self.client.port}")
            self.assertTrue(self.client.is_connected())
        else:
            print(f"[Test] ✗ Failed to connect to {self.client.host}:{self.client.port}")
            print("[Test] Please check:")
            print("  1. Server is running on Raspberry Pi")
            print("  2. IP address is correct")
            print("  3. Port 5005 is open")
            print("  4. Network connectivity")
            # Don't fail the test, just report
            self.skipTest("Server not available")
    
    def test_send_move_home_command(self):
        """Test sending move_home command to real server."""
        print("\n[Test] === Testing move_home() ===")
        
        result = move_home()
        
        print(f"[Test] Result: {result}")
        self.assertIsNotNone(result)
        self.assertIn("status", result)
        
        if result.get("status") == "success":
            print("[Test] ✓ move_home() succeeded")
        else:
            print(f"[Test] ✗ move_home() failed: {result.get('message', 'Unknown error')}")
    
    def test_send_move_command(self):
        """Test sending move command to real server."""
        print("\n[Test] === Testing move() ===")
        
        # 0.2271, -0.1448, 0.0360
        # Test with specific coordinates
        test_x = 0.2271
        test_y = -0.1448
        test_z = 0.0360
        
        print(f"[Test] Moving to ({test_x}, {test_y}, {test_z})")
        result = move(x=test_x, y=test_y, z=test_z)
        # result = move_home()
        
        print(f"[Test] Result: {result}")
        self.assertIsNotNone(result)
        self.assertIn("status", result)
        
        if result.get("status") == "success":
            print("[Test] ✓ move() succeeded")
            self.assertEqual(result.get("position", {}).get("x"), test_x)
            self.assertEqual(result.get("position", {}).get("y"), test_y)
            self.assertEqual(result.get("position", {}).get("z"), test_z)
        else:
            print(f"[Test] ✗ move() failed: {result.get('message', 'Unknown error')}")
    
    def test_send_grasp_command(self):
        """Test sending grasp command to real server."""
        print("\n[Test] === Testing grasp() ===")
        
        # Test grasp (close)
        print("[Test] Testing grasp=True (close gripper)")
        result = grasp(True)
        
        print(f"[Test] Result: {result}")
        self.assertIsNotNone(result)
        self.assertIn("status", result)
        
        if result.get("status") == "success":
            print("[Test] ✓ grasp(True) succeeded")
        else:
            print(f"[Test] ✗ grasp(True) failed: {result.get('message', 'Unknown error')}")
        
        # # Test release (open)
        # print("[Test] Testing grasp=False (open gripper)")
        # result = grasp(False)
        
        # print(f"[Test] Result: {result}")
        # self.assertIsNotNone(result)
        # self.assertIn("status", result)
        
        # if result.get("status") == "success":
        #     print("[Test] ✓ grasp(False) succeeded")
        # else:
        #     print(f"[Test] ✗ grasp(False) failed: {result.get('message', 'Unknown error')}")
    
    def test_multiple_commands_sequence(self):
        """Test sending multiple commands in sequence."""
        print("\n[Test] === Testing Command Sequence ===")
        
        commands = [
            ("move_home", lambda: move_home()),
            ("move to (15, 25, 35)", lambda: move(x=15.0, y=25.0, z=35.0)),
            ("grasp", lambda: grasp(True)),
            ("move to (20, 30, 40)", lambda: move(x=20.0, y=30.0, z=40.0)),
            ("release", lambda: grasp(False)),
        ]
        
        results = []
        for cmd_name, cmd_func in commands:
            print(f"\n[Test] Executing: {cmd_name}")
            result = cmd_func()
            results.append((cmd_name, result))
            
            print(f"[Test] Result: status={result.get('status')}, message={result.get('message', 'N/A')}")
            
            if result.get("status") != "success":
                print(f"[Test] ⚠ Command '{cmd_name}' returned error status")
        
        # Summary
        print("\n[Test] === Command Sequence Summary ===")
        success_count = sum(1 for _, r in results if r.get("status") == "success")
        print(f"[Test] Successful commands: {success_count}/{len(commands)}")
        
        for cmd_name, result in results:
            status_icon = "✓" if result.get("status") == "success" else "✗"
            print(f"[Test] {status_icon} {cmd_name}: {result.get('status', 'unknown')}")
    
    def test_connection_persistence(self):
        """Test that connection persists across multiple commands."""
        print("\n[Test] === Testing Connection Persistence ===")
        
        # First command should establish connection
        print("[Test] Sending first command (should establish connection)...")
        result1 = move_home()
        connected_after_first = self.client.is_connected()
        print(f"[Test] Connected after first command: {connected_after_first}")
        
        # Second command should reuse connection
        print("[Test] Sending second command (should reuse connection)...")
        result2 = move(x=10.0, y=10.0, z=10.0)
        connected_after_second = self.client.is_connected()
        print(f"[Test] Connected after second command: {connected_after_second}")
        
        # Third command should still reuse connection
        print("[Test] Sending third command (should still reuse connection)...")
        result3 = grasp(True)
        connected_after_third = self.client.is_connected()
        print(f"[Test] Connected after third command: {connected_after_third}")
        
        self.assertTrue(connected_after_first, "Connection should be established after first command")
        self.assertTrue(connected_after_second, "Connection should persist after second command")
        self.assertTrue(connected_after_third, "Connection should persist after third command")
        
        print("[Test] ✓ Connection persistence verified")
    
    def test_error_handling(self):
        """Test error handling with invalid commands."""
        print("\n[Test] === Testing Error Handling ===")
        
        # Test with None values (should use defaults or fail gracefully)
        print("[Test] Testing move() with None values...")
        result = move(x=None, y=None, z=None)
        print(f"[Test] Result: {result}")
        self.assertIsNotNone(result)
        self.assertIn("status", result)
        
        # Test connection failure scenario (by temporarily using wrong port)
        print("[Test] Testing connection failure handling...")
        original_port = self.client.port
        self.client.port = 9999  # Invalid port
        self.client.disconnect()
        
        result = move_home()
        print(f"[Test] Result with invalid port: {result}")
        self.assertIsNotNone(result)
        # Should return error status
        self.assertIn("status", result)
        
        # Restore port
        self.client.port = original_port


if __name__ == '__main__':
    print("="*60)
    print("Real Server Integration Tests")
    print("="*60)
    print(f"Target Server: {config.ROBOT_SOCKET_HOST}:{config.ROBOT_SOCKET_PORT}")
    print(f"Environment Variable ME578_RPI_IP_ADDR: {os.getenv('ME578_RPI_IP_ADDR', 'Not set')}")
    print("="*60)
    print("\nNote: These tests require the Raspberry Pi server to be running.")
    print("If tests fail, check server status and network connectivity.\n")
    
    unittest.main(verbosity=2)

