"""
Unit Tests for Robot Commands (ROSBridge Integration)

This module tests robot command execution via ROSBridge.
Run with: python -m unittest test.test_robot_commands

Prerequisites:
1. ROSBridge connection test passed (run test_rosbridge_connection.py)
2. robot_command_receiver node is running on Raspberry Pi
3. config.py is properly configured
4. ROBOT_DEBUG_MODE = False (for real hardware testing)
"""

import unittest
import time
import config
from executor_module.robot_functions import move_home, move, grasp


class TestRobotCommands(unittest.TestCase):
    """Unit tests for robot command execution via ROSBridge."""
    
    @classmethod
    def setUpClass(cls):
        """Set up test class - run once before all tests."""
        print("\n" + "=" * 60)
        print("=== Robot Commands Unit Tests ===")
        print("=" * 60)
        print(f"Communication Mode: {config.ROBOT_COMMUNICATION_MODE}")
        print(f"Debug Mode: {config.ROBOT_DEBUG_MODE}")
        print(f"ROSBridge Address: {config.ROSBridge_HOST}:{config.ROSBridge_PORT}")
        print("=" * 60)
        
        # Check configuration
        if config.ROBOT_COMMUNICATION_MODE != "rosbridge":
            raise unittest.SkipTest(
                f"ROBOT_COMMUNICATION_MODE is not 'rosbridge' (current: {config.ROBOT_COMMUNICATION_MODE}). "
                "Please set ROBOT_COMMUNICATION_MODE = 'rosbridge' in config.py"
            )
        
        if config.ROBOT_DEBUG_MODE:
            print("\n⚠ Warning: ROBOT_DEBUG_MODE is True")
            print("   Tests will use mock responses, not real communication")
    
    def setUp(self):
        """Set up test fixtures - run before each test."""
        # Small delay between tests to avoid overwhelming the system
        time.sleep(0.5)
    
    def tearDown(self):
        """Clean up after each test."""
        # Small delay after each test
        time.sleep(0.5)
    
    def test_move_home(self):
        """Test move_home command."""
        print("\n--- Testing move_home() ---")
        print(f"Target position: {config.ROBOT_HOME_POSITION}")
        
        result = move_home()
        
        # Assertions
        self.assertIsNotNone(result, "Result should not be None")
        self.assertIn("status", result, "Result should have 'status' field")
        self.assertEqual(result["status"], "success", 
                        f"move_home should succeed. Got: {result.get('message')}")
        self.assertIn("position", result, "Result should have 'position' field")
        self.assertIn("message", result, "Result should have 'message' field")
        
        # Verify position matches home position
        position = result["position"]
        self.assertAlmostEqual(position["x"], config.ROBOT_HOME_POSITION["x"], places=3,
                              msg="X coordinate should match home position")
        self.assertAlmostEqual(position["y"], config.ROBOT_HOME_POSITION["y"], places=3,
                              msg="Y coordinate should match home position")
        self.assertAlmostEqual(position["z"], config.ROBOT_HOME_POSITION["z"], places=3,
                              msg="Z coordinate should match home position")
        
        print(f"✓ move_home test passed: {result.get('message')}")
    
    def test_move(self):
        """Test move command."""
        print("\n--- Testing move() ---")
        test_position = {"x": 0.15, "y": 0.05, "z": 0.20}
        print(f"Target position: {test_position}")
        
        result = move(x=test_position["x"], y=test_position["y"], z=test_position["z"])
        
        # Assertions
        self.assertIsNotNone(result, "Result should not be None")
        self.assertIn("status", result, "Result should have 'status' field")
        self.assertEqual(result["status"], "success",
                        f"move should succeed. Got: {result.get('message')}")
        self.assertIn("position", result, "Result should have 'position' field")
        
        # Verify position matches target
        position = result["position"]
        self.assertAlmostEqual(position["x"], test_position["x"], places=3,
                              msg="X coordinate should match target")
        self.assertAlmostEqual(position["y"], test_position["y"], places=3,
                              msg="Y coordinate should match target")
        self.assertAlmostEqual(position["z"], test_position["z"], places=3,
                              msg="Z coordinate should match target")
        
        print(f"✓ move test passed: {result.get('message')}")
    
    def test_grasp_close(self):
        """Test grasp command (close gripper)."""
        print("\n--- Testing grasp(True) ---")
        
        result = grasp(True)
        
        # Assertions
        self.assertIsNotNone(result, "Result should not be None")
        self.assertIn("status", result, "Result should have 'status' field")
        self.assertEqual(result["status"], "success",
                        f"grasp(True) should succeed. Got: {result.get('message')}")
        self.assertIn("grasp_state", result, "Result should have 'grasp_state' field")
        self.assertTrue(result["grasp_state"], "grasp_state should be True")
        self.assertIn("message", result, "Result should have 'message' field")
        
        print(f"✓ grasp(True) test passed: {result.get('message')}")
    
    def test_grasp_open(self):
        """Test grasp command (open gripper)."""
        print("\n--- Testing grasp(False) ---")
        
        result = grasp(False)
        
        # Assertions
        self.assertIsNotNone(result, "Result should not be None")
        self.assertIn("status", result, "Result should have 'status' field")
        self.assertEqual(result["status"], "success",
                        f"grasp(False) should succeed. Got: {result.get('message')}")
        self.assertIn("grasp_state", result, "Result should have 'grasp_state' field")
        self.assertFalse(result["grasp_state"], "grasp_state should be False")
        self.assertIn("message", result, "Result should have 'message' field")
        
        print(f"✓ grasp(False) test passed: {result.get('message')}")
    
    def test_command_sequence(self):
        """Test a sequence of commands (integration test)."""
        print("\n--- Testing Command Sequence ---")
        print("Sequence: move_home → move → grasp(True) → move → grasp(False) → move_home")
        
        # Step 1: move_home
        print("\n[1/6] Executing move_home()...")
        result = move_home()
        self.assertEqual(result["status"], "success",
                        f"Step 1 (move_home) failed: {result.get('message')}")
        print("✓ Step 1 completed")
        time.sleep(1)
        
        # Step 2: move to target
        print("\n[2/6] Executing move(x=0.15, y=0.05, z=0.20)...")
        result = move(x=0.15, y=0.05, z=0.20)
        self.assertEqual(result["status"], "success",
                        f"Step 2 (move) failed: {result.get('message')}")
        print("✓ Step 2 completed")
        time.sleep(1)
        
        # Step 3: grasp (close)
        print("\n[3/6] Executing grasp(True)...")
        result = grasp(True)
        self.assertEqual(result["status"], "success",
                        f"Step 3 (grasp close) failed: {result.get('message')}")
        print("✓ Step 3 completed")
        time.sleep(1)
        
        # Step 4: move (lift)
        print("\n[4/6] Executing move(x=0.15, y=0.05, z=0.30)...")
        result = move(x=0.15, y=0.05, z=0.30)
        self.assertEqual(result["status"], "success",
                        f"Step 4 (move lift) failed: {result.get('message')}")
        print("✓ Step 4 completed")
        time.sleep(1)
        
        # Step 5: grasp (open)
        print("\n[5/6] Executing grasp(False)...")
        result = grasp(False)
        self.assertEqual(result["status"], "success",
                        f"Step 5 (grasp open) failed: {result.get('message')}")
        print("✓ Step 5 completed")
        time.sleep(1)
        
        # Step 6: move_home
        print("\n[6/6] Executing move_home()...")
        result = move_home()
        self.assertEqual(result["status"], "success",
                        f"Step 6 (move_home) failed: {result.get('message')}")
        print("✓ Step 6 completed")
        
        print("\n✓✓✓ Command sequence test passed!")
    
    def test_move_with_invalid_coordinates(self):
        """Test move command with None coordinates (should handle gracefully)."""
        print("\n--- Testing move() with None coordinates ---")
        
        # This should use default position or handle gracefully
        result = move(x=None, y=None, z=None)
        
        # The function should still return a result (may be error or use defaults)
        self.assertIsNotNone(result, "Result should not be None")
        self.assertIn("status", result, "Result should have 'status' field")
        
        # If it's an error, that's acceptable
        if result["status"] == "error":
            print(f"✓ move with None coordinates correctly returned error: {result.get('message')}")
        else:
            # If it succeeds with defaults, that's also acceptable
            self.assertIn("position", result, "Result should have 'position' field")
            print(f"✓ move with None coordinates used defaults: {result.get('position')}")
    
    def test_result_structure(self):
        """Test that all command results have expected structure."""
        print("\n--- Testing Result Structure ---")
        
        # Test move_home result structure
        result = move_home()
        self.assertIn("status", result)
        self.assertIn("action", result)
        self.assertIn("message", result)
        self.assertIn("position", result)
        print("✓ move_home result structure is correct")
        
        # Test move result structure
        result = move(x=0.15, y=0.05, z=0.20)
        self.assertIn("status", result)
        self.assertIn("action", result)
        self.assertIn("message", result)
        self.assertIn("position", result)
        print("✓ move result structure is correct")
        
        # Test grasp result structure
        result = grasp(True)
        self.assertIn("status", result)
        self.assertIn("action", result)
        self.assertIn("message", result)
        self.assertIn("grasp_state", result)
        print("✓ grasp result structure is correct")


if __name__ == "__main__":
    # Run tests with verbose output
    unittest.main(verbosity=2)

