#!/usr/bin/env python3

"""
ROS2 Node: Robot Command Receiver

This node receives robot commands via ROSBridge from the Mac client,
executes them, and publishes responses back.

Commands are received on /robot_commands topic (std_msgs/String, JSON format).
Responses are published on /robot_command_response topic (std_msgs/String, JSON format).

Supported actions:
- move: Move robot arm to specified coordinates (including home position)
- grasp: Open/close gripper

Note: move_home is handled by Mac side by converting it to a move command with home coordinates.
"""

import rclpy
from rclpy.node import Node
import json
import traceback
import time
import threading
import numpy as np
from std_msgs.msg import String, Bool
from xarmrob_interfaces.msg import ME439PointXYZ

import xarmrob.smooth_interpolation as smoo


class RobotCommandReceiver(Node):
    """ROS2 node that receives commands via ROSBridge and executes them."""
    
    def __init__(self):
        super().__init__('robot_command_receiver')
        
        # Home position (should match config.py on Mac side)
        # Matches pick_and_place.py: [0.1697, 0.0, 0.2]
        self.home_position = [0.1697, 0.0, 0.2]  # m (matches config.ROBOT_HOME_POSITION)
        
        # Current position tracking
        self.current_position = self.home_position.copy()
        self.current_gripper_state = False  # False = open, True = closed
        
        # Command frequency and speed parameters
        self.command_frequency = self.declare_parameter('command_frequency', 5).value
        self.movement_time_ms = round(1000 / self.command_frequency)
        self.endpoint_speed = self.declare_parameter('endpoint_speed', 0.05).value
        
        # Publishers
        self.pub_endpoint_desired = self.create_publisher(
            ME439PointXYZ, '/endpoint_desired', 1
        )
        self.pub_gripper_command = self.create_publisher(
            Bool, '/gripper_command', 1
        )
        self.pub_response = self.create_publisher(
            String, '/robot_command_response', 1
        )
        
        # Create messages
        self.endpoint_msg = ME439PointXYZ()
        self.gripper_msg = Bool()
        self.response_msg = String()
        
        # Subscriber for commands
        self.sub_commands = self.create_subscription(
            String, '/robot_commands', self.command_callback, 10
        )
        
        # Execution state
        self.executing = False
        self.execution_lock = threading.Lock()
        
        self.get_logger().info('=== Robot Command Receiver Node Started ===')
        self.get_logger().info(f'Subscribed to: /robot_commands')
        self.get_logger().info(f'Publishing to: /endpoint_desired, /gripper_command')
        self.get_logger().info(f'Response topic: /robot_command_response')
    
    def command_callback(self, msg: String):
        """
        Callback for receiving commands from ROSBridge.
        
        Args:
            msg: String message containing JSON command
        """
        try:
            # Parse JSON command
            command_json = json.loads(msg.data)
            self.get_logger().info(f'Received command: {command_json}')
            
            # Check if already executing
            with self.execution_lock:
                if self.executing:
                    self.get_logger().warn('Command received while executing previous command. Ignoring.')
                    self._send_error_response("Already executing a command")
                    return
                self.executing = True
            
            try:
                # Execute command based on action
                action = command_json.get('action', '')
                
                if action == 'move':
                    result = self._execute_move(command_json)
                elif action == 'grasp':
                    result = self._execute_grasp(command_json)
                else:
                    result = {
                        "status": "error",
                        "message": f"Unknown action: {action}. Supported actions: move, grasp"
                    }
                
                # Send response
                self._send_response(result)
                
            finally:
                with self.execution_lock:
                    self.executing = False
                    
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON command: {e}')
            self._send_error_response(f"Invalid JSON: {e}")
        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')
            self.get_logger().error(traceback.format_exc())
            self._send_error_response(f"Execution error: {e}")
    
    def _execute_move(self, command: dict) -> dict:
        """
        Execute move action.
        
        Args:
            command: Command dictionary
            
        Returns:
            Result dictionary
        """
        self.get_logger().info('Executing move...')
        
        x = command.get('x')
        y = command.get('y')
        z = command.get('z')
        
        if x is None or y is None or z is None:
            return {
                "status": "error",
                "message": "Missing coordinates (x, y, z required)",
                "position": None
            }
        
        target_position = [x, y, z]
        
        # Move to target position with smooth interpolation
        success = self._move_to_position(target_position)
        
        if success:
            self.current_position = target_position.copy()
            return {
                "status": "success",
                "message": f"Robot arm moved to position ({x}, {y}, {z})",
                "position": {"x": x, "y": y, "z": z}
            }
        else:
            return {
                "status": "error",
                "message": "Failed to move to target position",
                "position": {"x": x, "y": y, "z": z}
            }
    
    def _execute_grasp(self, command: dict) -> dict:
        """
        Execute grasp action.
        
        Args:
            command: Command dictionary
            
        Returns:
            Result dictionary
        """
        self.get_logger().info('Executing grasp...')
        
        grasp_state = command.get('grasp', False)
        
        # Publish gripper command
        self.gripper_msg.data = bool(grasp_state)
        self.pub_gripper_command.publish(self.gripper_msg)
        
        # Wait a bit for gripper to actuate
        time.sleep(0.3)
        
        self.current_gripper_state = grasp_state
        action_desc = "grasped" if grasp_state else "released"
        
        return {
            "status": "success",
            "message": f"End effector {action_desc} successfully"
        }
    
    def _move_to_position(self, target_position: list) -> bool:
        """
        Move robot arm to target position with smooth interpolation.
        
        Args:
            target_position: Target [x, y, z] coordinates
            
        Returns:
            True if successful, False otherwise
        """
        try:
            start_pos = np.array(self.current_position)
            end_pos = np.array(target_position)
            
            # Generate smooth trajectory
            t, trajectory = smoo.constant_velocity_interpolation(
                start_pos, end_pos, self.endpoint_speed, self.command_frequency
            )
            
            # Publish trajectory points
            for point in trajectory:
                self.endpoint_msg.xyz = point.tolist()
                self.pub_endpoint_desired.publish(self.endpoint_msg)
                time.sleep(1.0 / self.command_frequency)
            
            # Ensure final position is published
            self.endpoint_msg.xyz = target_position
            self.pub_endpoint_desired.publish(self.endpoint_msg)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error in _move_to_position: {e}')
            self.get_logger().error(traceback.format_exc())
            return False
    
    def _send_response(self, result: dict):
        """
        Send response message.
        
        Args:
            result: Result dictionary to send
        """
        try:
            response_json = json.dumps(result, ensure_ascii=False)
            self.response_msg.data = response_json
            self.pub_response.publish(self.response_msg)
            self.get_logger().info(f'Sent response: {result}')
        except Exception as e:
            self.get_logger().error(f'Error sending response: {e}')
    
    def _send_error_response(self, error_message: str):
        """
        Send error response message.
        
        Args:
            error_message: Error message string
        """
        self._send_response({
            "status": "error",
            "message": error_message
        })


def main(args=None):
    """Main function."""
    try:
        rclpy.init(args=args)
        node = RobotCommandReceiver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down robot_command_receiver node")
    except Exception:
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

