#!/usr/bin/env python3

"""
Socket Server for Robot Command Receiver

This server receives robot commands via TCP socket from the Mac client,
executes them through ROS2 topics, and sends responses back.

Commands are received via TCP socket (port 5005, JSON format).
Responses are sent back via the same socket connection (JSON format).

Supported actions:
- move: Move robot arm to specified coordinates (including home position)
- grasp: Open/close gripper

Note: move_home is handled by Mac side by converting it to a move command with home coordinates.
"""

import socket
import json
import threading
import traceback
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from xarmrob_interfaces.msg import ME439PointXYZ

import xarmrob.smooth_interpolation as smoo


class SocketRobotServer(Node):
    """ROS2 node that receives commands via TCP socket and executes them."""
    
    def __init__(self, host='0.0.0.0', port=5005):
        super().__init__('socket_robot_server')
        
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
        
        # Create messages
        self.endpoint_msg = ME439PointXYZ()
        self.gripper_msg = Bool()
        self.gripper_msg.data = False  # False = open, True = closed (initialization)
        
        # Socket server configuration
        self.host = host
        self.port = port
        self.sock = None
        self.running = False
        
        # Execution state
        self.executing = False
        self.execution_lock = threading.Lock()
        
        self.get_logger().info('=== Socket Robot Server Node Started ===')
        self.get_logger().info(f'Publishing to: /endpoint_desired, /gripper_command')
        self.get_logger().info(f'Socket server will listen on {self.host}:{self.port}')
    
    def start_socket_server(self):
        """Start the TCP socket server in a separate thread."""
        def server_thread():
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.host, self.port))
            self.sock.listen(1)
            self.running = True
            
            self.get_logger().info(f'[Socket Server] Listening on {self.host}:{self.port}...')
            
            while self.running:
                try:
                    conn, addr = self.sock.accept()
                    self.get_logger().info(f'[Socket Server] Connected by {addr}')
                    
                    # Handle client in a separate thread to allow multiple connections
                    client_thread = threading.Thread(
                        target=self._handle_client,
                        args=(conn, addr),
                        daemon=True
                    )
                    client_thread.start()
                    
                except socket.error as e:
                    if self.running:
                        self.get_logger().error(f'[Socket Server] Socket error: {e}')
                    break
                except Exception as e:
                    if self.running:
                        self.get_logger().error(f'[Socket Server] Unexpected error: {e}')
                        self.get_logger().error(traceback.format_exc())
                    break
            
            if self.sock:
                self.sock.close()
                self.get_logger().info('[Socket Server] Socket server stopped')
        
        server_thread_instance = threading.Thread(target=server_thread, daemon=True)
        server_thread_instance.start()
        return server_thread_instance
    
    def _handle_client(self, conn, addr):
        """Handle a client connection."""
        try:
            # Set socket timeout for receiving data (30 seconds)
            conn.settimeout(30.0)
            with conn:
                while self.running:
                    # Receive data
                    data = conn.recv(1024)
                    if not data:
                        self.get_logger().info(f'[Socket Server] Client {addr} disconnected')
                        break
                    
                    try:
                        # Parse JSON command
                        json_str = data.decode('utf-8')
                        command_json = json.loads(json_str)
                        self.get_logger().info(f'[Socket Server] Received command from {addr}: {command_json}')
                        
                        # Execute command
                        result = self._execute_command(command_json)
                        
                        # Send response
                        response_json = json.dumps(result, ensure_ascii=False)
                        conn.sendall(response_json.encode('utf-8'))
                        self.get_logger().info(f'[Socket Server] Sent response to {addr}: {result}')
                        
                    except json.JSONDecodeError as e:
                        self.get_logger().error(f'[Socket Server] Invalid JSON from {addr}: {e}')
                        error_response = {
                            "status": "error",
                            "message": f"Invalid JSON format: {e}"
                        }
                        conn.sendall(json.dumps(error_response).encode('utf-8'))
                    except Exception as e:
                        self.get_logger().error(f'[Socket Server] Error processing command from {addr}: {e}')
                        self.get_logger().error(traceback.format_exc())
                        error_response = {
                            "status": "error",
                            "message": f"Execution error: {e}"
                        }
                        conn.sendall(json.dumps(error_response).encode('utf-8'))
                        
        except Exception as e:
            self.get_logger().error(f'[Socket Server] Error handling client {addr}: {e}')
            self.get_logger().error(traceback.format_exc())
    
    def _execute_command(self, command: dict) -> dict:
        """
        Execute a robot command.
        
        Args:
            command: Command dictionary
            
        Returns:
            Result dictionary
        """
        # Check if already executing
        with self.execution_lock:
            if self.executing:
                self.get_logger().warn('Command received while executing previous command. Ignoring.')
                return {
                    "status": "error",
                    "message": "Already executing a command"
                }
            self.executing = True
        
        try:
            # Execute command based on action
            action = command.get('action', '')
            
            if action == 'move':
                result = self._execute_move(command)
            elif action == 'grasp':
                result = self._execute_grasp(command)
            else:
                result = {
                    "status": "error",
                    "message": f"Unknown action: {action}. Supported actions: move, grasp"
                }
            
            return result
            
        finally:
            with self.execution_lock:
                self.executing = False
    
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
        
        try:
            grasp_state = command.get('grasp', False)
            self.get_logger().info(f'Received grasp command: grasp_state={grasp_state} (type: {type(grasp_state)})')
            
            # Ensure correct boolean conversion
            # True = closed/grasp, False = open/release
            self.gripper_msg.data = bool(grasp_state)
            self.get_logger().info(f'Publishing gripper command: data={self.gripper_msg.data} (True=closed, False=open)')
            
            # Publish gripper command
            self.pub_gripper_command.publish(self.gripper_msg)
            self.get_logger().info('Gripper command published successfully')
            
            # Wait a bit for gripper to actuate
            time.sleep(0.5)  # Increased wait time to ensure message is processed
            
            self.current_gripper_state = grasp_state
            action_desc = "grasped" if grasp_state else "released"
            
            self.get_logger().info(f'Grasp execution completed: {action_desc}')
            
            return {
                "status": "success",
                "message": f"End effector {action_desc} successfully"
            }
            
        except Exception as e:
            self.get_logger().error(f'Error in _execute_grasp: {e}')
            self.get_logger().error(traceback.format_exc())
            return {
                "status": "error",
                "message": f"Failed to execute grasp: {e}"
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
    
    def stop(self):
        """Stop the socket server."""
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass


def main(args=None):
    """Main function."""
    import sys
    
    # Parse command line arguments
    host = '0.0.0.0'
    port = 5005
    
    if len(sys.argv) > 1:
        host = sys.argv[1]
    if len(sys.argv) > 2:
        port = int(sys.argv[2])
    
    try:
        rclpy.init(args=args)
        node = SocketRobotServer(host=host, port=port)
        
        # Start socket server in background thread
        server_thread = node.start_socket_server()
        
        # Spin ROS2 node (this blocks)
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nShutting down socket robot server")
        if 'node' in locals():
            node.stop()
    except Exception:
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

