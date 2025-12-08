#!/usr/bin/env python3

# ROS node to perform automated pick-and-place task with xArm robot
# Picks up green and red objects and places them in corresponding bins
# Based on endpoint_automatic_smooth.py by Peter Adamczyk
# Modified for pick-and-place task with gripper control - 2024-12

import rclpy
from rclpy.node import Node 
import numpy as np
import traceback
import time
from xarmrob_interfaces.msg import ME439PointXYZ
from std_msgs.msg import Bool

import xarmrob.smooth_interpolation as smoo 

## Define a temporary function using Python "lambda" functionality to print colored text
coloredtext = lambda r, g, b, text: f'\033[38;2;{r};{g};{b}m{text}\033[38;2;255;255;255m'

class PickAndPlace(Node): 
    def __init__(self): 
        super().__init__('pick_and_place')
        
        # Define key positions
        self.home_position = [0.1697, 0.0, 0.2]
        self.green_object = [0.2271, -0.1448, 0.0360]
        self.red_object = [0.2271, 0.1712, 0.0372]
        self.red_bin = [0.1503, 0.0, 0.0708]
        self.green_bin = [0.2021, 0.0, 0.0708]
        
        # Approach height offset (approach from above for safety)
        self.approach_height = 0.10  # 10cm above object/bin
        
        self.xyz_goal = self.home_position.copy()
        self.gripper = 0  # 0 = open, 1 = closed
        self.idx = 0
        self.trajectory_finished = False
        
        # =============================================================================
        #   # Publisher for the Endpoint goal. 
        # =============================================================================
        self.pub_endpoint_desired = self.create_publisher(ME439PointXYZ, '/endpoint_desired', 1)
        
        # =============================================================================
        #   # Publisher for the Gripper command (NEW!)
        # =============================================================================
        self.pub_gripper_command = self.create_publisher(Bool, '/gripper_command', 1)
        
        # Create the messages
        self.endpoint_desired_msg = ME439PointXYZ()
        self.endpoint_desired_msg.xyz = self.xyz_goal 
        
        self.gripper_command_msg = Bool()
        self.gripper_command_msg.data = False  # False = open, True = closed

        # Command frequency parameter: how often we expect it to be updated    
        self.command_frequency = self.declare_parameter('command_frequency', 5).value
        self.movement_time_ms = round(1000/self.command_frequency)  # milliseconds for movement time. 
        self.endpoint_speed = self.declare_parameter('endpoint_speed', 0.05).value  # nominal speed for continuous movement among points. 
        
        # Build the complete pick-and-place trajectory
        self.set_pick_and_place_trajectory()
        
        self.get_logger().info(coloredtext(100, 255, 100, '=== Pick and Place Task Starting ==='))
        self.get_logger().info('Waiting 3 seconds before starting trajectory...')
        time.sleep(3)
        
        # Set up a timer to send the commands at the specified rate. 
        self.timer = self.create_timer(self.movement_time_ms/1000, self.send_endpoint_desired)

    def create_approach_position(self, target_position):
        """Create an approach position above the target"""
        approach = target_position.copy()
        approach[2] = target_position[2] + self.approach_height
        return approach

    def send_endpoint_desired(self):
        """Callback to publish the endpoint at the specified rate"""
        if self.idx >= len(self.disp_traj):
            if not self.trajectory_finished:
                self.get_logger().info(coloredtext(100, 255, 100, '=== Pick and Place Task Complete! ==='))
                self.trajectory_finished = True
            return  # Stop publishing after completion
        else:
            self.xyz_goal = self.disp_traj[self.idx]
            
            # Check if we're at a gripper action point
            if self.idx in self.gripper_actions:
                action = self.gripper_actions[self.idx]
                self.gripper = action['state']
                
                # Publish gripper command
                self.gripper_command_msg.data = bool(self.gripper)
                self.pub_gripper_command.publish(self.gripper_command_msg)
                
                status = "CLOSED" if self.gripper == 1 else "OPEN"
                self.get_logger().info(coloredtext(255, 200, 100, f'>>> Gripper {status} <<<'))
            
            # Log position every 10 steps to avoid spam
            if self.idx % 10 == 0:
                self.get_logger().info(f'Step {self.idx}/{len(self.disp_traj)}: [{self.xyz_goal[0]:.4f}, {self.xyz_goal[1]:.4f}, {self.xyz_goal[2]:.4f}]')
            
            self.idx += 1
            
        self.endpoint_desired_msg.xyz = self.xyz_goal 
        self.pub_endpoint_desired.publish(self.endpoint_desired_msg)
        
    def set_pick_and_place_trajectory(self):
        """Create the complete pick-and-place trajectory with smooth interpolation"""
        
        # Create waypoint sequence with descriptions
        waypoint_sequence = [
            # Start at home
            {'position': self.home_position, 'gripper': 0, 'description': 'HOME (Start)', 'color': (150, 150, 255)},
            
            # === PICK GREEN OBJECT ===
            {'position': self.create_approach_position(self.green_object), 'gripper': 0, 'description': 'Approach GREEN object', 'color': (100, 255, 100)},
            {'position': self.green_object, 'gripper': 0, 'description': 'At GREEN object', 'color': (100, 255, 100)},
            {'position': self.green_object, 'gripper': 1, 'description': 'CLOSE gripper (pick GREEN)', 'color': (255, 200, 100)},
            {'position': self.create_approach_position(self.green_object), 'gripper': 1, 'description': 'Lift GREEN object', 'color': (100, 255, 100)},
            
            # === PLACE IN GREEN BIN ===
            {'position': self.create_approach_position(self.green_bin), 'gripper': 1, 'description': 'Approach GREEN bin', 'color': (100, 255, 100)},
            {'position': self.green_bin, 'gripper': 1, 'description': 'At GREEN bin', 'color': (100, 255, 100)},
            {'position': self.green_bin, 'gripper': 0, 'description': 'OPEN gripper (release GREEN)', 'color': (255, 200, 100)},
            {'position': self.create_approach_position(self.green_bin), 'gripper': 0, 'description': 'Retreat from GREEN bin', 'color': (100, 255, 100)},
            
            # === RETURN TO HOME ===
            {'position': self.home_position, 'gripper': 0, 'description': 'HOME (after GREEN)', 'color': (150, 150, 255)},
            
            # === PICK RED OBJECT ===
            {'position': self.create_approach_position(self.red_object), 'gripper': 0, 'description': 'Approach RED object', 'color': (255, 100, 100)},
            {'position': self.red_object, 'gripper': 0, 'description': 'At RED object', 'color': (255, 100, 100)},
            {'position': self.red_object, 'gripper': 1, 'description': 'CLOSE gripper (pick RED)', 'color': (255, 200, 100)},
            {'position': self.create_approach_position(self.red_object), 'gripper': 1, 'description': 'Lift RED object', 'color': (255, 100, 100)},
            
            # === PLACE IN RED BIN ===
            {'position': self.create_approach_position(self.red_bin), 'gripper': 1, 'description': 'Approach RED bin', 'color': (255, 100, 100)},
            {'position': self.red_bin, 'gripper': 1, 'description': 'At RED bin', 'color': (255, 100, 100)},
            {'position': self.red_bin, 'gripper': 0, 'description': 'OPEN gripper (release RED)', 'color': (255, 200, 100)},
            {'position': self.create_approach_position(self.red_bin), 'gripper': 0, 'description': 'Retreat from RED bin', 'color': (255, 100, 100)},
            
            # === FINAL RETURN TO HOME ===
            {'position': self.home_position, 'gripper': 0, 'description': 'HOME (Final)', 'color': (150, 150, 255)},
        ]
        
        # Log the planned trajectory
        self.get_logger().info(coloredtext(255, 255, 100, '\n=== Planned Trajectory ==='))
        for i, waypoint in enumerate(waypoint_sequence):
            pos = waypoint['position']
            grip = 'CLOSED' if waypoint['gripper'] == 1 else 'OPEN'
            color = waypoint['color']
            self.get_logger().info(coloredtext(*color, 
                f"{i+1:2d}. {waypoint['description']:35s} -> [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}] Gripper: {grip}"))
        self.get_logger().info(coloredtext(255, 255, 100, '='*70 + '\n'))
        
        # Build smooth interpolated trajectory between waypoints
        self.disp_traj = np.ndarray((0, 3))
        self.gripper_actions = {}  # Dictionary to store gripper actions at specific indices
        
        for ii in range(1, len(waypoint_sequence)):
            start_pos = np.array(waypoint_sequence[ii-1]['position'])
            end_pos = np.array(waypoint_sequence[ii]['position'])
            
            # Check if this is a gripper-only action (position doesn't change)
            if np.allclose(start_pos, end_pos):
                # Add a single point for gripper action
                self.disp_traj = np.vstack((self.disp_traj, [end_pos]))
                # Record gripper action at this index
                self.gripper_actions[len(self.disp_traj) - 1] = {
                    'state': waypoint_sequence[ii]['gripper'],
                    'description': waypoint_sequence[ii]['description']
                }
            else:
                # Smooth interpolation between different positions
                try:
                    t, seg_traj = smoo.constant_velocity_interpolation(
                        start_pos, end_pos, self.endpoint_speed, self.command_frequency
                    )
                    self.disp_traj = np.vstack((self.disp_traj, seg_traj))
                except Exception as e:
                    self.get_logger().error(f'Error in interpolation: {e}')
                    self.get_logger().error(f'From {start_pos} to {end_pos}')
                    break
        
        self.idx = 0
        self.get_logger().info(f"Generated trajectory with {len(self.disp_traj)} total points")
        self.get_logger().info(f"Estimated duration: {len(self.disp_traj) / self.command_frequency:.1f} seconds")


def main(args=None):
    try: 
        rclpy.init(args=args)
        pick_and_place_instance = PickAndPlace()  
        rclpy.spin(pick_and_place_instance)
        
    except KeyboardInterrupt:
        print("\nShutting down pick_and_place node")
    except: 
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()