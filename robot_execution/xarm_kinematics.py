#!/usr/bin/env python3
 
# ROS node to receive an Endpoint for a HiWonder xArm 1S 
# and convert it to Joint Angles, which are published 
# Peter Adamczyk, University of Wisconsin - Madison
# Updated 2025-04-03

import rclpy
from rclpy.node import Node 
import numpy as np
import traceback
from xarmrob_interfaces.srv import ME439XArmForwardKinematics, ME439XArmInverseKinematics
from xarmrob_interfaces.msg import ME439PointXYZ
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

## Define a temporary function using Python "lambda" functionality to print colored text
# see https://stackoverflow.com/questions/287871/how-do-i-print-colored-text-to-the-terminal/3332860#3332860
# search that page for "CircuitSacul" to find that answer
coloredtext = lambda r, g, b, text: f'\033[38;2;{r};{g};{b}m{text}\033[38;2;255;255;255m'




class XArmKinematics(Node): 
    def __init__(self):
        super().__init__('xarm_kinematics')
        
        # Create Services 
        self.srv_FK = self.create_service(ME439XArmForwardKinematics, 'xarm_forward_kinematics', self.compute_FK)
        self.srv_IK = self.create_service(ME439XArmInverseKinematics, 'xarm_inverse_kinematics', self.compute_IK)
        self.sub_IK = self.create_subscription(ME439PointXYZ, '/endpoint_desired', self.compute_IK_pub_JTANG,1)
        self.pub_JTANG = self.create_publisher(JointState, '/joint_angles_desired',1)
        self.ang_all = [0., -np.pi/2., np.pi/2., 0., 0., 0., 0.]
        self.joint_angles_desired_msg = JointState()
        self.joint_angles_desired_msg.name = ['base_joint', 'shoulder_joint', 'elbow_joint', 'forearm_joint', 'wrist_joint', 'fingers_joint', 'gripper'];
        self.joint_angles_desired_msg.position = self.ang_all 
        
        # Subscribe to gripper command topic (NEW!)
        self.gripper_state = 0  # 0 = open, 1 = closed
        self.sub_gripper = self.create_subscription(Bool, '/gripper_command', self.gripper_callback, 1)

        # Load parameters for the functions below: 
        # Matched lists of angles and microsecond commands
        self.map_ang_rad_01 = np.radians(np.array(self.declare_parameter('rotational_angles_for_mapping_joint_01',[-90,0,90]).value))
        # self.map_cmd_01 = np.array(self.declare_parameter('bus_servo_cmd_for_mapping_joint_01',[120, 500, 880]).value)
        self.map_ang_rad_12 = np.radians(np.array(self.declare_parameter('rotational_angles_for_mapping_joint_12',[-180,-90,0]).value))
        # self.map_cmd_12 = np.array(self.declare_parameter('bus_servo_cmd_for_mapping_joint_12',[870,500,120]).value)
        self.map_ang_rad_23 = np.radians(np.array(self.declare_parameter('rotational_angles_for_mapping_joint_23',[0,90,180]).value))
        # self.map_cmd_23 = np.array(self.declare_parameter('bus_servo_cmd_for_mapping_joint_23',[140,500,880]).value)
        self.map_ang_rad_34 = np.radians(np.array(self.declare_parameter('rotational_angles_for_mapping_joint_34',[-112,-90,0,90,112]).value))
        # self.map_cmd_34 = np.array(self.declare_parameter('bus_servo_cmd_for_mapping_joint_34', [1000,890,505,140,0]).value)
        self.map_ang_rad_45 = np.radians(np.array(self.declare_parameter('rotational_angles_for_mapping_joint_45',[-112,-90,0,90,112]).value))
        # self.map_cmd_45 = np.array(self.declare_parameter('bus_servo_cmd_for_mapping_joint_45',[0,120,490,880,1000]).value)
        self.map_ang_rad_56 = np.radians(np.array(self.declare_parameter('rotational_angles_for_mapping_joint_56',[-112,-90,0,90,112]).value))
        # self.map_cmd_56 = np.array(self.declare_parameter('bus_servo_cmd_for_mapping_joint_56',[0,120,500,880,1000]).value)
        self.map_ang_rad_gripper = np.radians(np.array(self.declare_parameter('rotational_angles_for_mapping_gripper',[0, 90]).value))
        # self.map_cmd_gripper = np.array(self.declare_parameter('bus_servo_cmd_for_mapping_gripper',[90,610]).value)
        
        # limits for each of the joints
        self.rotlim_01 = np.radians(np.array(self.declare_parameter('rotational_limits_joint_01',[-150,150]).value))
        self.rotlim_12 = np.radians(np.array(self.declare_parameter('rotational_limits_joint_12',[-180,0]).value))
        self.rotlim_23 = np.radians(np.array(self.declare_parameter('rotational_limits_joint_23',[0,180]).value))
        self.rotlim_34 = np.radians(np.array(self.declare_parameter('rotational_limits_joint_34',[-110,110]).value))
        self.rotlim_45 = np.radians(np.array(self.declare_parameter('rotational_limits_joint_45',[-100,100]).value))
        self.rotlim_56 = np.radians(np.array(self.declare_parameter('rotational_limits_joint_56',[-110,111]).value))
        self.rotlim_gripper = np.radians(np.array(self.declare_parameter('gripper_limits',[0,90]).value))
        
        # Sign of 'positive' rotations w.r.t. the y axis
        self.y_rotation_sign = np.sign(self.declare_parameter('y_rotation_sign',1).value)
        # Vectors from each frame origin to the next frame origin, in the proximal
        self.r_01 = np.column_stack(self.declare_parameter('frame_offset_01',[0., 0., 0.074]).value).transpose()
        self.r_12 = np.column_stack(self.declare_parameter('frame_offset_12',[0.010, 0., 0.]).value).transpose()
        self.r_23 = np.column_stack(self.declare_parameter('frame_offset_23',[0.101, 0., 0.]).value).transpose()
        self.r_34 = np.column_stack(self.declare_parameter('frame_offset_34',[0.0627, 0., 0.0758]).value).transpose()        
        self.r_45 = np.column_stack(self.declare_parameter('frame_offset_45',[0., 0., 0.]).value).transpose()
        self.r_56 = np.column_stack(self.declare_parameter('frame_offset_56',[0., 0., 0.]).value).transpose()
        self.r_6end = np.column_stack(self.declare_parameter('endpoint_offset_in_frame_6',[0.133, 0., -0.003]).value).transpose()
        


            
            
        
    # Service for Forward Kinematics    
    def compute_FK(self, request_FK, response_FK):
        ang = request_FK.joint_angles
        
        # Limit angles to the allowed ranges based on limit Parameters
        ang_lim = self.limit_joint_angles(ang)
        
        pos_endpoint, pos_all = self.fwdkin(ang_lim)        
        
        # Pack the response
        response_FK.joint_angles = ang_lim
        response_FK.endpoint = pos_endpoint
        if np.allclose(ang_lim,ang):
            response_FK.modified = False
        else: 
            response_FK.modified = True
            
        return response_FK
        
    # Function to compute endpoint location from joint angles. 
    def fwdkin(self, joint_angles):
        # Unpack joint angles.
        alpha0 = joint_angles[0]
        beta1 = joint_angles[1] * self.y_rotation_sign
        beta2 = joint_angles[2] * self.y_rotation_sign
        gamma3 = joint_angles[3]
        beta4 = joint_angles[4] * self.y_rotation_sign
        gamma5 = joint_angles[5]    
        
        # =============================================================================
        # # Transformation from frame 0 to 1 (+Z axis rotation)
        # =============================================================================
        # "Rotation matrix of frame 1 in frame 0's coordinates" (columns are unit vectors of Frame 1 in Frame 0 coordinates)
        R_01 = np.array([ [np.cos(alpha0), -np.sin(alpha0), 0.], 
                             [np.sin(alpha0), np.cos(alpha0), 0.],
                             [       0.,           0.,  1.] ])
        # "Homogeneous Transform of Frame 1 in Frame 0's Coordinates"
        T_01 = np.vstack( (np.column_stack( (R_01, self.r_01) ) , [0., 0., 0., 1.]) )
        
        # =============================================================================
        # # Transformation from frame 1 to 2 (+Y axis rotation)
        # =============================================================================
        R_12 = np.array([ [ np.cos(beta1), 0., np.sin(beta1)], 
                           [       0. ,     1.,        0.    ],
                           [-np.sin(beta1), 0., np.cos(beta1)] ])
        T_12 = np.vstack( (np.column_stack( (R_12, self.r_12) ) , [0., 0., 0., 1.]) )
            
        # =============================================================================
        # # Transformation from frame 2 to 3 (+Y axis rotation)
        # =============================================================================
        R_23 = np.array([ [ np.cos(beta2), 0., np.sin(beta2)], 
                           [       0. ,     1.,        0.    ],
                           [-np.sin(beta2), 0., np.cos(beta2)] ])
        T_23 = np.vstack( (np.column_stack( (R_23, self.r_23) ) , [0., 0., 0., 1.]) )
            
        # =============================================================================
        # # Transformation from frame 3 to 4 (+X axis rotation)
        # =============================================================================
        R_34 = np.array([ [ 1. ,        0.     ,        0.      ], 
                           [ 0. , np.cos(gamma3), -np.sin(gamma3)], 
                           [ 0. , np.sin(gamma3),  np.cos(gamma3)] ])
        T_34 = np.vstack( (np.column_stack( (R_34, self.r_34) ) , [0., 0., 0., 1.]) )
                
        # =============================================================================
        # # Transformation from frame 4 to 5 (+Y axis rotation)
        # =============================================================================
        R_45 = np.array([ [ np.cos(beta4), 0., np.sin(beta4)], 
                           [       0. ,     1.,        0.    ],
                           [-np.sin(beta4), 0., np.cos(beta4)] ])
        T_45 = np.vstack( (np.column_stack( (R_45, self.r_45) ) , [0., 0., 0., 1.]) )
                
        # =============================================================================
        # # Transformation from frame 5 to 6 (+X axis rotation)
        # =============================================================================
        R_56 = np.array([ [ 1. ,        0.     ,        0.      ], 
                           [ 0. , np.cos(gamma5), -np.sin(gamma5)], 
                           [ 0. , np.sin(gamma5),  np.cos(gamma5)] ])
        T_56 = np.vstack( (np.column_stack( (R_56, self.r_56) ) , [0., 0., 0., 1.]) )
        
        # return T_01, T_12, T_23, T_34, T_45, T_56         

        # Vector of Zero from the frame origin in question, augmented with a 1 so it can be used with the Homogeneous Transform
        zerovec = np.column_stack(np.array([0.,0.,0.,1.])).transpose()     
        
        pos_0 = zerovec[0:3,0] # base link location: 0
        pos_1 = (T_01@zerovec)[0:3,0]
        T_02 = T_01@T_12
        pos_2 = (T_02@zerovec)[0:3,0]
        T_03 = T_02@T_23
        pos_3 = (T_03@zerovec)[0:3,0]
        T_04 = T_03@T_34
        pos_4 = (T_04@zerovec)[0:3,0]
        T_05 = T_04@T_45
        pos_5 = (T_05@zerovec)[0:3,0]
        T_06 = T_05@T_56
        pos_6 = (T_06@zerovec)[0:3,0]
        
        pos_endpoint = (T_06@np.vstack((self.r_6end,1)) )[0:3,0]
        pos_all = np.column_stack( (pos_0, pos_1, pos_2, pos_3, pos_4, pos_5, pos_6, pos_endpoint) )     
        
        return pos_endpoint, pos_all
        
    

    # =============================================================================
    # # Function to compute joint angles to reach the target endpoint
    # =============================================================================
    def gripper_callback(self, msg):
        """Callback to update gripper state from Bool message"""
        # Convert Bool to gripper angle (0 or pi/2 radians = 0 or 90 degrees)
        self.gripper_state = np.pi/2 if msg.data else 0.0
        
    def compute_IK_pub_JTANG(self, msg_endpoint):  
        endpoint_goal = np.array(msg_endpoint.xyz)
        
        # effector_pose = request_IK.effector_pose
        # if not all(np.array(effector_pose)==0.):
        #     self.get_logger().info(coloredtext(255,0,0,'Warning: Custom End Effector Pose is not yet implemented!!')) 
        
        # First compute the inverse kinematics for perfect endpoint positioning
        ang = self.invkin(endpoint_goal)
        
        # Then Limit the angle at each joint to its achievable range
        ang_lim = self.limit_joint_angles(ang)
        
        # Compute the endpoint achieved by the limited angles
        pos_endpoint, pos_all = self.fwdkin(ang_lim)   
        
        # Pack the response
        joint_angles = ang_lim
        endpoint = pos_endpoint.flatten()

        # Report if the solution is Mismatched from the original. 
        if np.allclose(pos_endpoint.flatten(),endpoint_goal.flatten()):
            modified = False
        else: 
            modified = True
            self.get_logger().info(coloredtext(50,255,50,'\n\tMoving to [' + '{:.3f}, '*2 + '{:.3f}]').format(*endpoint))
            
        # Use the actual gripper state instead of always 0!
        self.joint_angles_desired_msg.position = np.append(ang_lim, self.gripper_state)
        self.joint_angles_desired_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_JTANG.publish(self.joint_angles_desired_msg)
    
    # =============================================================================
    # # Function to compute joint angles to reach the target endpoint
    # =============================================================================
    def compute_IK(self, request_IK, response_IK):  
        endpoint = np.array(request_IK.endpoint)
        
        effector_pose = request_IK.effector_pose
        if not all(np.array(effector_pose)==0.):
            self.get_logger().info(coloredtext(255,0,0,'Warning: Custom End Effector Pose is not yet implemented!!')) 
        
        # First compute the inverse kinematics for perfect endpoint positioning
        ang = self.invkin(endpoint)
        
        # Then Limit the angle at each joint to its achievable range
        ang_lim = self.limit_joint_angles(ang)
        
        # Compute the endpoint achieved by the limited angles
        pos_endpoint, pos_all = self.fwdkin(ang_lim)   
        
        # Pack the response
        response_IK.joint_angles = ang_lim
        response_IK.endpoint = pos_endpoint.flatten()

        # Report if the solution is Mismatched from the original. 
        if np.allclose(pos_endpoint.flatten(),endpoint.flatten()):
            response_IK.modified = False
        else: 
            response_IK.modified = True

        return response_IK    
    
        
    def forward_kinematics_dh(self, q, link_lengths):
        """Forward kinematics using DH-like transform - Python version of MATLAB FWD_KIN_DH"""
        l1, l2, l3, l4, l5, l6 = link_lengths
        q1, q2, q3, q4, q5, q6 = q
        
        # Define each transformation matrix
        T10 = np.array([
            [0, np.cos(q1), -np.sin(q1), 0],
            [0, np.sin(q1),  np.cos(q1), 0],
            [1, 0,           0,          l1],
            [0, 0,           0,          1]
        ])
        
        T21 = np.array([
            [np.cos(q2), -np.sin(q2), 0, 0],
            [np.sin(q2),  np.cos(q2), 0, l2],
            [0,           0,          1, 0],
            [0,           0,          0, 1]
        ])
        
        T32 = np.array([
            [0, np.cos(q3), -np.sin(q3), 0],
            [0, np.sin(q3),  np.cos(q3), l3],
            [1, 0,           0,          0],
            [0, 0,           0,          1]
        ])
        
        T43 = np.array([
            [-np.sin(q4), 0, np.cos(q4), 0],
            [ np.cos(q4), 0, np.sin(q4), l4],
            [0,           1, 0,          0],
            [0,           0, 0,          1]
        ])
        
        T54 = np.array([
            [0, np.cos(q5), -np.sin(q5), 0],
            [0, np.sin(q5),  np.cos(q5), l5],
            [1, 0,           0,          0],
            [0, 0,           0,          1]
        ])
        
        T65 = np.array([
            [0, np.cos(q6), -np.sin(q6), 0],
            [0, np.sin(q6),  np.cos(q6), 0],
            [1, 0,           0,          l6],
            [0, 0,           0,          1]
        ])
        
        # Compute the transforms to each frame
        T20 = T10 @ T21
        T30 = T20 @ T32
        T40 = T30 @ T43
        T50 = T40 @ T54
        T60 = T50 @ T65
        
        return T10, T20, T30, T40, T50, T60
    
    def compute_jacobian_6dof(self, T10, T20, T30, T40, T50, T60):
        """Compute Jacobian - Python version of MATLAB Jacobian_6DOF"""
        z_00 = np.array([0, 0, 1])
        z_10 = T10[0:3, 2]
        z_20 = T20[0:3, 2]
        z_30 = T30[0:3, 2]
        z_40 = T40[0:3, 2]
        z_50 = T50[0:3, 2]
        
        o_60 = T60[0:3, 3]
        o_50 = T50[0:3, 3]
        o_40 = T40[0:3, 3]
        o_30 = T30[0:3, 3]
        o_20 = T20[0:3, 3]
        o_10 = T10[0:3, 3]
        
        # Position part of Jacobian
        J_pos = np.column_stack([
            np.cross(z_00, o_60),
            np.cross(z_10, o_60 - o_10),
            np.cross(z_20, o_60 - o_20),
            np.cross(z_30, o_60 - o_30),
            np.cross(z_40, o_60 - o_40),
            np.cross(z_50, o_60 - o_50)
        ])
        
        # Orientation part of Jacobian
        J_rot = np.column_stack([z_00, z_10, z_20, z_30, z_40, z_50])
        
        # Full 6x6 Jacobian
        J = np.vstack([J_pos, J_rot])
        return J
    
    def ik_6dof_solver(self, p_goal, q_init, link_lengths):
        """
        Inverse Kinematics Solver - Python version of MATLAB IK_6DOF
        Position only version with damped least squares
        """
        # Parameters from MATLAB
        tolerance = 1e-3
        max_iterations = 1000
        status = False
        
        # Adaptive step size parameters
        alpha = 1.0
        alpha_min = 0.01
        alpha_max = 1.0
        beta = 0.5
        gamma = 1.1
        
        # Damping for numerical stability
        lambda_damp = 0.01
        
        # Joint limits from MATLAB
        q_min = np.array([-5*np.pi/6, -np.pi, 0, -11*np.pi/18, -5*np.pi/9, -11*np.pi/18])
        q_max = np.array([5*np.pi/6, 0, np.pi, 11*np.pi/18, 5*np.pi/9, 11*np.pi/18])
        
        # Initialize
        q = np.array(q_init, dtype=float)
        prev_error = np.inf
        
        for iteration in range(max_iterations):
            # Forward kinematics
            T10, T20, T30, T40, T50, T60 = self.forward_kinematics_dh(q, link_lengths)
            
            # Current end-effector position
            p_curr = T60[0:3, 3]
            
            # Position error
            pos_error = p_goal - p_curr
            error_norm = np.linalg.norm(pos_error)
            
            # Check convergence
            if error_norm < tolerance:
                status = True
                break
            
            # Compute Jacobian (position part only, 3x6)
            J_full = self.compute_jacobian_6dof(T10, T20, T30, T40, T50, T60)
            J = J_full[0:3, :]  # Take only position part
            
            # Damped Least Squares (DLS) Pseudo-inverse
            JtJ = J.T @ J
            J_dls = np.linalg.solve(JtJ + lambda_damp * np.eye(6), J.T)
            
            # Compute joint update
            delta_q = J_dls @ pos_error
            
            # Limit joint velocity
            max_delta = 0.2  # radians
            delta_q_norm = np.linalg.norm(delta_q)
            if delta_q_norm > max_delta:
                delta_q = delta_q * (max_delta / delta_q_norm)
            
            # Apply adaptive step size
            q_new = q + alpha * delta_q
            
            # Wrap angles to [-pi, pi]
            for i in range(6):
                q_new[i] = np.arctan2(np.sin(q_new[i]), np.cos(q_new[i]))
            
            # Enforce joint limits
            margin = 0.01
            for i in range(6):
                if q_new[i] < q_min[i] + margin:
                    q_new[i] = q_min[i] + margin
                elif q_new[i] > q_max[i] - margin:
                    q_new[i] = q_max[i] - margin
            
            # Check if new configuration reduces error
            T10_new, T20_new, T30_new, T40_new, T50_new, T60_new = self.forward_kinematics_dh(q_new, link_lengths)
            p_new = T60_new[0:3, 3]
            pos_error_new = np.linalg.norm(p_goal - p_new)
            
            # Adaptive step size adjustment
            if pos_error_new < np.linalg.norm(pos_error):
                q = q_new.copy()
                if error_norm < prev_error:
                    alpha = min(alpha * gamma, alpha_max)
                prev_error = error_norm
            else:
                alpha = max(alpha * beta, alpha_min)
                if alpha <= alpha_min:
                    q = q_new.copy()  # Accept anyway if stuck
            
            # Early stop if stuck
            if iteration > 100 and error_norm > 10 * tolerance and alpha <= alpha_min:
                break
        
        if iteration == max_iterations - 1 and not status:
            self.get_logger().warning(f'IK Max iterations reached. Final error: {error_norm:.6f}')
        
        return q, status
    
    def invkin(self, endpoint):
        """New inverse kinematics using the tested MATLAB algorithm"""
        xyz = np.array(endpoint)
        
        # Input validation
        if np.any(np.isnan(xyz)) or np.any(np.isinf(xyz)):
            self.get_logger().warning('Invalid endpoint coordinates (NaN/Inf) received. Using safe default position.')
            return np.array([0., -np.pi/2., np.pi/2., 0., 0., 0.])
        
        try:
            # Link lengths from MATLAB test file
            link_lengths = [0.074, 0.010, 0.101, 0.0758, 0.0627, 0.133]
            
            # Initial guess - use current angles if available, otherwise neutral pose
            if hasattr(self, 'ang_all') and len(self.ang_all) >= 6:
                q_init = np.array(self.ang_all[:6])
            else:
                # Neutral pose from MATLAB
                q_init = np.array([0., -np.pi/2., np.pi/2., 0., 0., 0.])
            
            # Call the MATLAB-equivalent IK solver
            q_solution, status = self.ik_6dof_solver(xyz, q_init, link_lengths)
            
            if not status:
                self.get_logger().warning(f'IK did not converge for target [{xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}]')
                # Return a safe pose but still attempt the motion
            
            # Store current angles for next iteration
            self.ang_all = np.append(q_solution, 0.0)  # Add gripper angle
            
            return q_solution
            
        except Exception as e:
            self.get_logger().error(f'IK solver failed: {e}')
            return np.array([0., -np.pi/2., np.pi/2., 0., 0., 0.])
    
        
    def limit_joint_angles(self, angles):
        angles_limited = angles
        
        # Clip (saturate) the angles at the achievable limits. 
        angles_limited[0] = np.clip(angles_limited[0], np.min(self.rotlim_01), np.max(self.rotlim_01))
        angles_limited[1] = np.clip(angles_limited[1], np.min(self.rotlim_12), np.max(self.rotlim_12))
        angles_limited[2] = np.clip(angles_limited[2], np.min(self.rotlim_23), np.max(self.rotlim_23))
        angles_limited[3] = np.clip(angles_limited[3], np.min(self.rotlim_34), np.max(self.rotlim_34))
        angles_limited[4] = np.clip(angles_limited[4], np.min(self.rotlim_45), np.max(self.rotlim_45))
        angles_limited[5] = np.clip(angles_limited[5], np.min(self.rotlim_56), np.max(self.rotlim_56))
    
        return angles_limited





def main(args=None):
    try: 
        rclpy.init(args=args)
        xarm_kinematics_instance = XArmKinematics()  
        rclpy.spin(xarm_kinematics_instance) 
        
    except: 
        traceback.print_exc()
        


if __name__ == '__main__':
    main()