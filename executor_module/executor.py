"""
Executor Module

This module executes execution plans by converting tasks into function calls
and managing data flow between actions.
"""

from typing import Dict, Any, Optional, List
from llm_module.plan_parser import ExecutionPlan, Task
from .robot_functions import move, grasp, see, move_home
import config


class Executor:
    """Executes execution plans by calling robot functions."""
    
    def __init__(self):
        """Initialize the executor."""
        self.execution_context: Dict[str, Any] = {}
        self.execution_history: List[Dict[str, Any]] = []
        # Initialize with home position so relative moves work from the start
        self.execution_context["last_position"] = config.ROBOT_HOME_POSITION.copy()
    
    def execute_plan(self, plan: ExecutionPlan) -> Dict[str, Any]:
        """
        Execute a complete execution plan.
        
        Args:
            plan: ExecutionPlan object to execute
            
        Returns:
            Dictionary with execution results
        """
        if not plan:
            return {
                "status": "error",
                "message": "No execution plan provided"
            }
        
        if not plan.tasks or len(plan.tasks) == 0:
            print("\n[Executor] ⚠ Empty execution plan (no tasks to execute)")
            return {
                "status": "success",
                "message": "Empty execution plan - no tasks to execute",
                "execution_history": []
            }
        
        print("\n" + "="*60)
        print("[Executor] ===== PLAN EXECUTION START =====")
        print("="*60)
        print(f"[Executor] Total tasks to execute: {len(plan.tasks)}")
        
        # Preserve execution context between command executions (for relative movement)
        # Only reset execution history for this plan execution
        if self.execution_context:
            print(f"[Executor] Preserving context from previous execution: {list(self.execution_context.keys())}")
            if "last_position" in self.execution_context:
                print(f"[Executor] Last known position: {self.execution_context['last_position']}")
        else:
            print(f"[Executor] Execution context initialized (empty)")
        
        print("="*60 + "\n")
        
        # Only reset execution history, keep context for next command
        self.execution_history = []
        
        try:
            for idx, task in enumerate(plan.tasks):
                print("\n" + "-"*60)
                print(f"[Executor] >>> EXECUTING TASK {task.task_id}/{len(plan.tasks)-1} <<<")
                print("-"*60)
                print(f"[Executor] Action: {task.action}")
                print(f"[Executor] Description: {task.description}")
                if task.parameters:
                    print(f"[Executor] Parameters: {task.parameters}")
                print(f"[Executor] Progress: {idx+1}/{len(plan.tasks)} tasks completed")
                
                # Show current context if relevant
                if self.execution_context:
                    print(f"[Executor] Current context: {list(self.execution_context.keys())}")
                
                print(f"[Executor] → Calling robot function: {task.action}()...")
                result = self._execute_task(task)
                
                if result["status"] != "success":
                    print(f"\n[Executor] ✗✗✗ TASK {task.task_id} FAILED ✗✗✗")
                    print(f"[Executor] Error: {result.get('message', 'Unknown error')}")
                    print("="*60)
                    return {
                        "status": "error",
                        "message": f"Task {task.task_id} failed: {result.get('message', 'Unknown error')}",
                        "failed_task": task.task_id,
                        "execution_history": self.execution_history
                    }
                
                # Store result in context for next tasks
                self._update_context(task, result)
                self.execution_history.append({
                    "task_id": task.task_id,
                    "action": task.action,
                    "result": result
                })
                
                print(f"[Executor] ✓✓✓ TASK {task.task_id} COMPLETED SUCCESSFULLY ✓✓✓")
                if result.get("message"):
                    print(f"[Executor] Result message: {result.get('message')}")
            
            print("\n" + "="*60)
            print("[Executor] ===== PLAN EXECUTION COMPLETED SUCCESSFULLY =====")
            print("="*60)
            print(f"[Executor] Total tasks executed: {len(self.execution_history)}")
            print(f"[Executor] Final context keys: {list(self.execution_context.keys())}")
            print("="*60 + "\n")
            
            return {
                "status": "success",
                "message": "All tasks executed successfully",
                "execution_history": self.execution_history
            }
            
        except Exception as e:
            print(f"\n[Executor] ✗✗✗ FATAL ERROR: Execution failed ✗✗✗")
            print(f"[Executor] Error: {str(e)}")
            print("="*60 + "\n")
            return {
                "status": "error",
                "message": f"Execution failed: {str(e)}",
                "execution_history": self.execution_history
            }
    
    def _execute_task(self, task: Task) -> Dict[str, Any]:
        """
        Execute a single task.
        
        Args:
            task: Task object to execute
            
        Returns:
            Dictionary with execution result
        """
        action = task.action
        parameters = task.parameters or {}
        
        try:
            if action == "move_home":
                return move_home()
            
            elif action == "move":
                # Check if this is a relative movement
                is_relative = parameters.get("relative", False)
                
                if is_relative:
                    # Handle relative movement
                    direction = parameters.get("direction", "").lower()
                    distance = parameters.get("distance")
                    
                    if not direction:
                        return {
                            "status": "error",
                            "message": "Relative movement requires 'direction' parameter"
                        }
                    
                    # Use default distance if not specified
                    if distance is None:
                        distance = config.ROBOT_RELATIVE_MOVE_DEFAULT_DISTANCE
                        print(f"[Executor] → Using default relative move distance: {distance}m")
                    
                    # Get current position from context, or use home position as fallback
                    if "last_position" in self.execution_context:
                        current_pos = self.execution_context["last_position"].copy()
                        print(f"[Executor] → Relative move: Using current position from context: {current_pos}")
                    else:
                        current_pos = config.ROBOT_HOME_POSITION.copy()
                        print(f"[Executor] → Relative move: No context position, using home position: {current_pos}")
                    
                    # Get direction mapping
                    if direction not in config.ROBOT_DIRECTION_MAPPING:
                        return {
                            "status": "error",
                            "message": f"Unknown direction '{direction}'. Valid directions: {list(config.ROBOT_DIRECTION_MAPPING.keys())}"
                        }
                    
                    direction_info = config.ROBOT_DIRECTION_MAPPING[direction]
                    axis = direction_info["axis"]
                    sign = direction_info["sign"]
                    
                    # Calculate target position
                    target_x = current_pos.get("x", 0.0)
                    target_y = current_pos.get("y", 0.0)
                    target_z = current_pos.get("z", 0.0)
                    
                    if axis == "x":
                        target_x += sign * distance
                    elif axis == "y":
                        target_y += sign * distance
                    elif axis == "z":
                        target_z += sign * distance
                    
                    print(f"[Executor] → Relative move: {direction} by {distance}m")
                    print(f"[Executor] → Current position: {current_pos}")
                    print(f"[Executor] → Target position: x={target_x}, y={target_y}, z={target_z}")
                    
                    return move(x=target_x, y=target_y, z=target_z)
                
                else:
                    # Handle absolute movement (existing logic)
                    # Extract coordinates, using context if available
                    x = parameters.get("x")
                    y = parameters.get("y")
                    z = parameters.get("z")
                    
                    print(f"[Executor] → Initial move parameters: x={x}, y={y}, z={z}")
                    
                    # Check if this is a bin location (predefined coordinates)
                    if x is not None and y is not None and z is not None:
                        bin_coords = config.BIN_COORDINATES.get("bin", {})
                        if (abs(x - bin_coords.get("x", 0)) < 0.001 and
                            abs(y - bin_coords.get("y", 0)) < 0.001 and
                            abs(z - bin_coords.get("z", 0)) < 0.001):
                            print(f"[Executor] → Detected bin location: ({x}, {y}, {z})")
                    
                    # If coordinates are None, try to get from context (from see() result)
                    if x is None and "last_vision_coordinates" in self.execution_context:
                        x = self.execution_context["last_vision_coordinates"].get("x")
                        print(f"[Executor] → Using x from context (vision): {x}")
                    if y is None and "last_vision_coordinates" in self.execution_context:
                        y = self.execution_context["last_vision_coordinates"].get("y")
                        print(f"[Executor] → Using y from context (vision): {y}")
                    if z is None and "last_vision_coordinates" in self.execution_context:
                        z = self.execution_context["last_vision_coordinates"].get("z")
                        print(f"[Executor] → Using z from context (vision): {z}")
                    
                    print(f"[Executor] → Final move parameters: x={x}, y={y}, z={z}")
                    return move(x=x, y=y, z=z)
            
            elif action == "grasp":
                grasp_value = parameters.get("grasp", True)
                return grasp(grasp_value)
            
            elif action == "see":
                target = parameters.get("target", "")
                if not target:
                    return {
                        "status": "error",
                        "message": "see() action requires 'target' parameter"
                    }
                
                # Check if robot is at home position before calling see()
                # Vision recognition requires the robot to be at a fixed home position
                current_pos = self.execution_context.get("last_position", config.ROBOT_HOME_POSITION.copy())
                home_pos = config.ROBOT_HOME_POSITION
                
                # Check if current position is at home (with small tolerance for floating point comparison)
                is_at_home = (
                    abs(current_pos.get("x", 0) - home_pos["x"]) < 0.001 and
                    abs(current_pos.get("y", 0) - home_pos["y"]) < 0.001 and
                    abs(current_pos.get("z", 0) - home_pos["z"]) < 0.001
                )
                
                if not is_at_home:
                    print(f"[Executor] ⚠ Robot is not at home position (current: {current_pos}, home: {home_pos})")
                    print(f"[Executor] → Automatically moving to home position before vision recognition...")
                    
                    # Move to home first
                    move_home_result = move_home()
                    
                    if move_home_result.get("status") != "success":
                        return {
                            "status": "error",
                            "message": f"Failed to move to home position before see(): {move_home_result.get('message', 'Unknown error')}"
                        }
                    
                    # Update context with home position
                    self.execution_context["last_position"] = move_home_result.get("position", home_pos.copy())
                    print(f"[Executor] ✓ Robot moved to home position successfully")
                
                # Now execute see()
                return see(target)
            
            else:
                return {
                    "status": "error",
                    "message": f"Unknown action: {action}"
                }
                
        except Exception as e:
            return {
                "status": "error",
                "message": f"Error executing {action}: {str(e)}"
            }
    
    def _update_context(self, task: Task, result: Dict[str, Any]):
        """
        Update execution context with task result.
        
        Args:
            task: Task that was executed
            result: Result from task execution
        """
        # Store vision results for use in subsequent move() calls
        if task.action == "see" and result.get("status") == "success":
            coordinates = result.get("coordinates") or result.get("position")
            if coordinates:
                self.execution_context["last_vision_coordinates"] = coordinates
                self.execution_context["last_vision_target"] = result.get("target")
                print(f"[Executor] → Context updated: stored vision coordinates {coordinates}")
                print(f"[Executor] → Context updated: stored vision target '{result.get('target')}'")
        
        # Store last position for reference (for both move and move_home)
        if task.action in ["move", "move_home"] and result.get("status") == "success":
            position = result.get("position")
            if position:
                self.execution_context["last_position"] = position
                print(f"[Executor] → Context updated: stored last position {position}")
        
        # Store grasp state
        if task.action == "grasp" and result.get("status") == "success":
            self.execution_context["grasp_state"] = result.get("grasp_state", False)
            print(f"[Executor] → Context updated: stored grasp state = {result.get('grasp_state', False)}")
    
    def reset(self):
        """Reset executor state."""
        self.execution_context = {}
        self.execution_history = []

