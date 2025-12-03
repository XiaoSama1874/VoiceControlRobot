"""
Plan Parser Module

This module parses and validates JSON execution plans from LLM.
"""

import json
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
import config


@dataclass
class Task:
    """Represents a single task in the execution plan."""
    task_id: int
    action: str
    description: str
    parameters: Optional[Dict[str, Any]] = None


@dataclass
class ExecutionPlan:
    """Represents a complete execution plan."""
    tasks: List[Task]
    
    def __repr__(self):
        return f"ExecutionPlan(tasks={len(self.tasks)})"


class PlanParser:
    """Parses and validates JSON execution plans."""
    
    def __init__(self):
        """Initialize the plan parser."""
        # Valid actions that can be executed (from config)
        self.VALID_ACTIONS = config.VALID_ROBOT_ACTIONS
    
    def parse(self, json_str: str) -> Optional[ExecutionPlan]:
        """
        Parse JSON string into ExecutionPlan object.
        
        Args:
            json_str: JSON string containing the execution plan
            
        Returns:
            ExecutionPlan object if valid, None otherwise (or empty ExecutionPlan for empty JSON)
        """
        try:
            # Parse JSON
            data = json.loads(json_str)
            
            # Handle empty JSON or empty tasks
            if not isinstance(data, dict):
                print("[PlanParser] Warning: JSON is not a dictionary, returning empty plan")
                return ExecutionPlan(tasks=[])
            
            if "tasks" not in data:
                print("[PlanParser] Warning: Missing 'tasks' key in JSON, returning empty plan")
                return ExecutionPlan(tasks=[])
            
            tasks_data = data["tasks"]
            if not isinstance(tasks_data, list):
                print("[PlanParser] Error: 'tasks' must be a list")
                return ExecutionPlan(tasks=[])
            
            # Handle empty tasks list
            if len(tasks_data) == 0:
                print("[PlanParser] Warning: Empty tasks list, returning empty plan")
                return ExecutionPlan(tasks=[])
            
            # Parse tasks
            tasks = []
            for task_data in tasks_data:
                task = self._parse_task(task_data)
                if task is None:
                    # If any task fails to parse, return empty plan instead of None
                    print("[PlanParser] Warning: Failed to parse a task, returning empty plan")
                    return ExecutionPlan(tasks=[])
                tasks.append(task)
            
            # Validate task IDs are sequential
            if not self._validate_task_ids(tasks):
                print("[PlanParser] Warning: Invalid task IDs, returning empty plan")
                return ExecutionPlan(tasks=[])
            
            return ExecutionPlan(tasks=tasks)
            
        except json.JSONDecodeError as e:
            print(f"[PlanParser] JSON decode error: {e}")
            return None
        except Exception as e:
            print(f"[PlanParser] Unexpected error: {e}")
            return None
    
    def _parse_task(self, task_data: Dict[str, Any]) -> Optional[Task]:
        """
        Parse a single task from dictionary.
        
        Args:
            task_data: Dictionary containing task information
            
        Returns:
            Task object if valid, None otherwise
        """
        try:
            # Required fields
            if "task_id" not in task_data:
                print("[PlanParser] Error: Missing 'task_id' in task")
                return None
            if "action" not in task_data:
                print("[PlanParser] Error: Missing 'action' in task")
                return None
            if "description" not in task_data:
                print("[PlanParser] Error: Missing 'description' in task")
                return None
            
            task_id = task_data["task_id"]
            action = task_data["action"]
            description = task_data["description"]
            parameters = task_data.get("parameters", {})
            
            # Validate task_id
            if not isinstance(task_id, int) or task_id < 0:
                print(f"[PlanParser] Error: Invalid task_id: {task_id}")
                return None
            
            # Validate action
            if action not in self.VALID_ACTIONS:
                print(f"[PlanParser] Warning: Unknown action '{action}'. Valid actions: {self.VALID_ACTIONS}")
                # Allow unknown actions but warn (for extensibility)
            
            return Task(
                task_id=task_id,
                action=action,
                description=description,
                parameters=parameters if parameters else None
            )
            
        except Exception as e:
            print(f"[PlanParser] Error parsing task: {e}")
            return None
    
    def _validate_task_ids(self, tasks: List[Task]) -> bool:
        """
        Validate that task IDs are sequential starting from 0.
        
        Args:
            tasks: List of Task objects
            
        Returns:
            True if valid, False otherwise
        """
        if not tasks:
            print("[PlanParser] Error: Empty task list")
            return False
        
        task_ids = [task.task_id for task in tasks]
        expected_ids = list(range(len(tasks)))
        
        if sorted(task_ids) != expected_ids:
            print(f"[PlanParser] Error: Task IDs must be sequential from 0. Got: {task_ids}")
            return False
        
        return True
    
    def validate_plan(self, plan: ExecutionPlan) -> bool:
        """
        Validate an execution plan.
        
        Args:
            plan: ExecutionPlan object to validate
            
        Returns:
            True if valid, False otherwise
        """
        if not plan or not plan.tasks:
            print("[PlanParser] Error: Empty execution plan")
            return False
        
        # Check task IDs are sequential
        if not self._validate_task_ids(plan.tasks):
            return False
        
        # Check all actions are valid (warning only)
        for task in plan.tasks:
            if task.action not in self.VALID_ACTIONS:
                print(f"[PlanParser] Warning: Task {task.task_id} has unknown action: {task.action}")
        
        return True

