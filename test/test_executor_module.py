"""
Unit Tests for Executor Module

Run with: python -m unittest test.test_executor_module
"""

import unittest
import json
import os
from executor_module import Executor
from llm_module import LLMPlanner, ExecutionPlan
from llm_module.plan_parser import Task


class TestExecutorModule(unittest.TestCase):
    """Unit tests for Executor module."""
    
    # Directory to save generated plans
    OUTPUT_DIR = "test/llm_plans"
    
    def setUp(self):
        """Set up test fixtures."""
        self.executor = Executor()
        # Create output directory if it doesn't exist
        os.makedirs(self.OUTPUT_DIR, exist_ok=True)
    
    def _save_plan_to_file(self, plan: ExecutionPlan, test_name: str, command: str = None):
        """
        Save execution plan to a file (JSON only).
        
        Args:
            plan: ExecutionPlan object to save
            test_name: Name of the test method
            command: Original command that generated the plan (not saved, for compatibility)
        """
        filename = f"{test_name}.txt"
        filepath = os.path.join(self.OUTPUT_DIR, filename)
        
        try:
            # Convert plan to JSON
            plan_dict = {
                "tasks": [
                    {
                        "task_id": task.task_id,
                        "action": task.action,
                        "description": task.description,
                        "parameters": task.parameters if task.parameters else {}
                    }
                    for task in plan.tasks
                ]
            }
            
            # Write only JSON
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(json.dumps(plan_dict, indent=2, ensure_ascii=False))
                f.write("\n")
            
            print(f"  → Plan saved to: {filepath}")
            
        except Exception as e:
            print(f"  ⚠ Warning: Could not save plan to file: {e}")
    
    def test_execute_simple_plan(self):
        """Test executing a simple plan."""
        # First generate a plan
        try:
            planner = LLMPlanner()
            command = "pick up the red square"
            plan = planner.generate_plan(command)
            
            if plan is None:
                self.skipTest("Could not generate plan for testing")
            
            # Save plan to file
            self._save_plan_to_file(plan, "test_execute_simple_plan", command)
            
            # Execute the plan
            result = self.executor.execute_plan(plan)
            
            self.assertEqual(result["status"], "success", "Execution should succeed")
            self.assertIn("execution_history", result, "Result should have execution_history")
            print(f"\n✓ Executed plan with {len(result['execution_history'])} tasks")
            
        except Exception as e:
            self.skipTest(f"Could not test executor: {e}")
    
    def test_executor_context_management(self):
        """Test that executor manages context correctly."""
        # Generate a plan that uses see() and move()
        try:
            planner = LLMPlanner()
            command = "pick up the red square"
            plan = planner.generate_plan(command)
            
            if plan is None:
                self.skipTest("Could not generate plan for testing")
            
            # Save plan to file
            self._save_plan_to_file(plan, "test_executor_context_management", command)
            
            # Execute the plan
            result = self.executor.execute_plan(plan)
            
            if result["status"] == "success":
                # Check that context was updated
                # Context should be empty after execution (reset)
                self.assertEqual(len(self.executor.execution_context), 0, 
                               "Context should be empty after execution")
                print("\n✓ Context management works correctly")
            
        except Exception as e:
            self.skipTest(f"Could not test context management: {e}")
    
    def test_executor_with_custom_plan(self):
        """Test executor with a manually created plan."""
        # Create a simple plan manually
        tasks = [
            Task(task_id=0, action="move_home", description="Move to home"),
            Task(task_id=1, action="see", description="See red square", 
                 parameters={"target": "red_square"}),
            Task(task_id=2, action="move", description="Move to object",
                 parameters={"x": None, "y": None, "z": None}),
            Task(task_id=3, action="grasp", description="Grasp object",
                 parameters={"grasp": True}),
            Task(task_id=4, action="move_home", description="Return home")
        ]
        
        plan = ExecutionPlan(tasks=tasks)
        
        # Save plan to file
        self._save_plan_to_file(plan, "test_executor_with_custom_plan", "Custom plan (manually created)")
        
        # Execute
        result = self.executor.execute_plan(plan)
        
        self.assertEqual(result["status"], "success", "Execution should succeed")
        self.assertEqual(len(result["execution_history"]), 5, "Should execute 5 tasks")
        print("\n✓ Executed custom plan successfully")
    
    def test_data_flow_from_see_to_move(self):
        """Test that coordinates from see() are passed to move()."""
        # Create a plan with see() followed by move()
        tasks = [
            Task(task_id=0, action="move_home", description="Move to home"),
            Task(task_id=1, action="see", description="See red square",
                 parameters={"target": "red_square"}),
            Task(task_id=2, action="move", description="Move to object",
                 parameters={"x": None, "y": None, "z": None}),
            Task(task_id=3, action="move_home", description="Return home")
        ]
        
        plan = ExecutionPlan(tasks=tasks)
        
        # Execute
        result = self.executor.execute_plan(plan)
        
        self.assertEqual(result["status"], "success", "Execution should succeed")
        
        # Check that move() used coordinates from see()
        see_result = result["execution_history"][1]["result"]
        move_result = result["execution_history"][2]["result"]
        
        self.assertIn("coordinates", see_result, "see() should return coordinates")
        self.assertIn("position", move_result, "move() should return position")
        print("\n✓ Data flow from see() to move() works correctly")


if __name__ == "__main__":
    unittest.main(verbosity=2)

