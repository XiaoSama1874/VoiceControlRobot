"""
Unit Tests for Full Pipeline (LLM + Executor)

Run with: python -m unittest test.test_full_pipeline
"""

import unittest
import json
import os
from llm_module import LLMPlanner
from executor_module import Executor


class TestFullPipeline(unittest.TestCase):
    """Unit tests for full pipeline (LLM + Executor)."""
    
    # Directory to save generated plans
    OUTPUT_DIR = "test/llm_plans"
    
    def setUp(self):
        """Set up test fixtures."""
        try:
            self.planner = LLMPlanner()
            self.executor = Executor()
            # Create output directory if it doesn't exist
            os.makedirs(self.OUTPUT_DIR, exist_ok=True)
        except Exception as e:
            self.skipTest(f"Could not initialize components: {e}")
    
    def _save_plan_to_file(self, plan, test_name: str, command: str):
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
    
    def test_full_pipeline_simple_command(self):
        """Test full pipeline with simple command."""
        command = "pick up the red square"
        
        # Generate plan
        plan = self.planner.generate_plan(command)
        self.assertIsNotNone(plan, "Plan should be generated")
        
        # Save plan to file
        if plan:
            self._save_plan_to_file(plan, "test_full_pipeline_simple_command", command)
        
        # Execute plan
        result = self.executor.execute_plan(plan)
        self.assertEqual(result["status"], "success", "Execution should succeed")
        
        print(f"\n✓ Full pipeline completed: {len(plan.tasks)} tasks generated, "
              f"{len(result['execution_history'])} tasks executed")
    
    def test_full_pipeline_complex_command(self):
        """Test full pipeline with complex command."""
        command = "pick up the red square and move it to the right"
        
        # Generate plan
        plan = self.planner.generate_plan(command)
        self.assertIsNotNone(plan, "Plan should be generated")
        
        # Save plan to file
        if plan:
            self._save_plan_to_file(plan, "test_full_pipeline_complex_command", command)
        
        # Execute plan
        result = self.executor.execute_plan(plan)
        self.assertEqual(result["status"], "success", "Execution should succeed")
        
        print(f"\n✓ Full pipeline completed: {len(plan.tasks)} tasks generated, "
              f"{len(result['execution_history'])} tasks executed")
    
    def test_full_pipeline_data_flow(self):
        """Test that data flows correctly from see() to move()."""
        command = "pick up the red square"
        
        # Generate plan
        plan = self.planner.generate_plan(command)
        if plan is None:
            self.skipTest("Could not generate plan")
        
        # Save plan to file
        self._save_plan_to_file(plan, "test_full_pipeline_data_flow", command)
        
        # Check if plan has see() followed by move()
        has_see = any(task.action == "see" for task in plan.tasks)
        has_move = any(task.action == "move" for task in plan.tasks)
        
        if has_see and has_move:
            # Execute plan
            result = self.executor.execute_plan(plan)
            self.assertEqual(result["status"], "success", "Execution should succeed")
            print("\n✓ Data flow from see() to move() works correctly")
        else:
            self.skipTest("Plan does not contain see() and move() actions")
    
    def test_full_pipeline_error_handling(self):
        """Test error handling in full pipeline."""
        # Test with an invalid plan (empty tasks)
        from llm_module.plan_parser import ExecutionPlan
        
        empty_plan = ExecutionPlan(tasks=[])
        result = self.executor.execute_plan(empty_plan)
        
        self.assertEqual(result["status"], "error", "Empty plan should return error")
        print("\n✓ Error handling works correctly for empty plan")
    
    def test_full_pipeline_multiple_commands(self):
        """Test full pipeline with multiple different commands."""
        commands = [
            "pick up the red square",
            "move the blue cube to the left",
            "grasp the green circle"
        ]
        
        for command in commands:
            plan = self.planner.generate_plan(command)
            if plan:
                result = self.executor.execute_plan(plan)
                self.assertEqual(result["status"], "success", 
                               f"Execution should succeed for: {command}")
        
        print(f"\n✓ Successfully processed {len(commands)} different commands")


if __name__ == "__main__":
    unittest.main(verbosity=2)

