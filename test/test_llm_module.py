"""
Unit Tests for LLM Module

Run with: python -m unittest test.test_llm_module
"""

import unittest
import json
import os
from llm_module import LLMPlanner, ExecutionPlan


class TestLLMModule(unittest.TestCase):
    """Unit tests for LLM module."""
    
    # Directory to save generated plans
    OUTPUT_DIR = "test/llm_plans"
    
    def setUp(self):
        """Set up test fixtures."""
        try:
            self.planner = LLMPlanner()
            # Create output directory if it doesn't exist
            os.makedirs(self.OUTPUT_DIR, exist_ok=True)
        except Exception as e:
            self.skipTest(f"Could not initialize LLM Planner: {e}")
    
    def _save_plan_to_file(self, plan: ExecutionPlan, test_name: str, command: str):
        """
        Save execution plan to a file (JSON only).
        
        Args:
            plan: ExecutionPlan object to save
            test_name: Name of the test method (e.g., "test_generate_plan_simple_command")
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
    
    def test_generate_plan_simple_command(self):
        """Test generating plan from a simple command."""
        command = "pick up the red object"
        plan = self.planner.generate_plan(command)
        
        self.assertIsNotNone(plan, "Plan should be generated")
        self.assertGreater(len(plan.tasks), 0, "Plan should have at least one task")
        
        # Save plan to file
        if plan:
            self._save_plan_to_file(plan, "test_generate_plan_simple_command", command)
        
        print(f"\n✓ Generated {len(plan.tasks)} tasks for command: '{command}'")
    
    def test_generate_plan_complex_command(self):
        """Test generating plan from a complex command."""
        command = "pick up the red object and move it to the right"
        plan = self.planner.generate_plan(command)
        
        self.assertIsNotNone(plan, "Plan should be generated")
        self.assertGreater(len(plan.tasks), 0, "Plan should have at least one task")
        
        # Save plan to file
        if plan:
            self._save_plan_to_file(plan, "test_generate_plan_complex_command", command)
        
        print(f"\n✓ Generated {len(plan.tasks)} tasks for command: '{command}'")


    def test_generate_plan_complex_command2(self):
        """Test generating plan from a complex command."""
        command = "draw a square on the sky"
        plan = self.planner.generate_plan(command)
        
        self.assertIsNotNone(plan, "Plan should be generated")
        self.assertGreater(len(plan.tasks), 0, "Plan should have at least one task")
        
        # Save plan to file
        if plan:
            self._save_plan_to_file(plan, "draw_square_on_sky", command)
        
        print(f"\n✓ Generated {len(plan.tasks)} tasks for command: '{command}'")


    def test_generate_plan_complex_command3(self):
        """Test generating plan from a complex command."""
        command = "Stack all the blocks together."
        plan = self.planner.generate_plan(command)
        
        self.assertIsNotNone(plan, "Plan should be generated")
        self.assertGreater(len(plan.tasks), 0, "Plan should have at least one task")
        
        # Save plan to file
        if plan:
            self._save_plan_to_file(plan, "stack_blocks_together", command)
        
        print(f"\n✓ Generated {len(plan.tasks)} tasks for command: '{command}'")
    
    def test_generate_plan_complex_command4(self):
        """Test generating plan from a complex command."""
        
        command = "pick up the red object and place it in the red bin"
        plan = self.planner.generate_plan(command)
        
        self.assertIsNotNone(plan, "Plan should be generated")
        self.assertGreater(len(plan.tasks), 0, "Plan should have at least one task")
        
        # Save plan to file
        if plan:
            self._save_plan_to_file(plan, "pick_up_red_object_and_place_in_red_bin", command)
        
        print(f"\n✓ Generated {len(plan.tasks)} tasks for command: '{command}'")

    def test_generate_plan_complex_command5(self):
        """Test generating plan from a complex command."""
        
        command = "Draw a W in the air"
        plan = self.planner.generate_plan(command)
        
        self.assertIsNotNone(plan, "Plan should be generated")
        self.assertGreater(len(plan.tasks), 0, "Plan should have at least one task")
        
        # Save plan to file
        if plan:
            self._save_plan_to_file(plan, "draw w", command)
        
        print(f"\n✓ Generated {len(plan.tasks)} tasks for command: '{command}'")

    def test_generate_plan_complex_command6(self):
        """Test generating plan from a complex command."""
        
        command = "pick up the red object and place it in the red bin and then pick up the green object and place it in the green bin and return to home"
        plan = self.planner.generate_plan(command)
        
        self.assertIsNotNone(plan, "Plan should be generated")
        self.assertGreater(len(plan.tasks), 0, "Plan should have at least one task")
        
        # Save plan to file
        if plan:
            self._save_plan_to_file(plan, "pick two objects and place them in the bins and return to home", command)
        
        print(f"\n✓ Generated {len(plan.tasks)} tasks for command: '{command}'")

    def test_generate_plan_with_move_command(self):
        """Test generating plan for move command."""
        command = "move the blue object to position 10, 5, 2"
        plan = self.planner.generate_plan(command)
        
        self.assertIsNotNone(plan, "Plan should be generated")
        
        # Save plan to file
        if plan:
            self._save_plan_to_file(plan, "test_generate_plan_with_move_command", command)
        
        print(f"\n✓ Generated {len(plan.tasks)} tasks for command: '{command}'")
    
    def test_plan_structure(self):
        """Test that generated plan has correct structure."""
        command = "pick up the red object"
        plan = self.planner.generate_plan(command)
        
        if plan:
            # Check that plan has tasks
            self.assertIsNotNone(plan.tasks, "Plan should have tasks list")
            self.assertGreater(len(plan.tasks), 0, "Plan should have at least one task")
            
            # Check task structure
            for task in plan.tasks:
                self.assertIsNotNone(task.task_id, "Task should have task_id")
                self.assertIsNotNone(task.action, "Task should have action")
                self.assertIsNotNone(task.description, "Task should have description")
                self.assertIsInstance(task.task_id, int, "task_id should be integer")
                self.assertGreaterEqual(task.task_id, 0, "task_id should be non-negative")
            
            # Save plan to file
            self._save_plan_to_file(plan, "test_plan_structure", command)
            
            print(f"\n✓ Plan structure is valid with {len(plan.tasks)} tasks")


if __name__ == "__main__":
    unittest.main(verbosity=2)

