"""
Unit Tests for Plan Parser Module

Run with: python -m unittest test.test_plan_parser
"""

import unittest
from llm_module.plan_parser import PlanParser, Task, ExecutionPlan


class TestPlanParser(unittest.TestCase):
    """Unit tests for Plan Parser."""
    test_json_str = '''{
            "tasks": [
                {
                "task_id": 0,
                "action": "move_home",
                "description": "Move robot arm to home position",
                "parameters": {}
                },
                {
                "task_id": 1,
                "action": "see",
                "description": "Use vision to locate the red square",
                "parameters": {
                    "target": "red_square"
                }
                },
                {
                "task_id": 2,
                "action": "move",
                "description": "Move robot arm to the red square location",
                "parameters": {
                    "x": null,
                    "y": null,
                    "z": null
                }
                },
                {
                "task_id": 3,
                "action": "grasp",
                "description": "Grasp the red square with the end effector",
                "parameters": {
                    "grasp": true
                }
                },
                {
                "task_id": 4,
                "action": "move_home",
                "description": "Return robot arm to home position with the red square",
                "parameters": {}
                }
            ]
    }'''
    def setUp(self):
        """Set up test fixtures."""
        self.parser = PlanParser()
    
    def test_parse_valid_json(self):
        """Test parsing valid JSON plan."""
       
        
        plan = self.parser.parse(self.test_json_str)
        self.assertIsNotNone(plan, "Plan should be parsed")
        self.assertEqual(len(plan.tasks), 5, "Should have 2 tasks")
        print("\n✓ Parsed valid JSON plan correctly")
    
    def test_parse_invalid_json(self):
        """Test parsing invalid JSON."""
        json_str = "invalid json"
        plan = self.parser.parse(json_str)
        self.assertIsNone(plan, "Invalid JSON should return None")
        print("\n✓ Correctly rejected invalid JSON")
    
    def test_parse_missing_tasks_key(self):
        """Test parsing JSON without tasks key."""
        json_str = '{"invalid": "structure"}'
        plan = self.parser.parse(json_str)
        self.assertIsNone(plan, "JSON without tasks should return None")
        print("\n✓ Correctly rejected JSON without tasks key")
    
    def test_parse_task_missing_fields(self):
        """Test parsing task with missing required fields."""
        json_str = '''{
            "tasks": [
                {
                    "task_id": 0
                }
            ]
        }'''
        plan = self.parser.parse(json_str)
        self.assertIsNone(plan, "Task with missing fields should return None")
        print("\n✓ Correctly rejected task with missing fields")
    
    def test_validate_plan(self):
        """Test plan validation."""
        tasks = [
            Task(task_id=0, action="move_home", description="Move to home"),
            Task(task_id=1, action="see", description="See object", 
                 parameters={"target": "red_square"})
        ]
        
        plan = ExecutionPlan(tasks=tasks)
        is_valid = self.parser.validate_plan(plan)
        
        self.assertTrue(is_valid, "Valid plan should pass validation")
        print("\n✓ Plan validation works correctly")
    
    def test_validate_plan_invalid_task_ids(self):
        """Test validation of plan with invalid task IDs."""
        tasks = [
            Task(task_id=0, action="move_home", description="Move to home"),
            Task(task_id=2, action="see", description="See object",  # Missing task_id 1
                 parameters={"target": "red_square"})
        ]
        
        plan = ExecutionPlan(tasks=tasks)
        is_valid = self.parser.validate_plan(plan)
        
        self.assertFalse(is_valid, "Plan with non-sequential task IDs should fail validation")
        print("\n✓ Correctly rejected plan with invalid task IDs")
    
    def test_parse_task_with_parameters(self):
        """Test parsing task with parameters."""
        json_str = '''{
            "tasks": [
                {
                    "task_id": 0,
                    "action": "move",
                    "description": "Move to position",
                    "parameters": {"x": 10.0, "y": 5.0, "z": 2.0}
                }
            ]
        }'''
        
        plan = self.parser.parse(json_str)
        self.assertIsNotNone(plan, "Plan should be parsed")
        self.assertEqual(len(plan.tasks), 1, "Should have 1 task")
        self.assertIsNotNone(plan.tasks[0].parameters, "Task should have parameters")
        self.assertEqual(plan.tasks[0].parameters["x"], 10.0, "Parameter x should be 10.0")
        print("\n✓ Parsed task with parameters correctly")
    
    def test_parse_task_with_null_parameters(self):
        """Test parsing task with null parameters."""
        json_str = '''{
            "tasks": [
                {
                    "task_id": 0,
                    "action": "move",
                    "description": "Move to position",
                    "parameters": {"x": null, "y": null, "z": null}
                }
            ]
        }'''
        
        plan = self.parser.parse(json_str)
        self.assertIsNotNone(plan, "Plan should be parsed")
        self.assertIsNotNone(plan.tasks[0].parameters, "Task should have parameters")
        self.assertIsNone(plan.tasks[0].parameters["x"], "Parameter x should be null")
        print("\n✓ Parsed task with null parameters correctly")


if __name__ == "__main__":
    unittest.main(verbosity=2)

