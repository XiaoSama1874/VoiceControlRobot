"""
LLM Planner Module

This module generates execution plans from natural language commands using OpenAI GPT-5.
"""

import os
import json
import time
from typing import Optional, Dict, Any
from openai import OpenAI
from dotenv import load_dotenv
from .plan_parser import PlanParser, ExecutionPlan
import config

# Load environment variables
load_dotenv()


class LLMPlanner:
    """Generates execution plans using OpenAI GPT-5."""
    
    def __init__(
        self,
        api_key: Optional[str] = None,
        model: str = None,
        temperature: float = None,
        max_tokens: int = None
    ):
        """
        Initialize the LLM planner.
        
        Args:
            api_key: OpenAI API key (if None, reads from OPENAI_API_KEY env var)
            model: Model name to use (default: from config)
            temperature: Sampling temperature (default: from config)
            max_tokens: Maximum tokens in response (default: from config)
        """
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("OpenAI API key not found. Set OPENAI_API_KEY environment variable or pass api_key parameter.")
        
        self.client = OpenAI(api_key=self.api_key)
        self.model = model or os.getenv("OPENAI_MODEL", config.LLM_MODEL)
        self.temperature = float(os.getenv("OPENAI_TEMPERATURE", temperature or config.LLM_TEMPERATURE))
        self.max_tokens = int(os.getenv("OPENAI_MAX_TOKENS", max_tokens or config.LLM_MAX_TOKENS))
        self.plan_parser = PlanParser()
        
        # Build system prompt
        self.system_prompt = self._build_system_prompt()
    
    def _build_system_prompt(self) -> str:
        """Build the system prompt for the LLM."""
        return """You are a robot task planner. Your job is to convert natural language commands into structured execution plans.

Available Actions:
1. move_home - Move the robot arm to home position (no parameters needed)
2. move(x, y, z) - Move the robot arm to coordinates (x, y, z). Use null if coordinates are unknown and need to be determined.
   - For ABSOLUTE movement: Use {"x": value, "y": value, "z": value} or null for unknown coordinates
   - For RELATIVE movement: Use {"relative": true, "direction": "right|left|forward|backward|back|up|down", "distance": value}
     * If distance is not specified, default distance (20cm) will be used
     * Direction mapping: right→x+, left→x-, forward→y+, backward/back→y-, up→z+, down→z-
     * Relative movement is based on the robot's current position
3. grasp(true/false) - Grasp (true) or release (false) the end effector
4. see(target) - Use vision to identify a target object and get its coordinates. Parameter: target (e.g., "red_cube", "blue_cube")
   - CRITICAL: You MUST call move_home() BEFORE calling see() because vision recognition requires the robot to be at a fixed home position
   - The robot must be at home position for accurate vision recognition

Output Format:
You must return a valid JSON object with the following structure:
{
  "tasks": [
    {
      "task_id": 0,
      "action": "move_home",
      "description": "Move to home position for vision recognition"
    },
    {
      "task_id": 1,
      "action": "see",
      "description": "Identify the red cube using vision",
      "parameters": {"target": "red_cube"}
    },
    {
      "task_id": 2,
      "action": "move",
      "description": "Move arm to red cube location",
      "parameters": {"x": null, "y": null, "z": null}
    },
    {
      "task_id": 3,
      "action": "grasp",
      "description": "Grasp the red cube",
      "parameters": {"grasp": true}
    },
    {
      "task_id": 4,
      "action": "move",
      "description": "Move robot arm to the right",
      "parameters": {"relative": true, "direction": "right", "distance": 20}
    },
    {
      "task_id": 5,
      "action": "move",
      "description": "Move robot arm forward (using default distance)",
      "parameters": {"relative": true, "direction": "forward"}
    }
  ]
}

Important Rules:
- Task IDs must be sequential starting from 0
- CRITICAL: ALWAYS call move_home() IMMEDIATELY BEFORE any see() action. Vision recognition requires the robot to be at home position.
- Do NOT start with move_home unless explicitly requested OR if the plan includes see() action
- Do NOT end with move_home unless the task involves object manipulation (grasp/release) or explicitly requested
- For simple movement commands (e.g., "move right", "move forward") that don't involve grasping objects or vision, do NOT add move_home at the end
- Use see() action to identify objects before moving to them
- Use null for coordinates that need to be determined by vision
- For relative movement commands (e.g., "move right", "move forward", "move the red cube to the right"):
  * Use {"relative": true, "direction": "direction_name", "distance": value_in_cm}
  * If distance is not mentioned, omit the "distance" field to use default (20cm)
  * Valid directions: "right", "left", "forward", "backward", "back", "up", "down"
- Be specific in descriptions
- Return ONLY valid JSON, no additional text or markdown formatting"""
    
    def generate_plan(self, command: str, max_retries: int = None) -> Optional[ExecutionPlan]:
        """
        Generate an execution plan from a natural language command.
        
        Args:
            command: Natural language command from voice recognition
            max_retries: Maximum number of retry attempts (default: from config)
            
        Returns:
            ExecutionPlan object if successful, None otherwise
        """
        if max_retries is None:
            max_retries = config.LLM_MAX_RETRIES
        
        print("\n" + "="*60)
        print("[LLMPlanner] ===== PLAN GENERATION START =====")
        print("="*60)
        print(f"[LLMPlanner] Input command: '{command}'")
        print(f"[LLMPlanner] Model: {self.model}")
        print(f"[LLMPlanner] Temperature: {self.temperature}")
        print(f"[LLMPlanner] Max tokens: {self.max_tokens}")
        
        user_prompt = f"Convert the following command into an execution plan: {command}"
        
        print("\n[LLMPlanner] --- System Prompt (first 200 chars) ---")
        print(self.system_prompt[:200] + "...")
        print("\n[LLMPlanner] --- User Prompt ---")
        print(user_prompt)
        print("="*60 + "\n")
        
        for attempt in range(max_retries):
            try:
                print(f"[LLMPlanner] → Attempt {attempt + 1}/{max_retries}: Calling OpenAI API...")
                
                # Call OpenAI API
                response = self.client.chat.completions.create(
                    model=self.model,
                    messages=[
                        {"role": "system", "content": self.system_prompt},
                        {"role": "user", "content": user_prompt}
                    ],
                    temperature=self.temperature
                    # max_tokens=self.max_tokens
                )
                
                # Extract response content
                content = response.choices[0].message.content.strip()
                print(f"[LLMPlanner] ✓ API response received")
                print(f"[LLMPlanner] → Response length: {len(content)} characters")
                print(f"[LLMPlanner] → Response preview (first 300 chars):")
                print("   " + content[:300].replace('\n', '\\n') + "...")
                # save the content to a file
                with open("llm_response.txt", "w") as f:
                    f.write(content)
                
                # Try to extract JSON from response (in case it's wrapped in markdown)
                json_str = self._extract_json(content)
                print(f"[LLMPlanner] → Extracted JSON length: {len(json_str)} characters")
                
                # Parse the plan
                print(f"[LLMPlanner] → Parsing JSON...")
                plan = self.plan_parser.parse(json_str)
                
                if plan is not None:
                    # Check if plan is empty (LLM couldn't understand the command)
                    if len(plan.tasks) == 0:
                        print("[LLMPlanner] ⚠ Empty plan returned (LLM may not have understood the command)")
                        print("[LLMPlanner] → Returning empty execution plan")
                        print("="*60 + "\n")
                        return plan
                    
                    # Validate the plan
                    print(f"[LLMPlanner] → Validating plan...")
                    if self.plan_parser.validate_plan(plan):
                        print(f"[LLMPlanner] ✓✓✓ Plan generated successfully!")
                        print(f"[LLMPlanner] → Total tasks: {len(plan.tasks)}")
                        for task in plan.tasks:
                            print(f"[LLMPlanner]   - Task {task.task_id}: {task.action} - {task.description}")
                        print("="*60 + "\n")
                        return plan
                    else:
                        print("[LLMPlanner] ✗ Generated plan failed validation")
                        if attempt < max_retries - 1:
                            print("[LLMPlanner] → Retrying...")
                            time.sleep(1)  # Brief delay before retry
                            continue
                else:
                    print("[LLMPlanner] ✗ Failed to parse JSON from response")
                    if attempt < max_retries - 1:
                        print("[LLMPlanner] → Retrying...")
                        time.sleep(1)
                        continue
                
            except Exception as e:
                print(f"[LLMPlanner] ✗ Error on attempt {attempt + 1}: {e}")
                if attempt < max_retries - 1:
                    print("[LLMPlanner] → Retrying with exponential backoff...")
                    time.sleep(2 ** attempt)  # Exponential backoff
                else:
                    print("[LLMPlanner] ✗✗✗ Max retries reached. Giving up.")
        
        print("="*60)
        print("[LLMPlanner] ===== PLAN GENERATION FAILED =====")
        print("="*60 + "\n")
        return None
    
    def _extract_json(self, text: str) -> str:
        """
        Extract JSON from text, handling markdown code blocks.
        
        Args:
            text: Text that may contain JSON
            
        Returns:
            Extracted JSON string
        """
        # Remove markdown code blocks if present
        if "```json" in text:
            start = text.find("```json") + 7
            end = text.find("```", start)
            if end != -1:
                return text[start:end].strip()
        elif "```" in text:
            start = text.find("```") + 3
            end = text.find("```", start)
            if end != -1:
                return text[start:end].strip()
        
        # Try to find JSON object boundaries
        start = text.find("{")
        end = text.rfind("}") + 1
        
        if start != -1 and end > start:
            return text[start:end]
        
        return text.strip()

