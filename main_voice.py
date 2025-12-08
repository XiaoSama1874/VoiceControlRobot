"""
Main test entry for voice module with LLM and executor integration.

This script demonstrates the complete pipeline:
1. Voice recognition and command detection
2. LLM plan generation
3. Plan execution
"""

import signal
import sys
from voice_module import VoiceListener
from llm_module import LLMPlanner
from executor_module import Executor
import config


def on_command_detected(command: str, planner: LLMPlanner, executor: Executor):
    """
    Callback function called when a complete command is detected.
    
    Args:
        command: The detected command text
        planner: LLMPlanner instance for generating execution plans
        executor: Executor instance for executing plans
    """
    print("\n" + "="*60)
    print("[Main] ===== COMMAND PROCESSING PIPELINE START =====")
    print("="*60)
    print(f"[Main] ✓ Command received from voice module: '{command}'")
    print("="*60 + "\n")
    
    # Generate execution plan using LLM
    print("[Main] → Step 1: Generating execution plan with LLM...")
    plan = planner.generate_plan(command)
    
    if plan is None:
        print("\n[Main] ✗ Step 1 failed: Could not generate execution plan")
        print("[Main] ===== PIPELINE ABORTED =====")
        print("="*60 + "\n")
        return
    
    # Check if plan is empty (LLM couldn't understand the command)
    if len(plan.tasks) == 0:
        print("\n[Main] ⚠ Step 1 completed: Empty execution plan generated")
        print("[Main] → LLM could not understand the command, returning empty plan")
        print("[Main] ===== PIPELINE COMPLETED (NO TASKS) =====")
        print("="*60 + "\n")
        return
    
    print("\n[Main] ✓ Step 1 completed: Execution plan generated")
    print("\n" + "="*60)
    print("[Main] EXECUTION PLAN SUMMARY:")
    print("="*60)
    for task in plan.tasks:
        print(f"[Main]   Task {task.task_id}: {task.action}")
        print(f"[Main]     Description: {task.description}")
        if task.parameters:
            print(f"[Main]     Parameters: {task.parameters}")
    print("="*60 + "\n")
    
    # Execute the plan
    print("[Main] → Step 2: Executing plan...")
    execution_result = executor.execute_plan(plan)
    
    print("\n[Main] ✓ Step 2 completed: Plan execution finished")
    print("="*60)
    if execution_result["status"] == "success":
        print("[Main] ===== PIPELINE COMPLETED SUCCESSFULLY =====")
        print(f"[Main] Total tasks executed: {len(execution_result.get('execution_history', []))}")
    else:
        print("[Main] ===== PIPELINE COMPLETED WITH ERRORS =====")
        print(f"[Main] Error: {execution_result.get('message', 'Unknown error')}")
    print("="*60 + "\n")


def main():
    """Main function to test voice listener with LLM integration."""
    print("Voice Control Robot - Voice & LLM Module Test")
    print("="*60)
    print("Instructions:")
    print(f"1. Say '{config.VOICE_START_KEYWORD}' to start a command")
    print("2. Say your command (e.g., 'pick up the red object and move it to the right')")
    print(f"3. Say '{config.VOICE_STOP_KEYWORD}' to end the command")
    print(f"4. The system will generate an execution plan using {config.LLM_MODEL}")
    print("5. Press Ctrl+C to exit")
    print("="*60 + "\n")
    
    # Initialize LLM planner
    try:
        planner = LLMPlanner()
        print("[LLM] LLM Planner initialized successfully")
    except Exception as e:
        print(f"[ERROR] Failed to initialize LLM Planner: {e}")
        print("Make sure you have set OPENAI_API_KEY in your .env file or environment variables.")
        sys.exit(1)
    
    # Initialize executor
    executor = Executor()
    print("[Executor] Executor initialized successfully\n")
    
    # Create callback wrapper
    def command_callback(command: str):
        on_command_detected(command, planner, executor)
    
    # Create voice listener (uses config defaults)
    listener = VoiceListener(
        on_command_detected=command_callback
    )
    
    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print("\n\nShutting down...")
        listener.stop_listening()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Start listening
    try:
        listener.start_listening()
        
        # Keep main thread alive
        while True:
            import time
            time.sleep(1)
            
    except KeyboardInterrupt:
        signal_handler(None, None)
    except Exception as e:
        print(f"Error: {e}")
        listener.stop_listening()
        sys.exit(1)


if __name__ == "__main__":
    main()

