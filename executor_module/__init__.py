"""
Voice Control Robot - Executor Module

This module executes execution plans by converting tasks into function calls.
"""

from .executor import Executor
from .robot_functions import move, grasp, see, move_home

__all__ = ['Executor', 'move', 'grasp', 'see', 'move_home']

