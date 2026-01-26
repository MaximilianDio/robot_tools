"""
robot_tools - Visualization and modeling tools for robot path planning and control

This package provides tools for robot modeling, collision detection, and visualization.
It includes the robot_visualization subpackage for 3D visualization capabilities.
"""

__version__ = "0.1.0"
__author__ = "Maximilian Dio"

# Import main classes for easy access
from robot_tools.robot_model import RobotModel
from robot_tools.collisions import (
    EllipsoidCollision,
    CylinderCollision,
    BoxCollision,
    create_collision_objects
)

# Make these available at package level
__all__ = [
    "RobotModel",
    "EllipsoidCollision",
    "CylinderCollision",
    "BoxCollision",
    "create_collision_objects",
]
