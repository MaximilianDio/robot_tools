"""robot_tools - Modeling and collision-checking tools for robot path planning.

Installing this distribution also provides the companion packages
``robot_visualization`` (PyVista-based 3D visualization), ``robot_video_tools``
(video generation and camera-calibrated overlays), and the vendored ``urdfpy``
fork used for URDF parsing.
"""

from robot_tools.collisions import (
    BoxCollision,
    CollisionObject,
    CylinderCollision,
    EllipsoidCollision,
    create_collision_objects,
    register_collision_type,
)
from robot_tools.robot_model import RobotModel

__version__ = "0.2.0"
__author__ = "Maximilian Dio"

__all__ = [
    "RobotModel",
    "CollisionObject",
    "EllipsoidCollision",
    "CylinderCollision",
    "BoxCollision",
    "create_collision_objects",
    "register_collision_type",
]
