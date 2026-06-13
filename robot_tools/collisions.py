"""Point-in-primitive collision checks for path planning.

Each primitive is described by a dictionary with a ``"type"`` key, a 4x4
homogeneous transform ``"T"`` placing it in the world frame, and
shape-specific size parameters, e.g.::

    {"type": "box", "T": np.eye(4), "xsize": 0.5, "ysize": 0.5, "zsize": 1.0}

To add a new primitive, subclass :class:`CollisionObject` and register it
with :func:`register_collision_type`; :func:`create_collision_objects` will
then accept the new ``"type"`` without further changes.
"""

import logging
from abc import ABC, abstractmethod

import numpy as np

logger = logging.getLogger(__name__)


class CollisionObject(ABC):
    """Base class for collision primitives placed by a homogeneous transform.

    Args:
        obstacle: Dictionary with at least the key ``"T"`` (4x4 homogeneous
            transform of the primitive's center in the world frame).
    """

    def __init__(self, obstacle: dict):
        self.obstacle = obstacle
        self.T = np.asarray(obstacle["T"], dtype=float)
        self.center = self.T[:3, 3]
        self.rotation_inv = np.linalg.inv(self.T[:3, :3])

    def to_local(self, point: np.ndarray) -> np.ndarray:
        """Transform a world-frame point into the primitive's local frame."""
        return self.rotation_inv @ (np.asarray(point) - self.center)

    @abstractmethod
    def is_in_collision(self, point: np.ndarray) -> bool:
        """Return True if the world-frame point lies inside or on the primitive."""


class EllipsoidCollision(CollisionObject):
    """Ellipsoid with semi-axes ``"xradius"``, ``"yradius"``, ``"zradius"``."""

    def __init__(self, obstacle: dict):
        super().__init__(obstacle)
        self.radii = np.array(
            [obstacle["xradius"], obstacle["yradius"], obstacle["zradius"]], dtype=float
        )
        logger.debug("Ellipsoid at %s with radii %s", self.center, self.radii)

    def is_in_collision(self, point: np.ndarray) -> bool:
        local_point = self.to_local(point)
        return np.sum((local_point / self.radii) ** 2) <= 1.0


class CylinderCollision(CollisionObject):
    """Cylinder along the local z-axis with ``"radius"`` and ``"height"``."""

    def __init__(self, obstacle: dict):
        super().__init__(obstacle)
        self.radius = float(obstacle["radius"])
        self.height = float(obstacle["height"])
        logger.debug(
            "Cylinder at %s with radius %s and height %s", self.center, self.radius, self.height
        )

    def is_in_collision(self, point: np.ndarray) -> bool:
        local_point = self.to_local(point)
        within_radius = local_point[0] ** 2 + local_point[1] ** 2 <= self.radius**2
        within_height = abs(local_point[2]) <= self.height / 2.0
        return within_radius and within_height


class BoxCollision(CollisionObject):
    """Axis-aligned box (in its local frame) with ``"xsize"``, ``"ysize"``, ``"zsize"``."""

    def __init__(self, obstacle: dict):
        super().__init__(obstacle)
        self.size = np.array(
            [obstacle["xsize"], obstacle["ysize"], obstacle["zsize"]], dtype=float
        )
        logger.debug("Box at %s with size %s", self.center, self.size)

    def is_in_collision(self, point: np.ndarray) -> bool:
        local_point = self.to_local(point)
        return bool(np.all(np.abs(local_point) <= self.size / 2.0))


_COLLISION_TYPES: dict = {}


def register_collision_type(name: str, cls: type) -> None:
    """Register a :class:`CollisionObject` subclass under an obstacle ``"type"`` name."""
    if not issubclass(cls, CollisionObject):
        raise TypeError(f"{cls.__name__} must inherit from CollisionObject")
    _COLLISION_TYPES[name] = cls


register_collision_type("ellipsoid", EllipsoidCollision)
register_collision_type("cylinder", CylinderCollision)
register_collision_type("box", BoxCollision)


def create_collision_objects(obstacles_list: list) -> list:
    """Build collision objects from a list of obstacle dictionaries.

    Args:
        obstacles_list: Obstacle dictionaries, each with a registered ``"type"``.

    Returns:
        List of :class:`CollisionObject` instances, one per obstacle.
    """
    objects = []
    for obstacle in obstacles_list:
        try:
            cls = _COLLISION_TYPES[obstacle["type"]]
        except KeyError:
            raise ValueError(
                f"Unknown obstacle type: {obstacle['type']!r}. "
                f"Registered types: {sorted(_COLLISION_TYPES)}"
            ) from None
        objects.append(cls(obstacle))
    return objects
