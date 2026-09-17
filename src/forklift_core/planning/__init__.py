"""Hardware-independent planar planning with explicit collision geometry."""

from .geometry import (
    Bounds,
    Footprint,
    FootprintCollisionChecker,
    Pose2D,
    Rectangle,
    collision_free_path,
    collision_free_pose,
)
from .hybrid_astar import PlannerConfig, PlanResult, plan_hybrid_astar

__all__ = [
    "Bounds",
    "Footprint",
    "FootprintCollisionChecker",
    "Pose2D",
    "Rectangle",
    "collision_free_path",
    "collision_free_pose",
    "PlanResult",
    "PlannerConfig",
    "plan_hybrid_astar",
]
