"""Moved to forklift_core.perception.near_field_tracking (plan D8c: the runner and the CPU
replay share it). Re-exported for the PR #2 insertion runner and its tests."""

from forklift_core.perception.near_field_tracking import HandoffSelection, RoofHandoff

__all__ = ["HandoffSelection", "RoofHandoff"]
