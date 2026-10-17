"""What a boundary record holds for the M-a1 replay (plan M-a1 slice 7.0)."""

import importlib.util
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from forklift_core.mission.boundary_record import BoundaryRecorder

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("boundary_audit", ROOT / "tools/boundary_audit.py")
A = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(A)


@dataclass
class Plan:
    success: bool
    poses: np.ndarray


class Memory:
    pass


class Refused(RuntimeError):
    def __init__(self, reason):
        super().__init__(reason)
        self.reason, self.memory = reason, Memory()


def test_the_audit_counts_calls_and_lists_what_a_replay_cannot_hand_back_by_value(tmp_path):
    rec = BoundaryRecorder(tmp_path, chunk_ticks=1)
    rec.tick_begin(-1, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    rec.call("Planner", "plan_transport", request={"args": [1.0]}, response=Plan(True, np.zeros((2, 3))))
    rec.tick_end(outputs={"setup": True}, branches=[])
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    rec.call("Planner", "grid_kwargs", request={"args": []}, response={"memory": Memory()},
             changes={"watched": {"obstacle": {"plans": {"$append": [1]}}}})
    rec.call("Capture", "capture", request={"kwargs": {}}, exception=Refused("no_frame"))
    rec.call("Planner", "bicycle_rollout", request={"args": [[1]]}, response=None,
             changes={"request_after": {"args": [[2]]}})
    rec.call("Vehicle", "get_world_pose", request={"args": []}, response=[np.zeros(3), np.ones(4)])
    rec.tick_end(outputs={"speed_mps": 0.1}, branches=["state.transitions"])
    rec.close()
    out = A.audit(tmp_path)
    assert out["ticks"] == 2 and out["setup_ticks"] == 1 and (out["first_tick"], out["last_tick"]) == (-1, 0)
    assert out["calls"] == {"Planner.plan_transport": 1, "Planner.grid_kwargs": 1, "Capture.capture": 1,
                            "Planner.bicycle_rollout": 1, "Vehicle.get_world_pose": 1}
    assert out["opaque"] == {"Planner.grid_kwargs": {"response:Memory": 1}, "Capture.capture": {"exception:Memory": 1}}
    assert out["dataclasses"] == {"Planner.plan_transport": {"Plan": 1}}
    assert out["exceptions"] == {"Capture.capture": {"Refused": 1}}
    assert out["request_after"] == {"Planner.bicycle_rollout": 1}
    assert out["watched"] == {"Planner.grid_kwargs": {"obstacle.plans": 1}}
    assert out["meta"]["opaque_values"] == 2 and out["output_keys"] == {"setup": 1, "speed_mps": 1}
