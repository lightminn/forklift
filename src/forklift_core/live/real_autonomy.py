"""The real truck's autonomy executor parts (docs/design/2026-11-19-real-assembly.md, 4b): the AUTO permit state the
final DRIVE and the watches read. No device is opened here; the 50 Hz loop that drives it is 4c.

States (design 7): OFF -> ARMING -> READY -> DRIVING, the S10 control stop STOP_PENDING -> DISARMING, and LATCHED.
The AUTO request is one new mission submit; the MCU ARM is asked once, after the core adopted the plan. Every latch
runs the same order: unsent candidates dropped, DISARM queued and a transmit tried, the arbiter STOPPED, then the
mission collected -- a collection failure cannot undo the first three. A latch is left only where the arbiter really
moves STOPPED -> PAUSED on a rearm the real conditions passed.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from forklift_core.mcu_link.messages import Mode

OFF, ARMING, READY, DRIVING, STOP_PENDING, DISARMING, LATCHED = (
    "OFF", "ARMING", "READY", "DRIVING", "STOP_PENDING", "DISARMING", "LATCHED")
STOP_ACK_S = 0.100  # the control stop's frame confirmed by a STATE naming its seq within this (design 7, provisional)


@dataclass
class PermitEvent:
    t_ns: int
    event: str
    reason: str | None = None
    detail: dict = field(default_factory=dict)


class AutoPermit:
    """One AUTO permit for the real truck. ``host`` is the ``McuLinkHost``, ``port`` the ``RealPlantPort``.
    ``stop_confirmed(now_ns)`` says the truck was seen standing on new samples after the last drive settled (the
    evidence the executor supplies, design 6)."""

    def __init__(self, host, port, *, stop_confirmed, stop_ack_s: float = STOP_ACK_S) -> None:
        self.host, self.port, self.stop_confirmed = host, port, stop_confirmed
        self.stop_ack_ns = int(stop_ack_s * 1e9)
        self.state = OFF
        self.reason: str | None = None
        self.events: list[PermitEvent] = []
        self._stop_seq: int | None = None
        self._stop_since_ns: int | None = None
        self._stop_boot_gen: tuple | None = None

    # --- what the rest reads ------------------------------------------------------------------------------------
    @property
    def armed(self) -> bool:
        """The watches' input (design 7): the arbiter drops ``armed`` on entering AUTO, so they read this."""
        if self.state in (ARMING, READY, DRIVING, STOP_PENDING, DISARMING):
            return True
        s = self.host.state  # a latch's DISARM not yet seen: the MCU may still be armed
        return self.state == LATCHED and s is not None and s.mode != Mode.DISARMED

    def drive_allowed(self) -> bool:
        """A mission DRIVE may be submitted (and only then)."""
        return self.state in (READY, DRIVING) and self.host.ready

    def _note(self, now_ns: int, event: str, reason: str | None = None, **detail) -> None:
        self.events.append(PermitEvent(int(now_ns), event, reason, detail))

    # --- arming ---------------------------------------------------------------------------------------------------
    def adopted(self, now_ns: int, *, pre_arm_refusal: str | None) -> bool:
        """The core adopted the submitted mission's plan (``adopted_id == mission_id``, running, approach) and the
        lease and generation were checked again by the caller: the control generation moves, the queue goes, and
        the MCU ARM is asked once. ``pre_arm_refusal``: the pre-ARM checks' first failure (design 6: DISARMED, a
        fresh stop, the old commands settled). Returns whether the ARM was asked."""
        if self.state != OFF:
            self.latch(now_ns, f"adopted_in_{self.state.lower()}")
            return False
        if pre_arm_refusal is not None:
            self.latch(now_ns, f"pre_arm:{pre_arm_refusal}")
            return False
        self.port.new_control_generation()
        blocked = self.host.request_arm(now_ns)
        if blocked is not None:
            self.latch(now_ns, f"arm_blocked:{getattr(blocked, 'reason', blocked)}")
            return False
        self.state = ARMING
        self._note(now_ns, "arm_requested")
        return True

    def drove(self, now_ns: int) -> None:
        """The first mission DRIVE went into the queue."""
        if self.state == READY:
            self.state = DRIVING
            self._note(now_ns, "driving")

    def tick(self, now_ns: int, transmit=None, arbiter_stop=None, collect=None) -> None:
        """Each loop cycle after the host's STATEs and ``host.tick``: ARMING -> READY on the host's ready, the
        authorization's loss latches, the control stop's confirmation and its DISARM run their course."""
        host = self.host
        if self.state == ARMING:
            if host.ready:
                self.state = READY
                self._note(now_ns, "ready")
            elif not host.arming and not host.authorized:
                self.latch(now_ns, "arm_failed", transmit, arbiter_stop, collect)
        elif self.state in (READY, DRIVING):
            if not host.authorized:
                self.latch(now_ns, "authorization_revoked", transmit, arbiter_stop, collect)
        elif self.state == STOP_PENDING:
            s = host.state
            if s is not None and self._seq_echoed(s) and (s.boot_id, s.arm_gen) == self._stop_boot_gen:
                self._note(now_ns, "control_stop_confirmed", seq=self._stop_seq)
                self._disarm(now_ns, "s10_stopped", transmit)
            elif now_ns - self._stop_since_ns > self.stop_ack_ns or not host.authorized:
                self._disarm(now_ns, "control_stop_unconfirmed", transmit)
        elif self.state == DISARMING:
            s = host.state
            if s is not None and s.mode == Mode.DISARMED and host.health(now_ns).fresh:
                self.state = LATCHED
                self._note(now_ns, "disarmed")

    def _seq_echoed(self, s) -> bool:
        # the 16-bit seq the MCU last received is this frame's or a later one (mod 2^16, within half the space)
        return ((int(s.last_rx_seq) - int(self._stop_seq)) & 0xFFFF) < 0x8000

    # --- stops ------------------------------------------------------------------------------------------------------
    def latch(self, now_ns: int, reason: str, transmit=None, arbiter_stop=None, collect=None) -> None:
        """Any latch reason (design 7): (1) unsent candidates and earlier requests dropped, (2) a DISARM queued and a
        transmit tried, (3) the arbiter STOPPED (``arbiter_stop(reason)``; the caller skips a second boundary in the
        same tick), (4) the running mission collected (``collect(reason)``). A latch while latched keeps the first
        reason."""
        first = self.state != LATCHED
        if first:
            self.reason = reason
        self.port.new_control_generation()  # (1) the queue and the intent go
        self.host.request_disarm(now_ns)  # (2)
        if transmit is not None:
            transmit()
        self.state = LATCHED
        self._stop_seq = None
        self._note(now_ns, "latched", reason, first=first)
        if arbiter_stop is not None:
            arbiter_stop(reason)  # (3)
        if collect is not None:
            collect(reason)  # (4): last -- its failure cannot undo (1)-(3)

    def control_stop(self, now_ns: int, seq: int) -> None:
        """S10's end (design 7): the caller made the stop frame (wheels 0, the measured lift) on a new generation and
        queued it as ``seq``; it is confirmed by a STATE of this boot and arm generation naming the seq within
        STOP_ACK_S, then DISARM. The normal recovery lock does not drop it."""
        s = self.host.state
        self._stop_seq, self._stop_since_ns = int(seq) & 0xFFFF, int(now_ns)
        self._stop_boot_gen = None if s is None else (s.boot_id, s.arm_gen)
        self.state = STOP_PENDING
        self._note(now_ns, "control_stop", seq=self._stop_seq)

    def _disarm(self, now_ns: int, reason: str, transmit=None) -> None:
        self.port.new_control_generation()
        self.host.request_disarm(now_ns)
        if transmit is not None:
            transmit()
        self.reason = self.reason or reason
        self.state = DISARMING
        self._note(now_ns, "disarm", reason)

    # --- rearm --------------------------------------------------------------------------------------------------
    def rearm_refusal(self, now_ns: int) -> str | None:
        """The real conditions a rearm passes before its ACK (design 7; the arbiter's ``rearm_refusal``): a fresh
        DISARMED with no faults on this boot, no earlier command that may still act -- a queued or written DRIVE of
        any kind not yet expired counts, since a zero-wheel lift is not tracked yet (4c) -- and a stop seen after."""
        if self.state not in (LATCHED, OFF):
            return f"permit_{self.state.lower()}"
        host = self.host
        s = host.state
        if s is None or not host.health(now_ns).fresh:
            return "state_stale"
        if s.mode != Mode.DISARMED:
            return f"mcu_{s.mode.name.lower()}"
        if int(s.fault_flags):
            return "mcu_faults"
        if self.port.older_nonzero_may_act(now_ns):
            return "old_command_may_act"
        traffic = host.drive_traffic()
        if traffic["pending_nonzero_expiries"] or any(w["expires_ns"] > now_ns for w in traffic["written"]):
            return "outstanding_unknown"
        if not self.stop_confirmed(now_ns):
            return "stop_unconfirmed"
        return None

    def rearmed(self, now_ns: int, control_gen: int) -> None:
        """The arbiter's ``on_rearmed``: the latch goes with the arbiter's STOPPED -> PAUSED, nowhere else."""
        if self.state == LATCHED:
            self.state = OFF
            self._note(now_ns, "rearmed", control_gen=int(control_gen))


# --- options (design 2026-11-19-real-assembly 3) ------------------------------------------------------------------
REAL_DT = 1 / 50
# the names the core reads with a default (getattr), each set explicitly on the real truck: on only what S10-A uses
FEATURES_OFF = {
    "near_tracking": None, "near_tracking_stop_at": None, "bottom_deck_check": "off", "pickup_prior": False,
    "occupied_dumps": False, "low_clearance_guard": "off", "escape_start_release": "off", "start_band_release": "off",
    "return_start_backoff_m": 0.0, "return_start_backoff_boxed": False, "restart_confirm_scans": 0,
    "near_end_stand": "off", "near_capture_lower": "off", "cusp_replan_retry": "off", "adoption_zone": None,
    "adoption_zone_check": "off", "return_stall_budget": None, "return_cusp_replans": 0, "permit_stop_replan": "off",
    "observe_start_backoff": "off", "near_recapture_gap_m": None, "near_observe_arrival": "off",
    "near_capture_refit": "off", "near_capture_redraw": "off", "near_capture_expected": "off",
    "far_capture_expected": "off", "inject_round1_degenerate": False, "exit_prefix_exempt": "off",
    "escape_presteer": "off", "docking_degenerate_continue": False, "d7_reapproach_hold": "off",
    "d7_docking": None, "dwells": None, "safety_stops": None, "near_inject": None, "repeat_captures": 0,
    "repeat_at_attempt": None, "extra_views": False, "goal_creep_band": "off",
}
REQUIRED_ON = {"real_vehicle": True, "grid_planning": True, "obstacle_act": True, "pocket_check": True,
               "s10_stop_at_lift": True}


def real_options(*, kinematics, planner_config, travel_config, scenario, speeds: dict, mission_stages, **extra):
    """The real truck's options, every one named (design 3): dt 1/50, the S10-A switches on, the rest off."""
    from types import SimpleNamespace

    return SimpleNamespace(dt=REAL_DT, kinematics=kinematics, planner_config=planner_config,
                           travel_config=travel_config, scenario=scenario, speeds=dict(speeds),
                           mission_stages=tuple(mission_stages), observation_waypoints=[], pickup_bounds=None,
                           observe_bounds=None, pickup_zone=None, return_to_pose=None, home_rear=None,
                           **REQUIRED_ON, **FEATURES_OFF, **extra)


def real_option_refusal(options, *, curvatures: dict, max_speed_mps: float) -> list[str]:
    """Why these options may not run on the real truck (empty: they may). ``curvatures``: the caps actually handed to
    the planner, the tracker and the preview, by name; they must agree and stay inside the steering map's symmetric
    range min(-kmin, kmax). ``max_speed_mps``: S10-A's approved rear-axle cap; every phase speed within it."""
    import math

    out = []
    if getattr(options, "dt", None) != REAL_DT:
        out.append("dt_not_1_50")
    for name, value in REQUIRED_ON.items():
        if getattr(options, name, None) is not value:
            out.append(f"{name}_not_{value}")
    if getattr(options, "near_tracking", None) is not None:
        out.append("near_tracking_on")
    if getattr(options, "bottom_deck_check", "off") != "off":
        out.append("bottom_deck_check_on")
    kin = getattr(options, "kinematics", None)
    if kin is None:
        out.append("kinematics_missing")
    else:
        low, high = kin.cfg.steering.curvature_range
        cap = min(-float(low), float(high))
        values = {k: float(v) for k, v in curvatures.items()}
        if len(set(values.values())) != 1:
            out.append("curvature_caps_differ:" + ",".join(f"{k}={v:g}" for k, v in sorted(values.items())))
        if any(v > cap + 1e-9 for v in values.values()):
            out.append(f"curvature_beyond_steering:{cap:g}")
    if not (isinstance(max_speed_mps, (int, float)) and math.isfinite(max_speed_mps) and max_speed_mps > 0):
        out.append("max_speed_invalid")
    else:
        for name, v in sorted((getattr(options, "speeds", None) or {}).items()):
            if not (math.isfinite(float(v)) and abs(float(v)) <= max_speed_mps + 1e-9):
                out.append(f"speed_over_cap:{name}")
        straight = getattr(options, "approach_straight_speed_mps", None)
        if straight is not None and abs(float(straight)) > max_speed_mps + 1e-9:
            out.append("speed_over_cap:approach_straight")
    return out


# --- S10-A settings (design 2) -------------------------------------------------------------------------------------
@dataclass(frozen=True)
class S10AConfig:
    map_id: str | None
    start_rear: tuple  # the taped start (map x, y, yaw at the rear axle)
    bounds: tuple  # the test field (x_min, x_max, y_min, y_max)
    pallet_id: str
    pallet_kind: str
    pallet_depth_m: float | None
    pallet_width_m: float | None
    destination_id: str
    destination: tuple  # the planning destination (S10 stops at the lift; the planner needs the whole mission)
    wait_pose: tuple
    max_speed_mps: float
    check_pallet_pose: tuple | None  # a reference pallet pose the dry assembly plans once to


def load_s10a(path) -> S10AConfig:
    import yaml

    with open(path) as f:
        d = yaml.safe_load(f)
    return S10AConfig(map_id=d.get("map_id"), start_rear=tuple(d["start_rear"]), bounds=tuple(d["bounds"]),
                      pallet_id=str(d["pallet"]["id"]), pallet_kind=str(d["pallet"]["kind"]),
                      pallet_depth_m=d["pallet"].get("depth_m"), pallet_width_m=d["pallet"].get("width_m"),
                      destination_id=str(d["destination"]["id"]), destination=tuple(d["destination"]["pose"]),
                      wait_pose=tuple(d["wait_pose"]), max_speed_mps=float(d["max_speed_mps"]),
                      check_pallet_pose=None if d.get("check_pallet_pose") is None else tuple(d["check_pallet_pose"]))


def s10a_refusal(cfg: S10AConfig, loaded_map_id: str | None) -> list[str]:
    """Structural problems of the S10-A settings (the dry assembly's exit 2)."""
    out = []
    if not cfg.map_id:
        out.append("map_id_unset")
    elif loaded_map_id != cfg.map_id:
        out.append(f"map_id_mismatch:{loaded_map_id}")
    for name in ("pallet_depth_m", "pallet_width_m"):
        if getattr(cfg, name) is None:
            out.append(f"{name}_unset")
    x0, x1, y0, y1 = (float(v) for v in cfg.bounds)
    if not (x0 < x1 and y0 < y1):
        out.append("bounds_invalid")
    for name, pose in (("start_rear", cfg.start_rear), ("destination", cfg.destination), ("wait_pose", cfg.wait_pose)):
        if not (x0 <= float(pose[0]) <= x1 and y0 <= float(pose[1]) <= y1):
            out.append(f"{name}_outside_bounds")
    return out


# --- readiness (design 6) ------------------------------------------------------------------------------------------
@dataclass
class Evidence:
    """What readiness reads, each a callable of now_ns supplied by the executor (4c wires them to new STATEs, the
    host's events, the SLAM link and the layer). ``*_reason`` callables return None when fine."""

    worker_ok: object
    layer_ok: object
    baseline_ok: object
    link_reason: object  # freshness, build and clock rate (the common MCU checks)
    measurement_reason: object  # steering, lift (finite with a valid origin), the counts' fault bit
    odometry_continuous: object
    slam_reason: object  # None: a fresh reply that computed a pose; the stage decides tracking or holding
    snapshot_reason: object  # None: installed for the applied correction, its newest scan fresh; "installing" waits
    operator_start_ok: object  # this session's, this lease's physical check for the next submit
    core_refusal: object  # real_auto_refusal on the core's safety state


def auto_ready_refusal(now_ns: int, stage: str, ev: Evidence) -> str | None:
    """The first unmet readiness condition (design 6), or None. ``stage``: "submit" (a submit and the adoption) or
    "drive" (arming and driving). A snapshot being installed after a correction change answers "installing"
    (the output closes, no latch)."""
    if stage not in ("submit", "drive"):
        raise ValueError(stage)
    checks = (
        ("worker", lambda: None if ev.worker_ok(now_ns) else "dead"),
        ("layer", lambda: None if ev.layer_ok(now_ns) else "missing"),
        ("baseline", lambda: None if ev.baseline_ok(now_ns) else "missing"),
        ("link", lambda: ev.link_reason(now_ns)),
        ("measurement", lambda: ev.measurement_reason(now_ns)),
        ("odometry", lambda: None if ev.odometry_continuous(now_ns) else "cut"),
        ("slam", lambda: ev.slam_reason(now_ns, stage)),
        ("snapshot", lambda: ev.snapshot_reason(now_ns)),
        ("core", lambda: ev.core_refusal(now_ns)),
        ("operator_start", lambda: None if stage == "drive" or ev.operator_start_ok(now_ns) else "unconfirmed"),
    )
    for name, check in checks:
        why = check()
        if why is not None:
            return f"{name}:{why}"
    return None


__all__ = ["ARMING", "AutoPermit", "Evidence", "FEATURES_OFF", "REAL_DT", "REQUIRED_ON", "S10AConfig", "auto_ready_refusal",
           "load_s10a", "real_option_refusal", "real_options", "s10a_refusal", "DISARMING", "DRIVING", "LATCHED", "OFF", "READY", "STOP_ACK_S", "STOP_PENDING"]
