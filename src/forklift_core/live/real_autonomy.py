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

from forklift_core.mcu_link.messages import FaultFlag, Mode, RejectReason, StateFlag

OFF, ARMING, READY, DRIVING, STOP_PENDING, DISARMING, LATCHED = (
    "OFF", "ARMING", "READY", "DRIVING", "STOP_PENDING", "DISARMING", "LATCHED")
STOP_ACK_S = 0.100  # the control stop's frame confirmed by a STATE naming its seq within this (design 7, provisional)
STOP_QUEUE_S = 0.040  # and written within this of its queueing (4c design 4 4: the queue dwell)
DISARM_RETRY_S = 0.100  # an unanswered DISARM asked again (4c design 4 7)
DISARM_DEADLINE_S = 0.500  # then the arbiter STOPPED and the DISARM recorded unconfirmed (4c design 4 7)


@dataclass
class PermitEvent:
    t_ns: int
    event: str
    reason: str | None = None
    detail: dict = field(default_factory=dict)


class AutoPermit:
    """One AUTO permit for the real truck (design 7). ``host``: the ``McuLinkHost``; ``port``: the ``RealPlantPort``.

    Evidence the executor supplies (4c wires it to the transmitter and new STATEs):
    - ``outstanding(now_ns) -> ("settled", t_ns) | ("pending", None) | ("unknown", None)``: every earlier command,
      a zero-wheel lift and a frame still in the transmitter included, settled since ``t_ns`` (4b: until 4c tracks the
      whole transmit path, "unknown" refuses);
    - ``stop_since(now_ns) -> (t_ns, boot_id) | None``: the truck seen standing on new samples since ``t_ns``.
    Callbacks bound once (``bind``): ``transmit()``, ``arbiter_stop(reason)`` -- the arbiter STOPPED, skipping a
    second boundary in the same tick --, ``collect(reason)`` -- the running mission ended and collected."""

    def __init__(self, host, port, *, outstanding, stop_since, stop_ack_s: float = STOP_ACK_S, rng=None) -> None:
        self.host, self.port = host, port
        self.outstanding, self.stop_since = outstanding, stop_since
        self.stop_ack_ns = int(stop_ack_s * 1e9)
        self.state = OFF
        self.reason: str | None = None
        self.events: list[PermitEvent] = []
        self._transmit = self._arbiter_stop = self._collect = None
        self._collected = True  # nothing to collect before a mission runs
        self._stop: dict | None = None  # the control stop: seq, boot/arm generation, written time
        self._disarm_asked_ns: int | None = None
        self._disarm_written_ns: int | None = None  # the newest DISARM's write (the transmitter's ``written``)
        self._disarm_seq: int | None = None  # and its seq: the MCU records it in last_rx_seq when it processed it
        self._disarm_first_ns: int | None = None  # the unanswered DISARM episode's first ask (retries keep it)
        self.disarm_unconfirmed = False  # past DISARM_DEADLINE_S with no answer: no rearm, no recovery done
        self.session_started = False
        import random

        self._rng = rng or random.SystemRandom()
        self._session: dict | None = None  # the session check: the two DISARMs' seqs and the boot

    def bind(self, *, transmit, arbiter_stop, collect) -> None:
        self._transmit, self._arbiter_stop, self._collect = transmit, arbiter_stop, collect

    # --- what the rest reads ------------------------------------------------------------------------------------
    @property
    def armed(self) -> bool:
        """The watches' input (design 7; the arbiter drops ``armed`` on entering AUTO). After a latch it stays on
        until a DISARMED is seen -- with no STATE at all (a lost link) too: unknown is not disarmed."""
        if self.state in (ARMING, READY, DRIVING, STOP_PENDING, DISARMING):
            return True
        if self.state != LATCHED:
            return False
        if self._disarm_written_ns is None and self._disarm_asked_ns is not None:
            return True  # the latch's DISARM not written yet
        return not self._disarmed_after_write_any()

    def drive_allowed(self) -> bool:
        """A mission DRIVE may be submitted (and only then)."""
        return self.state in (READY, DRIVING) and self.host.ready

    def _note(self, now_ns: int, event: str, reason: str | None = None, **detail) -> None:
        self.events.append(PermitEvent(int(now_ns), event, reason, detail))

    # --- the session's start (design 13: always from the stopped side) -------------------------------------------
    def start_session(self, now_ns: int) -> None:
        """The executor's first act: a DISARM, whatever the last executor left. ``session_started`` only after the
        session check (Codex 4b round 5, the teleop bridge's): a fresh DISARMED naming exactly the first DISARM's
        seq, then a second DISARM written a random distance of 1024 to 64512 seqs on, answered exactly on the same
        boot -- an earlier executor's late STATEs cannot match both (about 2^-32)."""
        self.port.new_control_generation()
        self.host.request_disarm(now_ns)
        self._asked_disarm(now_ns)
        self._session = {"stage": "first", "first": None, "boot": None}
        self._try(self._transmit)
        self._note(now_ns, "session_disarm")

    # --- arming ---------------------------------------------------------------------------------------------------
    def adopted(self, now_ns: int, *, pre_arm_refusal: str | None) -> bool:
        """The core adopted the submitted mission's plan and the caller checked the lease and generation again: the
        control generation moves, the queue goes, and the MCU ARM is asked once. ``pre_arm_refusal``: the pre-ARM
        checks' first failure (design 6). Returns whether the ARM was asked; a refusal latches (the full order)."""
        self._collected = False  # a mission runs from here: any latch collects it, once
        if not self.session_started:
            pre_arm_refusal = pre_arm_refusal or "session_not_started"
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

    def mission_ended(self, now_ns: int, reason: str) -> None:
        """Any mission end other than S10's control stop (cancel, pause -- S10-A stops there --, failure, the first
        frame's wait running out) leaves the MCU authority through the common latch (design 13)."""
        if self.state in (ARMING, READY, DRIVING):
            self.latch(now_ns, f"mission_end:{reason}")
        else:
            self._collected = True  # ended without an AUTO authority: nothing left to collect here

    def tick(self, now_ns: int) -> None:
        """Each loop cycle after the host's STATEs and ``host.tick``."""
        host = self.host
        s, rx = host.state, host.state_rx_ns
        sess = self._session
        if self._disarm_first_ns is not None and self._disarmed_after_write(now_ns):
            self._disarm_first_ns = None  # answered (the episode ends; a later DISARM starts its own)
            if self.disarm_unconfirmed:
                self.disarm_unconfirmed = False  # a late answer clears the mark (4c design 8)
                self._note(now_ns, "disarm_confirmed_late")
        if not self.session_started and sess is not None and self._disarmed_after_write(now_ns):
            if sess["stage"] == "first":
                sess.update(stage="second", first=self._disarm_seq, boot=s.boot_id)
                host.skip_seq(self._rng.randint(1024, 64512))
                host.request_disarm(now_ns)
                self._asked_disarm(now_ns)
                self._try(self._transmit)
            elif sess["stage"] == "second" and s.boot_id == sess["boot"] \
                    and ((self._disarm_seq - sess["first"]) & 0xFFFF) >= 1024:
                self.session_started = True
                self._session = None
                self._note(now_ns, "session_ready")
        if self.state == ARMING:
            if host.ready:
                self.state = READY
                self._note(now_ns, "ready")
            elif not host.arming and not host.authorized:
                self.latch(now_ns, "arm_failed")
        elif self.state in (READY, DRIVING):
            if not host.authorized:
                self.latch(now_ns, "authorization_revoked")
        elif self.state == STOP_PENDING:
            self._stop_pending(now_ns, s, rx)
        elif self.state == DISARMING:
            if self._disarmed_after_write(now_ns):
                self.state = LATCHED
                self._note(now_ns, "disarmed")
        self._disarm_retry(now_ns)

    def on_state(self, state, rx_ns: int) -> None:
        """Every STATE the host accepted, in order (a batch read at once included): while S10's stop is pending, a
        reject of its boot and arm generation received after it was queued is kept (it DISARMs, whatever a later
        sample says), and the confirmation is taken from the STATE that shows it, not only the batch's last."""
        stop = self._stop
        if self.state != STOP_PENDING or stop is None or rx_ns <= stop["since_ns"] \
                or (state.boot_id, state.arm_gen) != stop["boot_gen"]:
            return
        if state.reject_reason != RejectReason.NONE:
            stop.setdefault("rejected", state.reject_reason.name.lower())
            return
        w = stop["written_ns"]
        if w is not None and rx_ns > w and int(state.last_rx_seq) == stop["seq"] \
                and state.flags & StateFlag.COMMAND_FRESH and rx_ns - w <= self.stop_ack_ns:
            stop.setdefault("confirmed_rx", int(rx_ns))

    def _stop_pending(self, now_ns: int, s, rx) -> None:
        """S10's control stop (4c design 4 3-4), judged conservatively: written within STOP_QUEUE_S of its queueing;
        then a fresh STATE of the same boot and arm generation received after the write -- any reject reason there
        DISARMs at once (seq or not); naming exactly the stop's seq with COMMAND_FRESH within STOP_ACK_S of the write
        confirms it; past that, or the authority gone, DISARM."""
        host, stop = self.host, self._stop
        w = stop["written_ns"]
        # every failure DISARMs at once; the order only names the first that holds
        if stop.get("rejected") is not None:  # any reject since it was queued, before or after its write
            self._disarm(now_ns, f"control_stop_rejected:{stop['rejected']}")
            return
        queue_ns = int(STOP_QUEUE_S * 1e9)
        if w is not None and w - stop["since_ns"] > queue_ns:  # written after its queue dwell: no confirmation counts
            self._disarm(now_ns, "control_stop_late_write")
            return
        if not host.authorized:
            self._disarm(now_ns, "control_stop_unauthorized")
            return
        if w is None:
            if now_ns - stop["since_ns"] > queue_ns:
                self._disarm(now_ns, "control_stop_unwritten")
            return
        if stop.get("confirmed_rx") is not None and host.health(now_ns).fresh:
            self._note(now_ns, "control_stop_confirmed", seq=stop["seq"])
            self._disarm(now_ns, "s10_stopped")
            return
        if now_ns - w > self.stop_ack_ns:
            self._disarm(now_ns, "control_stop_unconfirmed")

    def stop_discarded(self, now_ns: int, seq: int) -> None:
        """The sender dropped a frame before its first byte: S10's stop frame among them DISARMs at once."""
        if self.state == STOP_PENDING and self._stop is not None and (int(seq) & 0xFFFF) == self._stop["seq"]:
            self._disarm(now_ns, "control_stop_discarded")

    def link_reset(self, now_ns: int) -> None:
        """The link was closed and reopened (a stuck write abandoned): an unanswered DISARM is asked again at once on
        the new link -- its episode's first ask, and so its deadline, kept."""
        if self._disarm_first_ns is not None and not self._disarmed_after_write(now_ns):
            self.host.request_disarm(now_ns)
            self._asked_disarm(now_ns)
            self._note(now_ns, "disarm_relink")

    def _asked_disarm(self, now_ns: int) -> None:
        """A DISARM was just queued: its write and seq are awaited afresh; the episode's first ask is kept."""
        self._disarm_asked_ns, self._disarm_written_ns, self._disarm_seq = int(now_ns), None, None
        if self._disarm_first_ns is None:
            self._disarm_first_ns = int(now_ns)

    def _disarm_retry(self, now_ns: int) -> None:
        """An unanswered DISARM (4c design 4 7): asked again every DISARM_RETRY_S (the tick's send writes it); past
        DISARM_DEADLINE_S from the first ask the arbiter goes STOPPED and the DISARM is marked unconfirmed -- once, not
        a new latch each tick. A retry changes the awaited seq only."""
        first = self._disarm_first_ns
        if first is None or self._disarmed_after_write(now_ns):
            return
        if now_ns - self._disarm_asked_ns >= int(DISARM_RETRY_S * 1e9):
            self.host.request_disarm(now_ns)
            self._asked_disarm(now_ns)
            self._note(now_ns, "disarm_retry")
        if not self.disarm_unconfirmed and now_ns - first >= int(DISARM_DEADLINE_S * 1e9):
            self.disarm_unconfirmed = True
            self._note(now_ns, "disarm_unconfirmed")
            self._try(self._arbiter_stop, "disarm_unconfirmed")

    # --- stops ------------------------------------------------------------------------------------------------------
    @staticmethod
    def _try(fn, *args):
        if fn is None:
            return None
        try:
            fn(*args)
        except Exception as exc:  # noqa: BLE001 -- the next step still runs; the caller sees it after
            return exc
        return None

    def latch(self, now_ns: int, reason: str) -> None:
        """Any latch reason (design 7). The permit closes first, then (1) the queue and the intent dropped, (2) a
        DISARM queued and a transmit tried, (3) the arbiter STOPPED, (4) the running mission collected -- each runs
        whatever the one before raised, and the first error is raised after all four. A latch while latched retries
        the DISARM only; the mission is collected once."""
        first = self.state != LATCHED
        if first:
            self.reason = reason
        self.state = LATCHED
        self._stop = None
        self._note(now_ns, "latched", reason, first=first)
        errors = [self._try(self.port.new_control_generation), self._try(self.host.request_disarm, now_ns)]
        self._asked_disarm(now_ns)
        errors.append(self._try(self._transmit))
        if first:
            errors.append(self._try(self._arbiter_stop, reason))
        if not self._collected:
            self._collected = True
            errors.append(self._try(self._collect, reason))
        error = next((e for e in errors if e is not None), None)
        if error is not None:
            raise error

    def control_stop(self, now_ns: int, seq: int) -> None:
        """S10's end (design 7): the caller made the stop frame (wheels 0, the measured lift) on a new generation and
        queued it as ``seq``; ``stop_written(seq, written_ns)`` tells when the transmitter finished writing it. It is
        confirmed by a STATE received after that write, naming exactly that u16 seq, of the same boot and arm
        generation, within STOP_ACK_S of the write -- then DISARM. The normal recovery lock does not drop it."""
        s = self.host.state
        self._stop = {"seq": int(seq) & 0xFFFF, "since_ns": int(now_ns), "written_ns": None,
                      "boot_gen": None if s is None else (s.boot_id, s.arm_gen)}
        self._collected = True  # S10's end is collected by the core's own terminate (no common latch)
        self.state = STOP_PENDING
        self._note(now_ns, "control_stop", seq=self._stop["seq"])

    def written(self, kind: str, seq: int, written_ns: int) -> None:
        """The transmitter wrote a whole frame (every frame, in order): a DISARM's write time, the stop frame's."""
        if kind == "DISARM":
            self._disarm_written_ns, self._disarm_seq = int(written_ns), int(seq) & 0xFFFF
        elif self._stop is not None and (int(seq) & 0xFFFF) == self._stop["seq"]:
            self._stop["written_ns"] = int(written_ns)

    def stop_written(self, seq: int, written_ns: int) -> None:
        self.written("DRIVE", seq, written_ns)

    def _disarmed_after_write_any(self) -> bool:
        """A DISARMED received after the newest DISARM's write that names exactly that DISARM's seq (the MCU records the
        seq of the frame it processed: a sample made before it -- this executor's or an earlier one's, whatever its
        seq -- does not; nothing else is sent while a DISARM is awaited, and a retry renews the seq awaited)."""
        host, w, q = self.host, self._disarm_written_ns, self._disarm_seq
        s, rx = host.state, host.state_rx_ns
        return (w is not None and q is not None and s is not None and rx is not None and rx > w
                and s.mode == Mode.DISARMED and (int(s.last_rx_seq) & 0xFFFF) == q)

    def _disarmed_after_write(self, now_ns: int) -> bool:
        """A fresh DISARMED received after the newest DISARM was written (a cached one, or one before the write,
        is not the answer)."""
        return self._disarmed_after_write_any() and self.host.health(now_ns).fresh

    def _disarm(self, now_ns: int, reason: str) -> None:
        self.port.new_control_generation()
        self.host.request_disarm(now_ns)
        self._asked_disarm(now_ns)
        self._try(self._transmit)
        self.reason = self.reason or reason
        self.state = DISARMING
        self._stop = None
        self._note(now_ns, "disarm", reason)

    # --- rearm --------------------------------------------------------------------------------------------------
    def rearm_refusal(self, now_ns: int) -> str | None:
        """The real conditions a rearm passes before its ACK (design 7; the arbiter's ``rearm_refusal``): a fresh
        DISARMED with no faults, every earlier command settled (unknown refuses), and a stop seen on this boot since
        they settled."""
        if self.state not in (LATCHED, OFF):
            return f"permit_{self.state.lower()}"
        if self.disarm_unconfirmed or self._disarm_first_ns is not None:
            return "disarm_unconfirmed"  # until a DISARMED names the newest DISARM (4c design 4 7)
        host = self.host
        s = host.state
        if s is None or not host.health(now_ns).fresh:
            return "state_stale"
        if s.mode != Mode.DISARMED:
            return f"mcu_{s.mode.name.lower()}"
        if int(s.fault_flags) & ~int(FaultFlag.COMMAND_TIMEOUT):
            return "mcu_faults"  # a command's expiry (cleared by the next ARM) is no fault for a rearm (4c design 8)
        if self.port.older_nonzero_may_act(now_ns):
            return "old_command_may_act"
        kind, settled_ns = self.outstanding(now_ns)
        if kind != "settled":
            return f"outstanding_{kind}"
        stop = self.stop_since(now_ns)
        if stop is None:
            return "stop_unconfirmed"
        if stop[1] != s.boot_id or stop[0] < settled_ns:
            return "stop_before_settled"
        return None

    def rearmed(self, now_ns: int, control_gen: int) -> None:
        """The arbiter's ``on_rearmed``: the latch goes with the arbiter's STOPPED -> PAUSED, nowhere else; the old
        reason stays in the events only."""
        if self.state == LATCHED:
            self.state = OFF
            self._note(now_ns, "rearmed", self.reason, control_gen=int(control_gen))
            self.reason = None


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
S10A_STAGES = ("approach", "insert", "extract", "transport", "withdraw", "return_home")  # what plan_transport plans
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


def real_option_refusal(options, *, tracker_curvature_inv_m, preview_curvature_inv_m, max_speed_mps: float) -> list[str]:
    """Why these options may not run on the real truck (empty: they may). The curvature caps are the ones actually
    handed over -- the planner's and the travel planner's ``curvature_limit_inv_m`` read from their configs, the
    tracker's setting and the preview's -- and must be positive, equal and inside the steering map's symmetric range
    min(-kmin, kmax). ``max_speed_mps``: S10-A's approved rear-axle cap; every phase speed finite and within it."""
    import math

    def finite_pos(v):
        return isinstance(v, (int, float)) and math.isfinite(v) and v > 0

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
    caps = {"planner": getattr(getattr(options, "planner_config", None), "curvature_limit_inv_m", None),
            "travel": getattr(getattr(options, "travel_config", None), "curvature_limit_inv_m", None),
            "tracker": tracker_curvature_inv_m, "preview": preview_curvature_inv_m}
    for name, v in sorted(caps.items()):
        if not finite_pos(v):
            out.append(f"curvature_invalid:{name}")
    if kin is None:
        out.append("kinematics_missing")
    elif all(finite_pos(v) for v in caps.values()):
        low, high = kin.cfg.steering.curvature_range
        cap = min(-float(low), float(high))
        if len({float(v) for v in caps.values()}) != 1:
            out.append("curvature_caps_differ:" + ",".join(f"{k}={float(v):g}" for k, v in sorted(caps.items())))
        if any(float(v) > cap + 1e-9 for v in caps.values()):
            out.append(f"curvature_beyond_steering:{cap:g}")
    if not finite_pos(max_speed_mps):
        out.append("max_speed_invalid")
    else:
        speeds = dict(getattr(options, "speeds", None) or {})
        if tuple(getattr(options, "mission_stages", ()) or ()) != S10A_STAGES:
            out.append("mission_stages_not_s10a")
        for stage in S10A_STAGES:
            if stage not in speeds:
                out.append(f"speed_missing:{stage}")
        straight = getattr(options, "approach_straight_speed_mps", None)
        if straight is not None:
            speeds["approach_straight"] = straight
        for name, v in sorted(speeds.items()):
            if not (isinstance(v, (int, float)) and math.isfinite(v) and 0 < abs(float(v)) <= max_speed_mps + 1e-9):
                out.append(f"speed_over_cap:{name}")
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
SLAM_MAX_AGE_S = 0.5  # a reply that computed a pose, at most this old (provisional, PR 7 measures the link)
HOLD_MAX_M = 1.5  # the travel a mission's hold may cover before it is no longer trusted (provisional)
SCAN_MAX_AGE_S = 0.35  # the installed snapshot's newest scan (the real layer's free_max_age_s)


@dataclass(frozen=True)
class SlamView:
    mode: str  # tracking | holding | warming_up | lost ...
    pose_reply_ns: int | None  # the newest reply that computed a new pose (a "skipped" reply does not count)
    hold_travel_m: float = 0.0
    hold_mission_gen: int | None = None  # the mission generation whose transition began the hold


@dataclass(frozen=True)
class SnapshotView:
    installed_correction: tuple | None  # the correction the installed layer snapshot was made for
    applied_correction: tuple  # the correction applied now
    newest_scan_ns: int | None  # the installed result's newest scan acquisition (not the snapshot's re-projection)
    installing: bool = False  # a new result asked after a correction change, not yet installed
    installed_generation: int | None = None  # the correction generations (a release counts one): equal values of
    applied_generation: int | None = None  # two generations are two corrections


@dataclass(frozen=True)
class MeasurementView:
    steering_ok: bool
    lift_ok: bool
    lift_origin_ok: bool  # the lift reading has a valid origin and range, not just a finite value
    counts_ok: bool  # the wheel counts' fault bit clear


@dataclass
class Evidence:
    """What readiness reads (design 6), each a callable of now_ns the executor supplies (4c wires them to new
    STATEs, the host's events, the SLAM link and the layer); every time is on the host's monotonic clock."""

    worker_ok: object
    layer_ok: object
    baseline_ok: object
    link_reason: object  # None, or why the link is not fit: freshness, build, clock rate (the common MCU checks)
    measurement: object  # -> MeasurementView
    odometry_continuous: object
    slam: object  # -> SlamView
    snapshot: object  # -> SnapshotView
    operator_start_ok: object  # this session's, this lease's physical check for the next submit
    core_refusal: object  # real_auto_refusal on the core's safety state
    mission_gen: object = None  # -> the running mission's generation (a hold must be this mission's)


def auto_ready_refusal(now_ns: int, stage: str, ev: Evidence) -> str | None:
    """The first unmet readiness condition (design 6), or None. ``stage``: "submit" (a submit and the adoption) or
    "drive" (arming and driving). Every latch reason comes before a wait: a reason starting "wait:" (a snapshot
    being installed after a correction change) closes the output without a latch (``latches``)."""
    if stage not in ("submit", "drive"):
        raise ValueError(stage)
    if not ev.worker_ok(now_ns):
        return "worker:dead"
    if not ev.layer_ok(now_ns):
        return "layer:missing"
    if not ev.baseline_ok(now_ns):
        return "baseline:missing"
    why = ev.core_refusal(now_ns)
    if why is not None:
        return f"core:{why}"
    why = ev.link_reason(now_ns)
    if why is not None:
        return f"link:{why}"
    m = ev.measurement(now_ns)
    for name in ("steering_ok", "lift_ok", "lift_origin_ok", "counts_ok"):
        if not getattr(m, name):
            return f"measurement:{name[:-3]}"
    if not ev.odometry_continuous(now_ns):
        return "odometry:cut"
    sl = ev.slam(now_ns)
    allowed = ("tracking",) if stage == "submit" else ("tracking", "holding")
    if sl.mode not in allowed:
        return f"slam:mode_{sl.mode}"
    if sl.pose_reply_ns is None or now_ns - sl.pose_reply_ns > SLAM_MAX_AGE_S * 1e9:
        return "slam:stale"
    if sl.mode == "holding":
        if ev.mission_gen is None or sl.hold_mission_gen is None or sl.hold_mission_gen != ev.mission_gen(now_ns):
            return "slam:hold_not_this_missions"
        if sl.hold_travel_m > HOLD_MAX_M:
            return "slam:hold_too_long"
    sn = ev.snapshot(now_ns)
    if sn.newest_scan_ns is None or now_ns - sn.newest_scan_ns > SCAN_MAX_AGE_S * 1e9:
        return "snapshot:scan_stale"
    if stage == "submit" and not ev.operator_start_ok(now_ns):
        return "operator_start:unconfirmed"
    if sn.installing or sn.installed_correction is None \
            or tuple(sn.installed_correction) != tuple(sn.applied_correction) \
            or sn.installed_generation is None or sn.installed_generation != sn.applied_generation:
        return "wait:snapshot_installing"
    return None


def latches(reason: str | None) -> bool:
    """A readiness refusal that latches the AUTO permit (a "wait:" one only closes the output)."""
    return reason is not None and not reason.startswith("wait:")


__all__ = ["S10A_STAGES", "ARMING", "AutoPermit", "Evidence", "MeasurementView", "SlamView", "SnapshotView", "latches", "FEATURES_OFF", "REAL_DT", "REQUIRED_ON", "S10AConfig", "auto_ready_refusal",
           "load_s10a", "real_option_refusal", "real_options", "s10a_refusal", "DISARMING", "DRIVING", "LATCHED", "OFF", "READY", "STOP_ACK_S", "STOP_PENDING"]
