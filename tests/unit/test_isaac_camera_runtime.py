"""CPU regressions for G1 OpenCV policy, annotator teardown and capture plan."""

import ast
import json
import runpy
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
VERIFY_SOURCE = ROOT / "sim/isaac/verify_perception_camera.py"
VERIFY = runpy.run_path(str(VERIFY_SOURCE))
MEASURE = VERIFY["MEASURE"]


class FakeOpenCL:
    def __init__(self, available, *, ignore_disable=False):
        self.available = available
        self.enabled = True
        self.ignore_disable = ignore_disable

    def setUseOpenCL(self, enabled):
        if not self.ignore_disable:
            self.enabled = enabled

    def haveOpenCL(self):
        return self.available

    def useOpenCL(self):
        return self.enabled


class FakeAnnotator:
    def __init__(self, *, attached=True, error=None, before_detach=lambda: None):
        self.is_attached = attached
        self.error = error
        self.before_detach = before_detach
        self.detach_calls = 0

    def attach(self, products):
        assert products == ["/Render/Original"]
        self.is_attached = True

    def detach(self, render_products=None):
        # The camera's current product may differ from the original attachment.
        assert render_products is None
        self.detach_calls += 1
        self.before_detach()
        if self.error:
            raise RuntimeError(self.error)
        if not self.is_attached:
            raise RuntimeError("Annotator rgb is not attached to any render products")
        self.is_attached = False


@pytest.mark.parametrize("available", [True, False])
def test_run_disables_opencl_and_persists_actual_state_before_measurement(
    monkeypatch, tmp_path, available
):
    ocl = FakeOpenCL(available)
    monkeypatch.setitem(
        sys.modules, "cv2", SimpleNamespace(ocl=ocl, __version__="fake")
    )
    expected = {
        "requested_use_opencl": False,
        "have_opencl": available,
        "use_opencl": False,
    }
    entered = []

    def measure(app, args, result, annotators):
        assert ocl.enabled is False, "OpenCL must be off before corner detection"
        saved = json.loads((args.output / "result.json").read_text())
        assert saved["opencv_opencl"] == expected
        assert saved["opencv_version"] == "fake"
        entered.append(True)

    monkeypatch.setitem(VERIFY["run"].__globals__, "_run_measurements", measure)
    result = {"status": "FAIL"}
    VERIFY["run"](None, SimpleNamespace(output=tmp_path), result)
    assert entered == [True]
    assert result["opencv_opencl"] == expected


def test_opencl_disable_readback_failure_is_recorded_and_stops_measurement(
    monkeypatch, tmp_path
):
    ocl = FakeOpenCL(True, ignore_disable=True)
    monkeypatch.setitem(
        sys.modules, "cv2", SimpleNamespace(ocl=ocl, __version__="fake")
    )

    def forbidden(*args):
        pytest.fail("measurement started with OpenCL enabled")

    monkeypatch.setitem(VERIFY["run"].__globals__, "_run_measurements", forbidden)
    with pytest.raises(RuntimeError, match="OpenCL"):
        VERIFY["run"](None, SimpleNamespace(output=tmp_path), {"status": "FAIL"})
    saved = json.loads((tmp_path / "result.json").read_text())
    assert saved["opencv_opencl"]["use_opencl"] is True


@pytest.mark.parametrize("measurement_fails", [False, True])
def test_run_saves_result_then_detaches_on_success_and_exception(
    monkeypatch, tmp_path, measurement_fails
):
    monkeypatch.setitem(
        sys.modules, "cv2", SimpleNamespace(ocl=FakeOpenCL(True), __version__="fake")
    )

    def check_saved():
        saved = json.loads((tmp_path / "result.json").read_text())
        assert saved["boards"] == [{"captured": True}]
        if not measurement_fails:
            assert saved["gates"] == {"1": "PASS"}

    rgb = FakeAnnotator(before_detach=check_saved)
    depth = FakeAnnotator(attached=False)

    def measure(app, args, result, annotators):
        annotators.update(rgb=rgb, z=depth)
        result["boards"] = [{"captured": True}]
        if measurement_fails:
            raise ValueError("original measurement failure")
        result.update(gates={"1": "PASS"}, status="REVIEW_REQUIRED")

    monkeypatch.setitem(VERIFY["run"].__globals__, "_run_measurements", measure)
    result = {"status": "FAIL"}
    if measurement_fails:
        with pytest.raises(ValueError, match="original measurement failure"):
            VERIFY["run"](None, SimpleNamespace(output=tmp_path), result)
    else:
        VERIFY["run"](None, SimpleNamespace(output=tmp_path), result)
        assert result["status"] == "REVIEW_REQUIRED"
    assert rgb.detach_calls == 1
    assert depth.detach_calls == 0
    saved = json.loads((tmp_path / "result.json").read_text())
    assert saved["annotator_cleanup"] == {
        "rgb": {"status": "detached"},
        "z": {"status": "already_detached"},
    }


def test_detach_skips_disconnected_and_consumes_ownership_for_repeat_cleanup():
    rgb, depth = FakeAnnotator(attached=False), FakeAnnotator()
    owned = {"rgb": rgb, "z": depth}
    report = MEASURE.detach_annotators(owned)
    assert report == {
        "rgb": {"status": "already_detached"},
        "z": {"status": "detached"},
    }
    assert MEASURE.detach_annotators(owned) == {}
    assert rgb.detach_calls == 0
    assert depth.detach_calls == 1
    assert owned == {}


def test_run_keeps_camera_alive_until_after_annotator_cleanup(monkeypatch, tmp_path):
    monkeypatch.setitem(
        sys.modules, "cv2", SimpleNamespace(ocl=FakeOpenCL(True), __version__="fake")
    )
    events = []
    rgb = FakeAnnotator(before_detach=lambda: events.append("detach"))

    class Camera:
        def __del__(self):
            events.append(("camera_released", rgb.is_attached))

    def measure(app, args, result, annotators):
        annotators["rgb"] = rgb
        return Camera()

    monkeypatch.setitem(VERIFY["run"].__globals__, "_run_measurements", measure)
    VERIFY["run"](None, SimpleNamespace(output=tmp_path), {"status": "FAIL"})
    assert events == ["detach", ("camera_released", False)]


@pytest.mark.parametrize(
    "error,status",
    [
        ("Annotator rgb is not attached to any render products", "already_detached"),
        ("unexpected graph failure", "error"),
    ],
)
def test_detach_records_errors_and_continues_other_channels(error, status):
    # The renderer may drop a connection after the is_attached readback.
    rgb, depth = FakeAnnotator(error=error), FakeAnnotator()
    owned = {"rgb": rgb, "z": depth}
    report = MEASURE.detach_annotators(owned)
    assert report["rgb"]["status"] == status
    assert error in report["rgb"]["error"]
    assert report["z"] == {"status": "detached"}
    assert depth.is_attached is False
    MEASURE.detach_annotators(owned)
    assert rgb.detach_calls == depth.detach_calls == 1


def _gate_complete_result(boards, fits):
    # Minimal result whose only variables are the two counted collections. The
    # panels carry the provenance a measured PASS requires, so the counts are
    # judged on a valid run rather than through the gate's fail-open path.
    return {
        "perception_camera_intrinsics": {"status": "PASS"},
        "mount": {"status": "PASS"},
        "gate_5": "PASS",
        "marker": [{"bright_centroid_outside_marker_bbox": True}],
        "intrinsics_fits": fits,
        "boards": boards,
        "height_panels": [
            {
                "status": "PASS",
                "panel_placement_K_reference": _placement_reference("PASS"),
                "selection_fit_reference": _reference(
                    "PASS", role="height_grid_sample_selection"
                ),
                "selection_coverage": {"status": "COMPLETE", "lost_bins": []},
                "captures": [
                    {
                        "status": "PASS",
                        "per_distance": [
                            {"distance_bin": index, "status": "PASS"}
                            for index in range(6)
                        ],
                    }
                ],
            }
            for _ in range(54)
        ],
    }


def test_capture_plan_puts_one_recorded_warmup_before_the_measured_repeats():
    # The first capture after authoring geometry is a measured ~0.12 px
    # transient, so it is taken and kept but never fitted or gated.
    plan = VERIFY["capture_plan"](MEASURE.REPEATS)
    assert [step["measured"] for step in plan] == [False] + [True] * MEASURE.REPEATS
    assert [step["repeat"] for step in plan] == [None, *range(MEASURE.REPEATS)]
    assert plan[0]["suffix"] == "warmup"
    assert [step["suffix"] for step in plan[1:]] == [
        f"repeat{index}" for index in range(MEASURE.REPEATS)
    ]
    assert plan[0]["exclusion_reason"] == VERIFY["WARMUP_EXCLUSION_REASON"]
    assert sum(step["measured"] for step in plan) == MEASURE.REPEATS


def test_warmup_capture_is_recorded_outside_the_gated_collections():
    measured, excluded = [], []
    for step in VERIFY["capture_plan"](MEASURE.REPEATS):
        VERIFY["file_capture_record"](
            {"capture": step["suffix"], "gate_4": "FAIL", "status": "FAIL"},
            step,
            measured,
            excluded,
        )
    assert len(measured) == MEASURE.REPEATS
    assert len(excluded) == 1
    assert excluded[0]["excluded_from_measurement"] is True
    assert excluded[0]["exclusion_reason"] == VERIFY["WARMUP_EXCLUSION_REASON"]
    # Discarding a measurement without recording it would be concealment.
    assert excluded[0]["gate_4"] == "FAIL"
    assert excluded[0]["status"] == "FAIL"
    assert all("excluded_from_measurement" not in record for record in measured)


def test_gate_counts_survive_the_recorded_warmup_captures():
    boards, warmup_boards = [], []
    for _ in range(len(MEASURE.DISTANCES_M) * len(MEASURE.SCREEN_CENTRES_UV)):
        for step in VERIFY["capture_plan"](MEASURE.REPEATS):
            VERIFY["file_capture_record"](
                {"gate_4": "PASS", "status": "PASS", "repeat": step["repeat"]},
                step,
                boards,
                warmup_boards,
            )
    assert len(boards) == 162
    assert len(warmup_boards) == 54
    result = _gate_complete_result(
        boards, [{"gate_2a": "PASS", "gate_2b": "PASS"} for _ in range(18)]
    )
    result["warmup_boards"] = warmup_boards
    VERIFY["finish_result"](result)
    assert result["gates"]["4"] == "PASS"
    assert result["gates"]["7a"] == "PASS"
    assert result["gates"]["2a"] == "PASS"
    assert result["numerical_status"] == "PASS"


def test_warmup_and_measured_captures_share_one_statistics_path():
    # A discarded capture must not get a second, weaker measurement path.
    # Both authored targets - board and height panel - repeat over the same
    # plan, and each statistic is computed exactly once, inside that loop.
    tree = ast.parse(VERIFY_SOURCE.read_text(encoding="utf-8"))
    plans = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.For)
        and isinstance(node.iter, ast.Call)
        and getattr(node.iter.func, "id", None) == "capture_plan"
    ]
    assert len(plans) == 2

    def call_sites(root, name):
        return sum(
            1
            for node in ast.walk(root)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == name
        )

    for statistic in ("board_statistics", "height_grid_statistics"):
        assert call_sites(tree, statistic) == 1
        assert sum(call_sites(plan, statistic) for plan in plans) == 1


def test_selection_fit_reference_names_a_measured_repeat_and_its_gate():
    # The selected repeat is now chosen by the predeclared rule rather than
    # fixed, but it is still one MEASURED repeat: a warm-up capture produces no
    # fit, so it is never a candidate and never nameable here.
    plan = VERIFY["capture_plan"](MEASURE.REPEATS)
    measured = [step["repeat"] for step in plan if step["measured"]]
    fits = [{"gate_2a": "FAIL"}, {"gate_2a": "PASS"}]
    reference = _reference("PASS", role="selection", gate_2a_first="FAIL")
    assert reference["repeat"] in measured
    assert reference["index"] == 1
    assert reference["repeat"] == 1
    assert reference["repeat_kind"] == "measured"
    assert reference["warmup_capture_excluded"] is True
    assert reference["gate_2a"] == fits[1]["gate_2a"] == "PASS"
    assert reference["anchor_horizontal_m"] == 0.8


def _reference(gate_2a, *, role, gate_2a_first=None, distance=0.8):
    """Build a fit reference the way the run does: choose, then name."""
    gates = [gate_2a] if gate_2a_first is None else [gate_2a_first, gate_2a]
    choice = VERIFY["choose_selection_fit"](
        [
            {"repeat": repeat, "index": repeat, "gate_2a": gate, "fit_available": True}
            for repeat, gate in enumerate(gates)
        ]
    )
    fits = [{"gate_2a": gate} for gate in gates]
    return VERIFY["selection_fit_reference"](fits, choice, distance=distance, role=role)


def _placement_reference(gate_2a):
    return _reference(gate_2a, role="height_panel_physical_placement")


def test_unvalidated_selection_fit_is_marked_instead_of_aborting_the_run():
    # Aborting would destroy the evidence the run exists to collect, so the
    # unvalidated fit is marked and gate 7a-prime carries it instead.
    assert "require_validated_selection_fit" not in VERIFY
    assert VERIFY["unvalidated_selection_fit_marks"](_placement_reference("PASS")) == {}
    marks = VERIFY["unvalidated_selection_fit_marks"](_placement_reference("FAIL"))
    assert marks["selection_fit_unvalidated"] is True
    assert (
        marks["selection_fit_unvalidated_reason"]
        == VERIFY["GATE_7A_PRIME_UNVALIDATED_FIT_REASON"]
    )
    detail = marks["selection_fit_unvalidated_detail"]
    # The named fit, and that its numbers survive as diagnostics.
    assert "intrinsics_fits[0]" in detail
    assert "0.8" in detail
    assert "repeat 0" in detail
    assert "gate_2a=FAIL" in detail
    assert "diagnostic" in detail


def test_selection_and_placement_uses_of_one_fit_stay_distinguishable():
    # The same fitted K both selects samples and physically places the panel.
    # A later K comparison must not move the panels without saying so.
    selection = _reference("PASS", role="height_grid_sample_selection")
    placement = _reference("PASS", role="height_panel_physical_placement")
    assert selection["role"] != placement["role"]
    assert {k: v for k, v in selection.items() if k != "role"} == {
        k: v for k, v in placement.items() if k != "role"
    }


def _height_geometry_calls(tree):
    return [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr in ("height_panel_geometry", "height_grid_selection")
    ]


def test_panel_geometry_failure_is_marked_instead_of_aborting_the_run():
    marks = VERIFY["panel_geometry_failure_marks"](
        ValueError("Height panel margin does not clear the selection erosion")
    )
    assert marks["status"] == "FAIL"
    assert marks["panel_geometry_failed"] is True
    assert (
        marks["panel_geometry_failed_reason"]
        == VERIFY["GATE_7A_PRIME_PANEL_GEOMETRY_REASON"]
    )
    detail = marks["panel_geometry_failed_detail"]
    assert "ValueError" in detail
    assert "does not clear the selection erosion" in detail


def test_one_panels_geometry_never_costs_the_other_panels_their_measurements():
    # The same posture the change set states for the unvalidated fit: the gate
    # table is the evidence the run exists to produce, so a geometry or margin
    # failure fails its own panel and is carried by gate 7a-prime instead.
    tree = ast.parse(VERIFY_SOURCE.read_text(encoding="utf-8"))
    calls = _height_geometry_calls(tree)
    assert len(calls) == 2
    guarded = {
        id(node)
        for handler in ast.walk(tree)
        if isinstance(handler, ast.Try)
        for node in _height_geometry_calls(handler)
    }
    assert guarded == {id(node) for node in calls}
    aborts = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "require"
        and any(
            isinstance(argument, ast.Constant)
            and "Composed panel differs" in str(argument.value)
            for argument in node.args
        )
    ]
    assert not aborts, "the composed-panel check must fail the panel, not the run"


def test_a_linear_algebra_failure_is_not_recorded_as_a_panel_geometry_failure():
    # numpy.linalg.LinAlgError subclasses ValueError, so a bare
    # `except ValueError` around the panel geometry would absorb it and copy a
    # failure of the shared mount into one panel record at a time. The handler
    # is narrowed at the call site: a linear-algebra failure is re-raised.
    tree = ast.parse(VERIFY_SOURCE.read_text(encoding="utf-8"))
    guards = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Try) and _height_geometry_calls(node)
    ]
    assert len(guards) == 2
    for guard in guards:
        handled = [ast.unparse(handler.type) for handler in guard.handlers]
        assert handled[0].endswith("LinAlgError"), handled
        assert "ValueError" in handled[1:], handled
        body = guard.handlers[0].body
        assert len(body) == 1
        assert isinstance(body[0], ast.Raise) and body[0].exc is None


@pytest.mark.parametrize(
    "states,expected",
    [
        ([], "FAIL"),
        (["PASS", "PASS", "PASS"], "PASS"),
        (["UNOBSERVED", "UNOBSERVED"], "UNOBSERVED"),
        (["PASS", "UNOBSERVED", "PASS"], "FAIL"),
        (["PASS", "FAIL", "PASS"], "FAIL"),
        (["FAIL"], "FAIL"),
        (["SOMETHING_ELSE"], "FAIL"),
    ],
)
def test_panel_status_never_promotes_a_panel_without_measured_captures(
    states, expected
):
    # all([]) is True: a panel with no measured capture would otherwise be
    # promoted to PASS by a vacuous quantifier.
    assert VERIFY["panel_status"](states) == expected


def test_the_height_panel_loop_judges_through_panel_status():
    # A helper the loop does not call pins nothing.
    tree = ast.parse(VERIFY_SOURCE.read_text(encoding="utf-8"))
    assigns = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and isinstance(node.value, ast.Call)
        and getattr(node.value.func, "id", None) == "panel_status"
        and any(
            isinstance(target, ast.Subscript)
            and isinstance(target.slice, ast.Constant)
            and target.slice.value == "status"
            for target in node.targets
        )
    ]
    assert len(assigns) == 1


def test_each_panel_owns_its_copy_of_the_two_fit_references():
    # One dict inserted by reference into all nine panels of an anchor would
    # let a later per-panel mutation corrupt nine records at once. The copy is
    # now a deep one: the reference carries nested rejected-repeat records.
    tree = ast.parse(VERIFY_SOURCE.read_text(encoding="utf-8"))
    placements = [
        value
        for node in ast.walk(tree)
        if isinstance(node, ast.Dict)
        for key, value in zip(node.keys, node.values, strict=True)
        if isinstance(key, ast.Constant) and key.value == "panel_placement_K_reference"
    ]
    assert len(placements) == 1
    assert isinstance(placements[0], ast.Call)
    assert getattr(placements[0].func, "id", None) == "panel_reference_copy"
    saves = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and getattr(node.func, "id", None) == "save_height_selection"
    ]
    assert len(saves) == 1
    reference_argument = saves[0].args[-1]
    assert isinstance(reference_argument, ast.Call)
    assert getattr(reference_argument.func, "id", None) == "panel_reference_copy"


def test_partial_attachment_remains_owned_for_finally_cleanup():
    rgb = FakeAnnotator(attached=False)

    def get_annotator(name, **kwargs):
        if name == "rgb":
            return rgb
        raise RuntimeError("cannot create segmentation annotator")

    rep = SimpleNamespace(
        AnnotatorRegistry=SimpleNamespace(get_annotator=get_annotator)
    )
    camera = SimpleNamespace(get_render_product_path=lambda: "/Render/Original")
    owned = {}
    with pytest.raises(RuntimeError, match="cannot create segmentation"):
        MEASURE.attach_annotators(rep, camera, annotators=owned)
    assert owned == {"rgb": rgb}
    MEASURE.detach_annotators(owned)
    assert rgb.is_attached is False
    assert rgb.detach_calls == 1


def _candidate(repeat, index, gate_2a, *, fit_available=True):
    """One measured repeat of one anchor, as the fitting loop records it."""
    return {
        "repeat": repeat,
        "index": index,
        "gate_2a": gate_2a,
        "fit_available": fit_available,
    }


def test_the_selection_repeat_is_chosen_by_a_predeclared_rule_not_a_constant():
    # USER decision, 2026-09-22: the K that selects the height-grid samples and
    # places the panel is taken from a fit that passed gate 2a, instead of a
    # fixed repeat 0 whose own gate 2a was never consulted.
    assert "SELECTION_FIT_REPEAT" not in VERIFY
    assert VERIFY["SELECTION_FIT_RULE"] == MEASURE.SELECTION_FIT_REPEAT_RULE
    protocol = MEASURE.protocol()
    assert protocol["selection_fit_repeat_rule"] == MEASURE.SELECTION_FIT_REPEAT_RULE
    policy = protocol["selection_fit_repeat_policy"]
    for phrase in ("gate 2a", "first measured repeat", "warm-up", "7a-prime"):
        assert phrase in policy


def test_selection_takes_the_first_measured_repeat_whose_gate_2a_passed():
    choice = VERIFY["choose_selection_fit"](
        [
            _candidate(0, 12, "FAIL"),
            _candidate(1, 13, "PASS"),
            _candidate(2, 14, "PASS"),
        ]
    )
    assert (choice["repeat"], choice["index"]) == (1, 13)
    assert choice["repeat_choice"] == "gate_2a_pass"
    assert choice["rule"] == MEASURE.SELECTION_FIT_REPEAT_RULE
    assert choice["rejected_repeats"] == [
        {
            "repeat": 0,
            "index": 12,
            "gate_2a": "FAIL",
            "rejected_because": "gate_2a_failed",
        }
    ]
    # Repeat 2 was never weighed: the rule stops at the first passing repeat.
    assert choice["unexamined_repeats_after_choice"] == [2]
    assert "gate 2a" in choice["chosen_because"]


def test_selection_keeps_the_first_measured_repeat_when_none_passed_gate_2a():
    # Unchanged behaviour in that case, which is what run 8 recorded: the panel
    # is still placed and sampled, and gate 7a-prime fails the unvalidated fit.
    choice = VERIFY["choose_selection_fit"](
        [_candidate(repeat, 12 + repeat, "FAIL") for repeat in range(3)]
    )
    assert (choice["repeat"], choice["index"]) == (0, 12)
    assert choice["repeat_choice"] == "fallback_no_repeat_passed_gate_2a"
    # The chosen fallback is never also listed among the rejected repeats.
    assert [entry["repeat"] for entry in choice["rejected_repeats"]] == [1, 2]
    assert choice["unexamined_repeats_after_choice"] == []
    reference = VERIFY["selection_fit_reference"](
        [{"gate_2a": "FAIL"} for _ in range(15)],
        choice,
        distance=4.0,
        role="height_panel_physical_placement",
    )
    assert reference["gate_2a"] == "FAIL"
    marks = VERIFY["unvalidated_selection_fit_marks"](reference)
    assert (
        marks["selection_fit_unvalidated_reason"]
        == VERIFY["GATE_7A_PRIME_UNVALIDATED_FIT_REASON"]
    )


def test_a_repeat_that_produced_no_fit_can_never_be_selected():
    choose = VERIFY["choose_selection_fit"]
    chosen = choose(
        [_candidate(0, 12, "FAIL", fit_available=False), _candidate(1, 13, "PASS")]
    )
    assert (chosen["repeat"], chosen["index"]) == (1, 13)
    assert chosen["rejected_repeats"][0]["rejected_because"] == "no_fit"
    # Nothing passed and the historical anchor produced no fit: the anchor has
    # no rendered K at all, exactly as before this rule existed.
    assert (
        choose(
            [
                _candidate(0, 12, "FAIL", fit_available=False),
                _candidate(1, 13, "FAIL"),
            ]
        )
        is None
    )
    assert choose([]) is None


def test_the_reference_records_the_rule_the_choice_and_the_rejected_repeats():
    fits = [{"gate_2a": "FAIL"}, {"gate_2a": "PASS"}, {"gate_2a": "PASS"}]
    choice = VERIFY["choose_selection_fit"](
        [_candidate(0, 0, "FAIL"), _candidate(1, 1, "PASS"), _candidate(2, 2, "PASS")]
    )
    common = {"distance": 0.8}
    selection = VERIFY["selection_fit_reference"](
        fits, choice, role="height_grid_sample_selection", **common
    )
    placement = VERIFY["selection_fit_reference"](
        fits, choice, role="height_panel_physical_placement", **common
    )
    # Both role tags survive the rule change, and still differ only by role.
    assert selection["role"] == "height_grid_sample_selection"
    assert placement["role"] == "height_panel_physical_placement"
    assert {k: v for k, v in selection.items() if k != "role"} == {
        k: v for k, v in placement.items() if k != "role"
    }
    # gate_2a is read from the fit itself; the chooser never asserts it.
    assert selection["gate_2a"] == fits[1]["gate_2a"] == "PASS"
    assert (selection["index"], selection["repeat"]) == (1, 1)
    assert selection["repeat_kind"] == "measured"
    assert selection["warmup_capture_excluded"] is True
    assert selection["selection_rule"] == MEASURE.SELECTION_FIT_REPEAT_RULE
    assert selection["repeat_choice"] == "gate_2a_pass"
    assert [entry["repeat"] for entry in selection["rejected_repeats"]] == [0]
    assert selection["unexamined_repeats_after_choice"] == [2]
    # A reader of one panel must see that the placement moves with the repeat.
    assert selection["panel_placement_repeat_dependent"] is True


def test_each_panel_owns_its_nested_choice_record():
    # dict() is a shallow copy: nine panels sharing one rejected_repeats list
    # would let a single panel's mutation corrupt nine records at once, which
    # is the defect the per-panel copy already guards for the flat fields.
    fits = [{"gate_2a": "FAIL"}, {"gate_2a": "PASS"}]
    choice = VERIFY["choose_selection_fit"](
        [_candidate(0, 0, "FAIL"), _candidate(1, 1, "PASS")]
    )
    reference = VERIFY["selection_fit_reference"](
        fits, choice, distance=0.8, role="height_panel_physical_placement"
    )
    first = VERIFY["panel_reference_copy"](reference)
    second = VERIFY["panel_reference_copy"](reference)
    first["rejected_repeats"][0]["rejected_because"] = "tampered"
    assert second["rejected_repeats"][0]["rejected_because"] == "gate_2a_failed"
    assert reference["rejected_repeats"][0]["rejected_because"] == "gate_2a_failed"
    source = VERIFY_SOURCE.read_text(encoding="utf-8")
    assert "dict(placement_reference)" not in source
    assert "dict(selection_reference)" not in source
    assert source.count("panel_reference_copy(") >= 3
