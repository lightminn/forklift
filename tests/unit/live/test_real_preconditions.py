"""The operating unlock U1 (docs/plans/2026-11-26-real-operating-assembly.md 2.4, 2.10, 3): S10's operating
preconditions -- refused on today's settings, every item bound to what it was measured on so a later change voids it,
and, with everything filled in, only ``precondition:unlock_incomplete`` left until the unlock's last PR."""

import copy
import shutil
from pathlib import Path

import pytest
import yaml

from forklift_core.live import real_preconditions as rp

REPO = Path(__file__).resolve().parents[3]
REAL = REPO / "config" / "real"
SERIAL = "0123456789ABCDEF01234567"
PRODUCT = "forklift-mcu normal open v1.2-3-gabcdef0 waiver=v1"
BOARD = {"usb_product": PRODUCT, "usb_serial": SERIAL, "state_seen": True, "drive_refusal": None, "boot_id": 7}


def _measure_all(node):
    if isinstance(node, dict):
        for k, v in node.items():
            if k == "provenance" and isinstance(v, dict):
                v["measured"] = True
            else:
                _measure_all(v)
    elif isinstance(node, list):
        for v in node:
            _measure_all(v)
    return node


class Site:
    """A temporary repository copy with every precondition met (``record`` built from the facts), so each test changes
    one thing and sees exactly its reason."""

    def __init__(self, root: Path):
        self.root = root
        cfg = root / "config" / "real"
        cfg.mkdir(parents=True)
        (root / "docs").mkdir()
        (root / "docs" / "evidence.md").write_text("evidence\n")
        (root / "docs" / "bound.md").write_text("bound\n")
        for name in ("autonomy.yaml", "s10a.yaml", "imu.yaml", "obstacle_layer_real.yaml"):
            shutil.copy(REAL / name, cfg / name)
        layer = yaml.safe_load((REAL / "obstacle_layer_real.yaml").read_text())
        odo = REPO / layer["odometry_age"]
        (root / layer["odometry_age"]).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(odo, root / layer["odometry_age"])
        for name in ("vehicle.yaml", "obstacle_layer.yaml", "safety_watch.yaml"):
            d = _measure_all(yaml.safe_load((REAL / name).read_text()))
            if name == "vehicle.yaml" and "chassis_model" in d:
                d["chassis_model"] = str((REAL / d["chassis_model"]).resolve())
            (cfg / name).write_text(yaml.safe_dump(d, sort_keys=False))
        lidar = yaml.safe_load((REAL / "lidar.yaml").read_text())
        lidar["timing"].update(first_beam_err_s=0.002, basis="engineering", decision_id="D-1", basis_doc="docs/bound.md",
                               applies_to={"image_id": "x"})
        lidar["mount"]["measured"] = True
        (cfg / "lidar.yaml").write_text(yaml.safe_dump(lidar, sort_keys=False))
        self.record_path = cfg / "s10_preconditions.yaml"
        self.write_record(self.full_record())

    def paths(self):
        cfg = self.root / "config" / "real"
        return dict(record_path=self.record_path, lidar_path=cfg / "lidar.yaml", imu_path=cfg / "imu.yaml",
                    settings_path=cfg / "autonomy.yaml", s10a_path=cfg / "s10a.yaml",
                    layer_path=cfg / "obstacle_layer_real.yaml", budget_path=cfg / "obstacle_layer.yaml",
                    vehicle_path=cfg / "vehicle.yaml", watch_path=cfg / "safety_watch.yaml", repo=self.root)

    def facts(self, board=BOARD):
        return rp.gather(board=board, **self.paths())

    def write_record(self, rec):
        self.record_path.write_text(yaml.safe_dump(rec, sort_keys=False))

    def full_record(self):
        self.write_record({"schema": rp.SCHEMA, "items": {}, "config_exceptions": []})
        f = self.facts()
        stop = dict(f.stop_files)
        common = {"status": "pass", "evidence": "docs/evidence.md", "date": "2026-12-01"}
        binds = {
            "s5_safety": {"board_serial": SERIAL, "product": PRODUCT, "stop_files": stop},
            "iwdg_evidence": {"commit": "abcdef0", "board_serial": SERIAL},
            "fault_detection": {"board_serial": SERIAL, "product": PRODUCT},
            "port_review": {},
            "scan_load": {"lidar_sha256": f.lidar_sha256, "scan_image": "img"},
            "scan_coverage": {"lidar_sha256": f.lidar_sha256},
            "s10_speed": {"approved_max_speed_mps": 0.15, "tested_speed_mps": 0.15},
            "start_taped": {"start_rear": list(f.s10a.start_rear)},
            "lift": {"board_serial": SERIAL, "lift_target_m": f.settings["lift_target_m"],
                     "lift_rate_mps": f.settings["lift_rate_mps"], "test_pallet": f.s10a.pallet_id,
                     "lift_mechanism": "lead screw, docs/evidence.md"},
            "firmware_build": {"product": PRODUCT, "firmware_vehicle_calibration_id": f.vehicle_calibration_id,
                               "readback_sha256": "0" * 64, "board_lift_rate_max_mps": 0.10},
            "stop_model": {"stop_files": stop},
            "jetson_load": {"stop_files": stop},
        }
        items = {name: {**common, "binds": b} for name, b in binds.items()}
        target = float(f.settings["lift_target_m"])
        items["lift"].update(
            origin={"state_lift_m_at_floor": 0.0, "tape_cm_at_target": target * 100,
                    "state_lift_m_at_target": target},
            hold=[{"raised_cm": target * 40, "sag_cm_after_60s": 0.1, "worst": True, "worst_reason": "chain slack"},
                  {"raised_cm": target * 100, "sag_cm_after_60s": 0.2}],
            sag_limit_cm=1.0, sag_limit_decision_id="D-2")
        return {"schema": rp.SCHEMA, "items": items, "config_exceptions": []}

    def change_record(self, fn):
        rec = self.full_record()
        fn(rec)
        self.write_record(rec)
        return rp.operating_preconditions(self.facts())


@pytest.fixture
def site(tmp_path):
    return Site(tmp_path)


def test_todays_settings_are_refused_with_their_reasons():
    f = rp.gather(lidar_path=REAL / "lidar.yaml", imu_path=REAL / "imu.yaml", settings_path=REAL / "autonomy.yaml",
                  s10a_path=REAL / "s10a.yaml", layer_path=REAL / "obstacle_layer_real.yaml",
                  budget_path=REAL / "obstacle_layer.yaml", vehicle_path=REAL / "vehicle.yaml",
                  watch_path=REAL / "safety_watch.yaml")
    out = rp.operating_preconditions(f)
    assert out[0] == rp.UNLOCK_INCOMPLETE
    for reason in ("precondition:C1:board:no_state", "precondition:C5:lidar:first_beam_err_s",
                   "precondition:C5:lidar:mount_unmeasured", "precondition:C2:s5_safety:unset",
                   "precondition:C11:lift:unset", "precondition:C12:vehicle:wheel_radius:unmeasured",
                   "precondition:C13:watch:current:unmeasured"):
        assert reason in out
    # the teleop bridge's own values are not S10's
    assert not any("teleop_speed_limit" in r or ":watch:tilt:" in r for r in out)


def test_everything_met_leaves_only_the_unlock_lock(site):
    assert rp.operating_preconditions(site.facts()) == [rp.UNLOCK_INCOMPLETE]


@pytest.mark.parametrize("board,reason", [
    ({**BOARD, "state_seen": False}, "precondition:C1:board:no_state"),
    ({**BOARD, "drive_refusal": "outputs_locked"}, "precondition:C1:board:outputs_locked"),
    ({**BOARD, "usb_product": None}, "precondition:C1:board:no_usb_product"),
    ({**BOARD, "usb_product": PRODUCT + "-dirty"}, "precondition:C1:board:dirty_image"),
    ({**BOARD, "usb_serial": None}, "precondition:C1:board:no_usb_serial"),
])
def test_board_facts(site, board, reason):
    assert reason in rp.operating_preconditions(site.facts(board))


def test_another_board_or_image_voids_the_bound_items(site):
    other = rp.operating_preconditions(site.facts({**BOARD, "usb_serial": "F" * 24}))
    for r in ("precondition:C2:s5_safety:board_serial", "precondition:C2:iwdg_evidence:board_serial",
              "precondition:C3:fault_detection:board_serial", "precondition:C11:lift:board_serial"):
        assert r in other
    image = rp.operating_preconditions(site.facts({**BOARD, "usb_product": PRODUCT.replace("abcdef0", "1234567")}))
    for r in ("precondition:C2:s5_safety:product", "precondition:C12:firmware_build:product"):
        assert r in image


def test_a_changed_stop_file_voids_the_stop_items(site):
    budget = site.root / "config/real/obstacle_layer.yaml"
    budget.write_text(budget.read_text() + "\n# changed\n")
    out = rp.operating_preconditions(site.facts())
    for r in ("precondition:C2:s5_safety:stop_files", "precondition:C13:stop_model:stop_files",
              "precondition:C14:jetson_load:stop_files"):
        assert r in out


def test_a_changed_lidar_file_voids_the_scan_items(site):
    lidar = site.root / "config/real/lidar.yaml"
    lidar.write_text(lidar.read_text() + "\n# changed\n")
    out = rp.operating_preconditions(site.facts())
    assert "precondition:C5:scan_load:lidar_sha256" in out and "precondition:C5:scan_coverage:lidar_sha256" in out


def test_lift_and_pallet_settings_are_bound(site):
    auto = site.root / "config/real/autonomy.yaml"
    d = yaml.safe_load(auto.read_text())
    d["lift_target_m"] = 0.25
    auto.write_text(yaml.safe_dump(d, sort_keys=False))
    out = rp.operating_preconditions(site.facts())
    assert "precondition:C11:lift:lift_target_m" in out


@pytest.mark.parametrize("change,reason", [
    (lambda r: r["items"].update(port_review=None), "precondition:C4:port_review:unset"),
    (lambda r: r["items"]["port_review"].update(status="approved_limited"), "precondition:C4:port_review:decision_id"),
    (lambda r: r["items"]["port_review"].update(status="maybe"), "precondition:C4:port_review:status"),
    (lambda r: r["items"]["s5_safety"].update(evidence="docs/missing.md"), "precondition:C2:s5_safety:evidence"),
    (lambda r: r["items"]["s5_safety"].update(evidence="/etc/passwd"), "precondition:C2:s5_safety:evidence"),
    (lambda r: r["items"]["s5_safety"].update(evidence="../outside.md"), "precondition:C2:s5_safety:evidence"),
    (lambda r: r["items"]["s5_safety"].update(date=None), "precondition:C2:s5_safety:date"),
    (lambda r: r["items"]["fault_detection"]["binds"].pop("product"),
     "precondition:C3:fault_detection:binds_missing:product"),
    (lambda r: r["items"]["start_taped"]["binds"].update(start_rear=[1.0, 0.0, 0.0]),
     "precondition:C10:start_taped:start_rear"),
    (lambda r: r["items"]["firmware_build"]["binds"].update(firmware_vehicle_calibration_id="0" * 16),
     "precondition:C12:firmware_build:firmware_vehicle_calibration_id"),
    (lambda r: r["items"]["s10_speed"]["binds"].update(approved_max_speed_mps=0.10),
     "precondition:C9:s10_speed:over_approved"),
    (lambda r: r["items"]["s10_speed"]["binds"].update(tested_speed_mps=0.10),
     "precondition:C9:s10_speed:over_tested"),
    (lambda r: r["items"]["firmware_build"]["binds"].update(board_lift_rate_max_mps=0.01),
     "precondition:C11:lift_rate_over_board"),
])
def test_record_items(site, change, reason):
    assert reason in site.change_record(change)


def _hold(r, rows=None, **kw):
    if rows is not None:
        r["items"]["lift"]["hold"] = rows
    r["items"]["lift"].update(kw)


@pytest.mark.parametrize("change,reason", [
    (lambda r: _hold(r, rows=[{"raised_cm": 8.0, "sag_cm_after_60s": 0.1, "worst": True, "worst_reason": "x"}]),
     "precondition:C11:lift:hold_no_target_height"),
    (lambda r: _hold(r, rows=[{"raised_cm": 20.0, "sag_cm_after_60s": 0.1}]),
     "precondition:C11:lift:hold_no_worst_height"),
    (lambda r: _hold(r, rows=[{"raised_cm": 8.0, "sag_cm_after_60s": 0.1, "worst": True, "worst_reason": " "},
                              {"raised_cm": 20.0, "sag_cm_after_60s": 0.1}]),
     "precondition:C11:lift:hold_no_worst_height"),
    (lambda r: _hold(r, rows=[{"raised_cm": 8.0, "sag_cm_after_60s": 1.5, "worst": True, "worst_reason": "x"},
                              {"raised_cm": 20.0, "sag_cm_after_60s": 0.1}]),
     "precondition:C11:lift:sag_over_limit"),
    (lambda r: _hold(r, sag_limit_decision_id=None), "precondition:C11:lift:sag_limit_decision_id"),
    (lambda r: _hold(r, rows="20"), "precondition:C11:lift:hold_values"),
    (lambda r: r["items"]["lift"]["origin"].update(state_lift_m_at_floor=0.008), "precondition:C11:lift:origin_floor"),
    (lambda r: r["items"]["lift"]["origin"].update(tape_cm_at_target=18.5), "precondition:C11:lift:origin_scale"),
    (lambda r: r["items"]["lift"]["origin"].update(state_lift_m_at_target=0.19, tape_cm_at_target=19.0),
     "precondition:C11:lift:origin_target"),
])
def test_lift_record(site, change, reason):
    assert reason in site.change_record(change)


def test_a_whole_range_self_locking_document_replaces_the_worst_height_row(site):
    def change(r):
        _hold(r, rows=[{"raised_cm": 20.0, "sag_cm_after_60s": 0.1}], self_locking_evidence="docs/evidence.md")

    assert site.change_record(change) == [rp.UNLOCK_INCOMPLETE]


def test_an_unmeasured_value_needs_a_lead_exception(site):
    veh = site.root / "config/real/vehicle.yaml"
    d = yaml.safe_load(veh.read_text())
    d["wheel_radius"]["provenance"]["measured"] = False
    veh.write_text(yaml.safe_dump(d, sort_keys=False))
    rec = site.full_record()  # the vehicle hash moved: the record rebuilt on the new stop files
    site.write_record(rec)
    assert "precondition:C12:vehicle:wheel_radius:unmeasured" in rp.operating_preconditions(site.facts())
    rec["config_exceptions"] = [{"file": "vehicle", "key": "wheel_radius", "decision_id": "D-3"}]
    site.write_record(rec)
    assert rp.operating_preconditions(site.facts()) == [rp.UNLOCK_INCOMPLETE]


@pytest.mark.parametrize("text,why", [
    ("schema: other\nitems: {}\n", "schema"),
    (f"schema: {rp.SCHEMA}\nitems: {{bogus: null}}\n", "items"),
    (f"schema: {rp.SCHEMA}\nitems: {{lift: 3}}\n", "item_shape:lift"),
    (f"schema: {rp.SCHEMA}\nitems: {{}}\nconfig_exceptions: [{{file: vehicle}}]\n", "config_exceptions"),
])
def test_record_shape(site, text, why):
    site.record_path.write_text(text)
    assert f"precondition:record:{why}" in rp.operating_preconditions(site.facts())


def test_the_shipped_record_is_well_formed_and_all_unset():
    rec, why = rp.load_record()
    assert why is None and set(rec["items"]) == set(rp.ITEMS) and all(v is None for v in rec["items"].values())


@pytest.mark.parametrize("change,reason", [
    (lambda l: l["timing"].update(basis=None), "precondition:C5:lidar:basis"),
    (lambda l: l["timing"].update(applies_to=None), "precondition:C5:lidar:applies_to"),
    (lambda l: l["timing"].update(basis_doc="docs/none.md"), "precondition:C5:lidar:basis_doc"),
    (lambda l: l["mount"].update(measured=False), "precondition:C5:lidar:mount_unmeasured"),
])
def test_scan_qualification_is_read_from_lidar_yaml(site, change, reason):
    p = site.root / "config/real/lidar.yaml"
    d = yaml.safe_load(p.read_text())
    change(d)
    p.write_text(yaml.safe_dump(d, sort_keys=False))
    assert reason in rp.operating_preconditions(site.facts())


def test_imu_qualification_is_apart_from_the_refusals(site):
    f = site.facts()
    assert rp.imu_qualified(f, opened_profile_fits=True) is False  # imu.yaml unmeasured today
    f2 = copy.deepcopy(f)
    f2.imu["timing"] = {"measured": True, "delay_bound_s": 0.02}
    assert rp.imu_qualified(f2, opened_profile_fits=True) is True
    assert rp.imu_qualified(f2, opened_profile_fits=None) is False
    f2.provenance["watch"] = [(k, {**v, "measured": False}) if k == "imu" else (k, v) for k, v in f2.provenance["watch"]]
    assert rp.imu_qualified(f2, opened_profile_fits=True) is False  # the level reference unmeasured
    assert not any(r.startswith("precondition:C7") for r in rp.operating_preconditions(f))
