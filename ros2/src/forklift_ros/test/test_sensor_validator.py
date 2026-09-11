"""Failure reporting at the validator CLI boundary, without a ROS runtime."""

import importlib
import json
import sys
import types
from pathlib import Path

import pytest


def test_png_export_failure_cannot_report_success(tmp_path, monkeypatch):
    package_name = "validator_export_test"
    package = types.ModuleType(package_name)
    package.__path__ = [str(Path(__file__).resolve().parents[1] / "forklift_ros")]
    monkeypatch.setitem(sys.modules, package_name, package)
    try:
        api = importlib.import_module(f"{package_name}.sensor_validator")
        output = tmp_path / "result"
        monkeypatch.setattr(
            sys, "argv", ["validator", "--phase", "live", "--output", str(output)]
        )
        monkeypatch.setattr(api, "observe_ros", lambda args, state: {"passed": True})

        def cannot_export(self, path):
            raise OSError("PNG output unavailable")

        monkeypatch.setattr(api.ExperimentWindow, "export_pngs", cannot_export)
        with pytest.raises(SystemExit) as stopped:
            api.main()
        result = json.loads((output / "result.json").read_text())
        assert stopped.value.code == 1
        assert result["passed"] is False
        assert result["error"] == "OSError: PNG output unavailable"
    finally:
        for name in list(sys.modules):
            if name.startswith(package_name + "."):
                del sys.modules[name]
