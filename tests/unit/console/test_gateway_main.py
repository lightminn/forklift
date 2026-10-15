import pytest

from console.gateway.__main__ import ISAAC_STOP_INSTRUCTION, parse_args


def test_isaac_defaults_to_stopping_the_simulation():
    args, ros = parse_args(
        ["--backend", "isaac", "--ros-args", "-p", "use_sim_time:=true"]
    )
    assert args.stop_instruction == ISAAC_STOP_INSTRUCTION
    assert ros == ["--ros-args", "-p", "use_sim_time:=true"]


def test_the_real_truck_needs_its_actual_stop_method(capsys):
    # there is no tested physical e-stop yet (2026-10-12): the operator must be told
    # the method that really exists on that truck, so there is no default
    with pytest.raises(SystemExit):
        parse_args(["--backend", "real"])
    assert "--stop-instruction" in capsys.readouterr().err
    args, _ = parse_args(
        ["--backend", "real", "--stop-instruction", "주 전원 스위치를 끄세요"]
    )
    assert args.stop_instruction == "주 전원 스위치를 끄세요"
