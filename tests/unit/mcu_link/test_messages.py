import math
import struct

import pytest

from forklift_core.mcu_link.framing import cobs_decode, cobs_encode, crc16_ccitt_false
from forklift_core.mcu_link.messages import (
    Arm,
    ClearFaults,
    Disarm,
    Drive,
    FaultFlag,
    LinkDecoder,
    LinkError,
    McuState,
    Mode,
    RejectReason,
    StateFlag,
    decode,
    encode,
    valid_after_echo_ms,
    wheel_count_delta,
)

# mcu-link/1 contract section 6 golden vectors.
ARM_1234 = "0e01023412efbeadde0403020105010103764500"
DRIVE_0007 = (
    "04010107010103c03f010310c00101073e9a99993e960b010302efbeaddeefcdab020101010388be00"
)
CLEAR_0008 = "0401040807efbeadde024e00"
DISARM_0009 = "0401030903bc1100"
STATE_0102 = (
    "0a01810201443322110201010eefbeadde03020201020701800108c01dfefff1fb0901"
    "0103a03f010102bf0103803d0108403fdc0538ff1e09a00f8813f0d8c4090104c0c4e0"
    "02110202040b2cbb00"
)

DRIVE = Drive(
    left_wheel_rad_s=1.5,
    right_wheel_rad_s=-2.25,
    steering_rad=0.125,
    lift_m=struct.unpack("<f", struct.pack("<f", 0.3))[0],
    valid_after_echo_ms=150,
    lift_hold=True,
    arm_gen=0x0203,
    boot_id=0xDEADBEEF,
    echo_mcu_time_us=0x00000001_00ABCDEF,
)
STATE = McuState(
    mcu_time_us=0x00000002_11223344,
    boot_id=0xDEADBEEF,
    arm_gen=0x0203,
    last_rx_seq=0x0102,
    mode=Mode.ARMED,
    reject_reason=RejectReason.ZERO_FIRST,
    fault_flags=FaultFlag.ESTOP_INPUT
    | FaultFlag.CYCLE_OVERRUN
    | FaultFlag.CURRENT_SENSE_INVALID,
    wheel_count_left=-123456,
    wheel_count_right=654321,
    wheel_rate_left_rad_s=1.25,
    wheel_rate_right_rad_s=-0.5,
    steering_rad=0.0625,
    lift_m=0.75,
    current_ma=(1500, -200, 30, 4000),
    duty_e4=(5000, -10000, 2500, 0),
    driver_status=(0xC0, 0xC4, 0xE0, 0x00),
    rx_errors=17,
    cycle_overruns=2,
    flags=StateFlag.LIMITED | StateFlag.COMMAND_FRESH | StateFlag.DRIVERS_READY,
)

GOLDEN = [
    (Arm(boot_id=0xDEADBEEF, echo_mcu_time_us=0x00000005_01020304), 0x1234, ARM_1234),
    (DRIVE, 7, DRIVE_0007),
    (ClearFaults(boot_id=0xDEADBEEF), 8, CLEAR_0008),
    (Disarm(), 9, DISARM_0009),
    (STATE, 0x0102, STATE_0102),
]


def body_of(wire_hex: str) -> bytes:
    raw = cobs_decode(bytes.fromhex(wire_hex)[:-1])
    return raw[:-2]


@pytest.mark.parametrize(("message", "seq", "wire"), GOLDEN)
def test_golden_vectors_encode(message, seq, wire):
    assert encode(message, seq).hex() == wire


@pytest.mark.parametrize(("message", "seq", "wire"), GOLDEN)
def test_golden_vectors_decode(message, seq, wire):
    assert decode(body_of(wire)) == (seq, message)


def test_state_si_views():
    assert STATE.current_a == (1.5, -0.2, 0.03, 4.0)
    assert STATE.duty == (0.5, -1.0, 0.25, 0.0)


def test_drive_rejects_bad_values():
    with pytest.raises(ValueError):
        Drive(math.nan, 0, 0, 0, 100, False, 1, 1, 0)
    with pytest.raises(ValueError):
        Drive(0, 0, math.inf, 0, 100, False, 1, 1, 0)
    for bad in ({"valid_after_echo_ms": 0}, {"arm_gen": 0}, {"boot_id": 0}):
        kwargs = dict(
            left_wheel_rad_s=0,
            right_wheel_rad_s=0,
            steering_rad=0,
            lift_m=0,
            valid_after_echo_ms=100,
            lift_hold=False,
            arm_gen=1,
            boot_id=1,
            echo_mcu_time_us=0,
        )
        kwargs.update(bad)
        with pytest.raises(ValueError):
            Drive(**kwargs)
    with pytest.raises(ValueError):
        Arm(boot_id=0, echo_mcu_time_us=0)
    with pytest.raises(ValueError):
        ClearFaults(boot_id=2**32)


def _body(type_: int, payload: bytes) -> bytes:
    return struct.pack("<BBH", 1, type_, 5) + payload


STATE_FORMAT = "<QIHHBBIiiffff4h4h4BHHB"


@pytest.mark.parametrize(
    ("index", "value"),
    [
        (4, 6),  # mode beyond ESTOP
        (5, 14),  # reject reason beyond sensor_invalid
        (6, 1 << 17),  # undefined fault bit
        (27, 0x10),  # undefined state flag
    ],
)
def test_decode_rejects_undefined_bits_and_enums(index, value):
    fields = list(struct.unpack(STATE_FORMAT, body_of(STATE_0102)[4:]))
    fields[index] = value
    with pytest.raises(LinkError):
        decode(_body(0x81, struct.pack(STATE_FORMAT, *fields)))


def test_decode_rejects_bad_framing_fields():
    with pytest.raises(LinkError):
        decode(struct.pack("<BBH", 2, 0x03, 0))  # version 2
    with pytest.raises(LinkError):
        decode(struct.pack("<BBH", 1, 0x7F, 0))  # unknown type
    with pytest.raises(LinkError):
        decode(struct.pack("<BBH", 1, 0x02, 0) + bytes(11))  # ARM payload 11 bytes


def test_link_decoder_keeps_host_receive_time():
    decoder = LinkDecoder()
    wire = bytes.fromhex(STATE_0102) + b"\x55\x00" + bytes.fromhex(DISARM_0009)
    received = decoder.feed(wire, received_ns=123_456_789)
    assert [r.seq for r in received] == [0x0102, 9]
    assert received[0].message == STATE
    assert all(r.received_ns == 123_456_789 for r in received)
    assert decoder.errors == 1


def test_link_decoder_counts_message_level_errors():
    body = struct.pack("<BBH", 1, 0x7F, 0)
    raw = body + crc16_ccitt_false(body).to_bytes(2, "little")
    decoder = LinkDecoder()
    assert decoder.feed(cobs_encode(raw) + b"\x00", received_ns=0) == []
    assert decoder.errors == 1


def test_valid_after_echo_ms():
    assert valid_after_echo_ms(1_150_000_000, 1_000_000_000) == 150
    assert valid_after_echo_ms(1_150_999_999, 1_000_000_000) == 150  # rounds down
    assert valid_after_echo_ms(10**12, 0) == 65535  # clamped to the field
    for expires in (1_000_000_000, 1_000_999_999, 999_000_000):  # nothing left to send
        with pytest.raises(ValueError):
            valid_after_echo_ms(expires, 1_000_000_000)


def test_wheel_count_delta_is_modular():
    assert wheel_count_delta(-(2**31), 2**31 - 1) == 1
    assert wheel_count_delta(5, 10) == -5
