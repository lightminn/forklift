import math
import struct

import pytest

from forklift_core.mcu_link.framing import cobs_decode, cobs_encode, crc16_ccitt_false
from forklift_core.mcu_link.messages import (
    Arm,
    ClearFaults,
    Commission,
    Disarm,
    Drive,
    FaultFlag,
    LinkDecoder,
    LinkError,
    McuState,
    Mode,
    RejectReason,
    ResetCause,
    StateFlag,
    decode,
    encode,
    valid_after_echo_ms,
    wheel_count_delta,
)

# mcu-link/3 contract section 6 golden vectors (v3: every vector remade, the version is in every CRC).
ARM_1234 = "0e03023412efbeadde04030201050101031f0500"
DRIVE_0007 = (
    "04030107010103c03f010310c00101073e9a99993e960b010302efbeaddeefcdab0201"
    "0101033f4600"
)
CLEAR_0008 = "0403040807efbeaddea4c100"
DISARM_0009 = "0403030903d4fc00"
COMMISSION_000A = "0403050a050224fa960a0302efbeaddeefcdab02010101037f4500"
STATE_0102 = (
    "0a03810201443322110201010eefbeadde03020201020701800108c01dfefff1fb0901"
    "0103a03f010102bf0103803d0108403fdc0538ff1e09a00f8813f0d8c4090104c0c4e0"
    "02110202020b063412ffff0301010103010703c0b000"
)
STATE_0103 = (
    "0a03810301443322110201010befbeadde030202010207010a4a01c01dfefff1fb0901"
    "0103a03f010102bf0103803d0108403fdc0538ff1e09a00f8813f0d8c4090104c0c4e0"
    "0211020203d90203800104ffff02020505ff7fc42100"
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
    kth_frames=(0x1234, 0xFFFF),
    kth_crc_errors=(3, 0),
    kth_alt_crc_errors=(0x0100, 7),
)
# v2: one axis's commissioning duty, and the new fault and build bits with a reset cause (every flag bit is defined)
COMMISSION = Commission(axis=2, duty_e4=-1500, valid_after_echo_ms=150, arm_gen=0x0203, boot_id=0xDEADBEEF,
                        echo_mcu_time_us=0x00000001_00ABCDEF)
STATE_V2_BITS = McuState(
    **{**STATE.__dict__,
       "fault_flags": FaultFlag.ENCODER_GAP | FaultFlag.ENCODER_STUCK_R | FaultFlag.ENCODER_JUMP_L
       | FaultFlag.COMMISSION_TIME_LIMIT,
       "flags": StateFlag.LIMITED | StateFlag.DRIVERS_READY | StateFlag.COMMISSION_BUILD | StateFlag.OUTPUTS_LOCKED
       | StateFlag.ESTOP_UNWIRED,
       "reset_cause": ResetCause.IWDG,
       "kth_frames": (0x8000, 1), "kth_crc_errors": (0xFFFF, 2), "kth_alt_crc_errors": (5, 0x7FFF)},
)

GOLDEN = [
    (Arm(boot_id=0xDEADBEEF, echo_mcu_time_us=0x00000005_01020304), 0x1234, ARM_1234),
    (DRIVE, 7, DRIVE_0007),
    (ClearFaults(boot_id=0xDEADBEEF), 8, CLEAR_0008),
    (Disarm(), 9, DISARM_0009),
    (STATE, 0x0102, STATE_0102),
    (COMMISSION, 0x000A, COMMISSION_000A),
    (STATE_V2_BITS, 0x0103, STATE_0103),
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
    return struct.pack("<BBH", 3, type_, 5) + payload


STATE_FORMAT = "<QIHHBBIiiffff4h4h4BHHBB6H"


@pytest.mark.parametrize(
    ("index", "value"),
    [
        (4, 6),  # mode beyond ESTOP
        (5, 16),  # reject reason beyond estop_unwired
        (6, 1 << 25),  # undefined fault bit
        (28, 5),  # reset cause beyond OTHER
    ],
)
def test_decode_rejects_undefined_bits_and_enums(index, value):
    fields = list(struct.unpack(STATE_FORMAT, body_of(STATE_0102)[4:]))
    fields[index] = value
    with pytest.raises(LinkError):
        decode(_body(0x81, struct.pack(STATE_FORMAT, *fields)))


def test_decode_rejects_bad_framing_fields():
    with pytest.raises(LinkError):
        decode(struct.pack("<BBH", 1, 0x03, 0))  # version 1 (mcu-link/1)
    with pytest.raises(LinkError):
        decode(struct.pack("<BBH", 2, 0x7F, 0))  # unknown type
    with pytest.raises(LinkError):
        decode(struct.pack("<BBH", 2, 0x02, 0) + bytes(11))  # ARM payload 11 bytes
    with pytest.raises(LinkError):
        decode(struct.pack("<BBH", 2, 0x05, 0) + bytes(18))  # COMMISSION payload 18 bytes


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


def test_the_golden_vectors_are_the_contract_formats_with_the_standard_crc():
    """Independent of the module: struct formats of the contract and binascii's CRC-16/CCITT-FALSE."""
    import binascii

    body = struct.pack("<BBH", 3, 0x05, 0x000A) + struct.pack("<BhHHIQ", 2, -1500, 150, 0x0203, 0xDEADBEEF,
                                                               0x00000001_00ABCDEF)
    body += binascii.crc_hqx(body, 0xFFFF).to_bytes(2, "little")
    assert cobs_encode(body) + b"\x00" == bytes.fromhex(COMMISSION_000A)
    fields = (0x00000002_11223344, 0xDEADBEEF, 0x0203, 0x0102, 2, 7, (1 << 17) | (1 << 19) | (1 << 22) | (1 << 24),
              -123456, 654321, 1.25, -0.5, 0.0625, 0.75, 1500, -200, 30, 4000, 5000, -10000, 2500, 0,
              0xC0, 0xC4, 0xE0, 0x00, 17, 2, 0x01 | 0x08 | 0x10 | 0x40 | 0x80, 2,
              0x8000, 1, 0xFFFF, 2, 5, 0x7FFF)  # v3: kth_frames, kth_crc_errors, kth_alt_crc_errors (M, H)
    body = struct.pack("<BBH", 3, 0x81, 0x0103) + struct.pack("<QIHHBBIiiffff4h4h4BHHBB6H", *fields)
    body += binascii.crc_hqx(body, 0xFFFF).to_bytes(2, "little")
    assert cobs_encode(body) + b"\x00" == bytes.fromhex(STATE_0103)


@pytest.mark.parametrize("kw", [dict(axis=4), dict(axis=-1), dict(duty_e4=10001), dict(duty_e4=-10001),
                                dict(valid_after_echo_ms=0), dict(arm_gen=0), dict(boot_id=0)])
def test_commission_rejects_out_of_range_values(kw):
    base = dict(axis=0, duty_e4=0, valid_after_echo_ms=150, arm_gen=1, boot_id=1, echo_mcu_time_us=0)
    with pytest.raises(ValueError):
        Commission(**{**base, **kw})


# --- v3: the KTH counters (contract 3.3) ---------------------------------------------------------------------------

def test_version_2_frames_are_refused():
    body = struct.pack("<BBH", 2, 0x03, 9)
    with pytest.raises(LinkError, match="version"):
        decode(body)


def test_the_kth_counters_round_trip_at_their_bounds():
    for vals in ((0, 0, 0, 0, 0, 0), (0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF), (1, 2, 3, 4, 5, 6)):
        st = McuState(**{**STATE.__dict__, "kth_frames": vals[0:2], "kth_crc_errors": vals[2:4],
                         "kth_alt_crc_errors": vals[4:6]})
        _seq, back = decode(body_of(encode(st, 1).hex()))
        assert (back.kth_frames, back.kth_crc_errors, back.kth_alt_crc_errors) == (vals[0:2], vals[2:4], vals[4:6])


@pytest.mark.parametrize("field, bad", [("kth_frames", (0x10000, 0)), ("kth_crc_errors", (0, -1)),
                                        ("kth_alt_crc_errors", (1,))])
def test_a_kth_counter_out_of_range_is_not_encoded(field, bad):
    with pytest.raises((LinkError, ValueError, struct.error)):
        encode(McuState(**{**STATE.__dict__, field: bad}), 1)


def test_the_kth_counter_delta_is_unsigned_modular():
    from forklift_core.mcu_link.messages import kth_counter_delta

    assert kth_counter_delta(5, 0xFFFE) == 7
    assert kth_counter_delta(0x8000, 0) == 0x8000  # never the signed half-range reading
    assert kth_counter_delta(3, 3) == 0


def test_a_v3_state_fills_the_largest_body():
    from forklift_core.mcu_link.framing import BODY_MAX

    assert len(body_of(STATE_0102)) + 2 == BODY_MAX == 90  # body_of drops the CRC
