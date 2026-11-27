"""Bench extension of mcu-link (contract section 8, wire generation bench6): BENCH_SET, BENCH_ARM, BENCH_STATE.

The golden vectors are shared with the firmware's test_link.c; the operating v3 vectors stay in test_messages.py."""

import binascii
import struct

import pytest

from forklift_core.mcu_link.framing import BODY_MAX, cobs_decode, cobs_encode
from forklift_core.mcu_link.messages import (
    TYPE_BENCH_ARM,
    TYPE_BENCH_SET,
    TYPE_BENCH_STATE,
    BenchArm,
    BenchDriveState,
    BenchSet,
    BenchState,
    LinkError,
    decode,
    encode,
)

BENCH_SET_000B = (
    "0403060b08efbeaddeefcdab020101010778563412010203010101010c10271027c409dc059bc420049bc4200330750103d0070101010101"
    "03e94200"
)
BENCH_ARM_000C = "0403080c08efbeaddeefcdab0201010107785634124c7300"
BENCH_STATE_0103 = (
    "1003820301020178563412443322110201010abc0ac01dfefff1fb0906f9ffffff08010102010203020404fffffa04011305020203659700"
)

SET = BenchSet(boot_id=0xDEADBEEF, echo_mcu_time_us=0x00000001_00ABCDEF, run_id=0x12345678, mode=1, axis=2, source=0,
               count_sign=1, bypass=1, duty_max_e4=(10000, 10000, 2500, 1500), time_ms=(2147483, 2147483, 30000, 2000),
               stop_counts=0)
ARM = BenchArm(boot_id=0xDEADBEEF, echo_mcu_time_us=0x00000001_00ABCDEF, run_id=0x12345678)
STATE = BenchState(pair_seq=0x0102, run_id=0x12345678, result_run_id=0x11223344, axis=2, source=0, count_state=0,
                   kth_lost=0, raw_angle=0x0ABC, cumulative=-123456, target=654321, distance_ref=-7, cut_position=8,
                   stop_reason=1, distance_confirm=0, blocked=3, dropped_pairs=4, evidence_age_ms=0xFFFF,
                   end_evidence_age_ms=250, mcu_bypass=1, drive=0x13, conn_gen=5, run_flags=0, result_axis=2,
                   result_source=0)

# bench6 meaning (plan 4.2 A1b-1): an H KTH run stopping at -4096 with count_sign -1, and a provisional cut
# waiting for its confirmation (stop 9, confirm 0), H lost, run flags UNCLEAN | RES_UNCLEAN | RES_TENTATIVE, the
# live evidence age saturated (0xFFFE) and the end age 3 ms.
BENCH_SET_000D = "0403060d08efbeaddeefcdab020101010a0df0ad0b010301ff0201010c10271027dc05d0079bc420049bc42003d00701038813010106f0ffff684b00"
BENCH_STATE_0202 = "190382020201020df0ad0b0df0ad0b03010102230178ecffff0101010a78fcffff78ecffff090202010104feff03090213070b0301245500"
SET6 = BenchSet(boot_id=0xDEADBEEF, echo_mcu_time_us=0x00000001_00ABCDEF, run_id=0x0BADF00D, mode=1, axis=3, source=1,
                count_sign=-1, bypass=2, duty_max_e4=(10000, 10000, 1500, 2000), time_ms=(2147483, 2147483, 2000, 5000),
                stop_counts=-4096)
STATE6 = BenchState(pair_seq=0x0201, run_id=0x0BADF00D, result_run_id=0x0BADF00D, axis=3, source=1, count_state=1,
                    kth_lost=2, raw_angle=0x0123, cumulative=-5000, target=0, distance_ref=-904, cut_position=-5000,
                    stop_reason=9, distance_confirm=0, blocked=2, dropped_pairs=0, evidence_age_ms=0xFFFE,
                    end_evidence_age_ms=3, mcu_bypass=2, drive=0x13, conn_gen=7, run_flags=0x0B, result_axis=3,
                    result_source=1)

GOLDEN = [(SET, 0x000B, BENCH_SET_000B), (ARM, 0x000C, BENCH_ARM_000C), (STATE, 0x0103, BENCH_STATE_0103),
          (SET6, 0x000D, BENCH_SET_000D), (STATE6, 0x0202, BENCH_STATE_0202)]


def body_of(wire_hex: str) -> bytes:
    return cobs_decode(bytes.fromhex(wire_hex)[:-1])[:-2]


@pytest.mark.parametrize(("message", "seq", "wire"), GOLDEN)
def test_bench_golden_vectors_encode(message, seq, wire):
    assert encode(message, seq).hex() == wire


@pytest.mark.parametrize(("message", "seq", "wire"), GOLDEN)
def test_bench_golden_vectors_decode(message, seq, wire):
    assert decode(body_of(wire)) == (seq, message)


def test_the_bench_vectors_are_the_contract_layouts_with_the_standard_crc():
    def wire(type_, seq, payload):
        body = struct.pack("<BBH", 3, type_, seq) + payload
        body += binascii.crc_hqx(body, 0xFFFF).to_bytes(2, "little")
        return (cobs_encode(body) + b"\x00").hex()

    payload = struct.pack("<IQIBBBbI", 0xDEADBEEF, 0x00000001_00ABCDEF, 0x12345678, 1, 2, 0, 1, 1)
    payload += struct.pack("<4H", 10000, 10000, 2500, 1500) + struct.pack("<4I", 2147483, 2147483, 30000, 2000)
    payload += struct.pack("<i", 0)
    assert len(payload) == 52 and wire(TYPE_BENCH_SET, 0x000B, payload) == BENCH_SET_000B
    payload = struct.pack("<IQI", 0xDEADBEEF, 0x00000001_00ABCDEF, 0x12345678)
    assert len(payload) == 16 and wire(TYPE_BENCH_ARM, 0x000C, payload) == BENCH_ARM_000C
    payload = struct.pack("<HII", 0x0102, 0x12345678, 0x11223344) + bytes([2, 0, 0, 0])
    payload += struct.pack("<Hiiii", 0x0ABC, -123456, 654321, -7, 8) + bytes([1, 0])
    payload += struct.pack("<HHHH", 3, 4, 0xFFFF, 250) + bytes([1, 0x13, 5, 0, 2, 0])
    assert len(payload) == 48 and wire(TYPE_BENCH_STATE, 0x0103, payload) == BENCH_STATE_0103


def test_every_bench_message_fits_the_v3_body():
    assert all(len(body_of(w)) + 2 <= BODY_MAX for _, _, w in GOLDEN)


def test_the_drive_byte_reads_state_and_mode():
    assert STATE.drive_state is BenchDriveState.ENDED and STATE.mode == 1


@pytest.mark.parametrize("kw", [{"boot_id": 0}, {"run_id": 0}, {"count_sign": 128}, {"stop_counts": 2**31},
                                {"duty_max_e4": (1, 2, 3)}, {"time_ms": (1, 2, 3, 2**32)}])
def test_bench_set_refuses_what_cannot_be_encoded(kw):
    with pytest.raises(ValueError):
        BenchSet(**{**SET.__dict__, **kw})


def test_bench_arm_needs_a_run_id():
    with pytest.raises(ValueError):
        BenchArm(boot_id=1, echo_mcu_time_us=0, run_id=0)


def test_a_bench_message_of_the_wrong_length_is_refused():
    body = body_of(BENCH_ARM_000C)
    with pytest.raises(LinkError):
        decode(body + b"\x00")
