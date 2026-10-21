import pytest

from forklift_core.mcu_link.framing import (
    FrameDecoder,
    FrameError,
    cobs_decode,
    cobs_encode,
    crc16_ccitt_false,
)

# mcu-link/1 contract section 6: ARM, seq 0x1234, boot_id 0xDEADBEEF, echo 0x01020304.
ARM_1234 = bytes.fromhex("0f01023412efbeadde04030201cc6900")


def test_crc16_ccitt_false_check_value():
    assert crc16_ccitt_false(b"123456789") == 0x29B1
    assert crc16_ccitt_false(b"") == 0xFFFF


@pytest.mark.parametrize(
    ("raw", "encoded"),
    [
        ("00", "0101"),
        ("0000", "010101"),
        ("11220033", "0311220233"),
        ("11223344", "0511223344"),
    ],
)
def test_cobs_standard_examples(raw, encoded):
    assert cobs_encode(bytes.fromhex(raw)) == bytes.fromhex(encoded)
    assert cobs_decode(bytes.fromhex(encoded)) == bytes.fromhex(raw)


def test_cobs_full_block_has_no_trailing_code():
    raw = bytes(range(1, 255))
    encoded = cobs_encode(raw)
    assert encoded == b"\xff" + raw
    assert cobs_decode(encoded) == raw


@pytest.mark.parametrize("bad", ["031100", "051122", "00"])
def test_cobs_decode_rejects_malformed(bad):
    with pytest.raises(FrameError):
        cobs_decode(bytes.fromhex(bad))


def test_decoder_resynchronises_after_garbage_and_bad_crc():
    good = ARM_1234
    corrupted = bytearray(ARM_1234)
    corrupted[5] ^= 0x01  # inside the payload: CRC no longer matches
    stream = b"\x55\x66\x00" + good + b"\x00\x00" + bytes(corrupted) + good
    decoder = FrameDecoder()
    bodies = decoder.feed(stream)
    assert len(bodies) == 2
    assert bodies[0] == bodies[1]
    assert bodies[0][:4] == bytes.fromhex("01023412")
    # the garbage frame and the CRC failure count; empty frames do not
    assert decoder.errors == 2


def test_decoder_joins_a_frame_split_across_feeds():
    decoder = FrameDecoder()
    assert decoder.feed(ARM_1234[:5]) == []
    bodies = decoder.feed(ARM_1234[5:])
    assert len(bodies) == 1
    assert decoder.errors == 0


def test_decoder_drops_oversize_frames():
    body = bytes([1, 2, 0, 0]) + bytes(80)  # 84-byte body > 80
    crc = crc16_ccitt_false(body)
    wire = cobs_encode(body + crc.to_bytes(2, "little")) + b"\x00"
    decoder = FrameDecoder()
    assert decoder.feed(wire + ARM_1234) and decoder.errors == 1
