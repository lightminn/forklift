"""mcu-link/1 framing: CRC-16/CCITT-FALSE, COBS and a resynchronising stream decoder.

Contract: ``docs/interfaces/2026-10-22-mcu-link-contract.md`` section 1.
"""

from __future__ import annotations

import binascii

BODY_MAX = 80  # version .. crc, bytes
_ENCODED_MAX = BODY_MAX + 1  # COBS adds one byte per started 254-byte block


class FrameError(ValueError):
    """A frame that cannot be decoded (COBS, CRC or size)."""


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, xorout 0."""
    return binascii.crc_hqx(data, 0xFFFF)


def cobs_encode(data: bytes) -> bytes:
    """COBS-encode ``data`` (no delimiter); a final full block gets no trailing code."""
    out = bytearray()
    block = bytearray()
    for index, byte in enumerate(data):
        if byte == 0:
            out.append(len(block) + 1)
            out += block
            block.clear()
            continue
        block.append(byte)
        if len(block) == 254 and index + 1 < len(data):
            out.append(0xFF)
            out += block
            block.clear()
    out.append(len(block) + 1)
    out += block
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """Inverse of :func:`cobs_encode`; raises :class:`FrameError` on malformed input."""
    out = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        index += 1
        end = index + code - 1
        if code == 0 or end > len(data):
            raise FrameError("bad COBS code")
        block = data[index:end]
        if 0 in block:
            raise FrameError("zero inside a COBS block")
        out += block
        index = end
        if code != 0xFF and index < len(data):
            out.append(0)
    return bytes(out)


def _checked_body(encoded: bytes) -> bytes:
    if len(encoded) > _ENCODED_MAX:
        raise FrameError("frame too long")
    raw = cobs_decode(encoded)
    if len(raw) > BODY_MAX:
        raise FrameError("body too long")
    if len(raw) < 6:
        raise FrameError("body too short")
    body, crc = raw[:-2], int.from_bytes(raw[-2:], "little")
    if crc16_ccitt_false(body) != crc:
        raise FrameError("CRC mismatch")
    return body


class FrameDecoder:
    """Splits a byte stream at 0x00 and returns CRC-checked bodies without the CRC.

    Broken frames are dropped and counted in ``errors``; empty frames (repeated
    delimiters) are not errors. A frame may arrive across several ``feed`` calls.
    """

    def __init__(self) -> None:
        self.errors = 0
        self._buffer = bytearray()
        self._overflow = False

    def feed(self, data: bytes) -> list[bytes]:
        bodies = []
        for byte in data:
            if byte != 0:
                if len(self._buffer) > _ENCODED_MAX:
                    self._overflow = True
                else:
                    self._buffer.append(byte)
                continue
            encoded, overflow = bytes(self._buffer), self._overflow
            self._buffer.clear()
            self._overflow = False
            if not encoded and not overflow:
                continue
            try:
                if overflow:
                    raise FrameError("frame too long")
                bodies.append(_checked_body(encoded))
            except FrameError:
                self.errors += 1
        return bodies
