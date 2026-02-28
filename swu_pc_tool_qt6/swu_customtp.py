# swu_customtp.py
# Minimal CustomTP framer for SWU DATA blocks.
#
# Uses the same on-wire frame structure as the attached can_custom_tp_sender.py:
# - 64-byte CAN FD payload
# - first 2 bytes = header
#   * bit7 of HDR1: LAST flag
#   * bits5..0 of HDR1 + HDR2: 14-bit sequence number
# - remaining 62 bytes: payload chunk (padded with 0x00 on last frame)
#
# This module only frames (splits) a raw PDU into 64B chunks.

from __future__ import annotations
from dataclasses import dataclass
from typing import List


MAX_CANFD_PAYLOAD = 64
HDR_SIZE = 2
CHUNK_SIZE = MAX_CANFD_PAYLOAD - HDR_SIZE  # 62


@dataclass(frozen=True)
class CustomTPFrame:
    seq: int
    is_last: bool
    payload_64b: bytes  # exactly 64 bytes


def build_frames(pdu: bytes, *, start_seq: int = 0) -> List[CustomTPFrame]:
    if start_seq < 0 or start_seq > 0x3FFF:
        raise ValueError("start_seq must be 0..0x3FFF")
    if not isinstance(pdu, (bytes, bytearray)):
        raise TypeError("pdu must be bytes-like")
    pdu = bytes(pdu)

    frames: List[CustomTPFrame] = []
    total = len(pdu)
    if total == 0:
        # represent empty as single last frame with no data
        total_chunks = 1
    else:
        total_chunks = (total + CHUNK_SIZE - 1) // CHUNK_SIZE

    for i in range(total_chunks):
        seq = (start_seq + i) & 0x3FFF
        offset = i * CHUNK_SIZE
        chunk = pdu[offset: offset + CHUNK_SIZE]
        is_last = (i == total_chunks - 1)

        # pad chunk to 62 bytes
        if len(chunk) < CHUNK_SIZE:
            chunk = chunk + b"\x00" * (CHUNK_SIZE - len(chunk))

        hdr1 = ((0x80 if is_last else 0x00) |
                ((seq >> 8) & 0x3F))  # 6 bits
        hdr2 = seq & 0xFF
        payload = bytes([hdr1, hdr2]) + chunk
        assert len(payload) == 64
        frames.append(CustomTPFrame(seq=seq, is_last=is_last, payload_64b=payload))

    return frames
