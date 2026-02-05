  # swu_protocol.py
  # SWU (Software Update) protocol encoder/decoder for AWRL6844 SBL over CAN FD.
  #
  # Spec highlights :
  # - CAN Standard ID (11-bit): CAN_ID = (DeviceID<<8) | MSG_ID, DeviceID=0 => CAN_ID == MSG_ID
  #   MSG_ID: 0x32 (Control), 0x33 (Data), 0x50 (ACK)
  # - CommandID/AckID: 3 bytes, Big Endian (MSB first)
  # - SWU_REQUEST(Control, single CAN frame) now includes ModelID and SWVersion so APP can decide:
  #     * If ModelID mismatch  -> do NOT trigger, reply ACK_SWU_REQUEST FAIL (MODEL_MISMATCH)
  #     * If SWVersion match   -> do NOT trigger, reply ACK_SWU_REQUEST FAIL (VERSION_MATCH)
  #     * If different version -> set SWU flag in flash, warm reset into SBL, reply ACK_SWU_REQUEST SUCCESS
  # - DATA is sent over IsoTP/CustomTP and processed after reassembly.
  #   DATA Upper Message format (after TP reassembly):
  #     CommandID(3B, BE=0x105301)
  #     BlockIdx(2B, BE, 1-base)
  #     ExpectedCRC32(4B, BE)   // streaming CRC over "ImagePayload only"
  #     PayloadLength(4B, BE)   // valid image payload bytes (exclude TP padding)
  #     ImagePayload(PayloadLength bytes)
  #     Padding(optional)       // ignored by CRC and length checks
  #
  # Note: Multi-byte field endianness can be configured via ByteOrder for compatibility,
  #       but the SWU spec uses Big Endian for SWU payload fields.

from __future__ import annotations
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Tuple


class ByteOrder(IntEnum):
    """Byte order selector for multi-byte fields (except CommandID/AckID which are fixed BE).

Note: Use IntEnum only (do not mix str+IntEnum) for Python 3.13 compatibility.
"""
    BIG = 0
    LITTLE = 1


def pack_u16(v: int, order: ByteOrder) -> bytes:
    v &= 0xFFFF
    return v.to_bytes(2, "big" if order == ByteOrder.BIG else "little", signed=False)


def pack_u32(v: int, order: ByteOrder) -> bytes:
    v &= 0xFFFFFFFF
    return v.to_bytes(4, "big" if order == ByteOrder.BIG else "little", signed=False)


def unpack_u16(b: bytes, order: ByteOrder) -> int:
    if len(b) < 2:
        raise ValueError("need 2 bytes")
    return int.from_bytes(b[:2], "big" if order == ByteOrder.BIG else "little", signed=False)


def unpack_u32(b: bytes, order: ByteOrder) -> int:
    if len(b) < 4:
        raise ValueError("need 4 bytes")
    return int.from_bytes(b[:4], "big" if order == ByteOrder.BIG else "little", signed=False)


def pack_cmd_id_3b(cmd_id: int) -> bytes:
    """CommandID/AckID are 3 bytes BE MSB-first, e.g., 0x105301 => 0x10 0x53 0x01."""
    cmd_id &= 0xFFFFFF
    return bytes([(cmd_id >> 16) & 0xFF, (cmd_id >> 8) & 0xFF, cmd_id & 0xFF])


def unpack_cmd_id_3b(b: bytes) -> int:
    if len(b) < 3:
        raise ValueError("need 3 bytes")
    return (b[0] << 16) | (b[1] << 8) | b[2]


class MsgID(IntEnum):
    CONTROL = 0x32
    DATA = 0x33
    ACK = 0x50


class CommandID(IntEnum):
    SWU_REQUEST = 0x105001
    SWU_START = 0x105101
    DOWNLOAD_START = 0x105201
    DATA = 0x105301
    VERIFY_IMAGE = 0x105401


class AckID(IntEnum):
    ACK_SWU_REQUEST = 0x1050AA
    ACK_SWU_START = 0x1051AA
    ACK_DOWNLOAD_START = 0x1052AA
    ACK_DATA = 0x1053AA
    ACK_VERIFY = 0x1054AA

    # Backward-compatible alias (older code used AckID.DATA)
    DATA = ACK_DATA
    ACK_DATA_BLOCK_START = 0x1055AA


class Status(IntEnum):
    SUCCESS = 0x01
    FAIL = 0x44


class FailReason(IntEnum):
    SUCCESS = 0x00
    CRC_MISMATCH = 0x01
    UNEXPECTED_BLOCKIDX = 0x02
    LENGTH_MISMATCH = 0x03
    FLASH_ACCESS_FAIL = 0x04
    RX_TIMEOUT = 0x05
    META_INVALID = 0x06
    ERASE_ERROR = 0x07
    # Backward-compat alias (older documents)
    ERASE_RANGE_ERROR = ERASE_ERROR
    STATE_ERROR = 0x08

    # Scenario extensions (APP-side SWU trigger decision)
    VERSION_MATCH = 0x09      # requested SW version == current/installed version
    MODEL_MISMATCH = 0x0A     # requested model_id != device model_id


@dataclass(frozen=True)
class AckFrame:
    ack_id: int
    raw_payload: bytes


@dataclass(frozen=True)
class AckData:
    block_idx: int
    status: int
    fail_reason: int


@dataclass(frozen=True)
class AckVerify:
    status: int
    fail_reason: int


def build_swu_request(model_id: int, sw_version: int, *, order: ByteOrder = ByteOrder.BIG) -> bytes:
    """Build SWU_REQUEST control payload.

    Format:
      CommandID(3B, BE=0x105001) + ModelID(4B) + SWVersion(4B)
    """
    return pack_cmd_id_3b(CommandID.SWU_REQUEST) + pack_u32(model_id, order) + pack_u32(sw_version, order)


def build_swu_start(model_id: int, sw_version: int, date_yyyymmdd: int, image_size: int, block_size: int,
                    order: ByteOrder) -> bytes:
    return (pack_cmd_id_3b(CommandID.SWU_START) +
            pack_u32(model_id, order) +
            pack_u32(sw_version, order) +
            pack_u32(date_yyyymmdd, order) +
            pack_u32(image_size, order) +
            pack_u32(block_size, order))


def build_download_start() -> bytes:
    return pack_cmd_id_3b(CommandID.DOWNLOAD_START)


def build_data_payload(block_idx: int, expected_crc32: int, payload: bytes, *, order: ByteOrder = ByteOrder.BIG) -> bytes:
    """Build SWU DATA upper-message payload (after TP reassembly).

    Format:
      CommandID(3B, BE=0x105301) +
      BlockIdx(2B, BE, 1-base) +
      ExpectedCRC32(4B, BE) +
      PayloadLength(4B, BE) +
      Payload(PayloadLength bytes)

    - PayloadLength is the length of the image-data payload (Len(BlockIdx)).
    - ExpectedCRC32 is the streaming CRC32 computed over image-data payload bytes only
      (from image start up to end of this block).
    """
    if block_idx <= 0 or block_idx > 0xFFFF:
        raise ValueError(f"block_idx out of range: {block_idx}")
    if expected_crc32 < 0 or expected_crc32 > 0xFFFFFFFF:
        raise ValueError(f"expected_crc32 out of range: {expected_crc32}")
    if payload is None:
        raise ValueError("payload is None")
    plen = len(payload)
    # DATA header multi-byte fields are BE per spec (order kept for compatibility).
    return (
        pack_cmd_id_3b(CommandID.DATA)
        + pack_u16(block_idx, order=order)
        + pack_u32(expected_crc32, order=order)
        + pack_u32(plen, order=order)
        + payload
    )


def build_verify_image(expected_crc32: int, order: ByteOrder) -> bytes:
    return pack_cmd_id_3b(CommandID.VERIFY_IMAGE) + pack_u32(expected_crc32, order)


def parse_ack(payload_64b: bytes) -> AckFrame:
    """Parse a raw 64B CAN payload from ACK CAN ID (0x50)."""
    if payload_64b is None:
        raise ValueError("ACK payload is None")
    if len(payload_64b) < 3:
        raise ValueError("ACK payload too short")
    ack_id = unpack_cmd_id_3b(payload_64b[:3])
    return AckFrame(ack_id=ack_id, raw_payload=payload_64b)


def parse_ack_data(ack: AckFrame, order: ByteOrder) -> AckData:
    if ack.ack_id != int(AckID.ACK_DATA):
        raise ValueError(f"not ACK_DATA: 0x{ack.ack_id:06X}")
    # AckID(3) + BlockIdx(2) + Status(1) + FailReason(1)
    b = ack.raw_payload
    if len(b) < 3 + 2 + 1 + 1:
        raise ValueError("ACK_DATA too short")
    block_idx = unpack_u16(b[3:5], order)
    status = b[5]
    fail_reason = b[6]
    return AckData(block_idx=block_idx, status=status, fail_reason=fail_reason)


def parse_ack_verify(ack: AckFrame) -> AckVerify:
    if ack.ack_id != int(AckID.ACK_VERIFY):
        raise ValueError(f"not ACK_VERIFY: 0x{ack.ack_id:06X}")
    b = ack.raw_payload
    if len(b) < 3 + 1 + 1:
        raise ValueError("ACK_VERIFY too short")
    status = b[3]
    fail_reason = b[4]
    return AckVerify(status=status, fail_reason=fail_reason)


def parse_ack_status_reason(ack: AckFrame) -> Optional[Tuple[int, int]]:
    """Extract optional Status/FailReason from ACK payload.

    Many ACKs are:
      AckID(3B) + Status(1B) + FailReason(1B)
    If those bytes are missing (or padding is zeros), returns None unless status is valid.
    """
    b = ack.raw_payload
    if len(b) < 5:
        return None
    status = b[3]
    reason = b[4]
    if status not in (int(Status.SUCCESS), int(Status.FAIL)):
        return None
    return status, reason
