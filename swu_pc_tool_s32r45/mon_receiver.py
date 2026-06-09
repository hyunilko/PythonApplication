#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mon_receiver.py

S32R45 heartbeat(mon_packet_t) CAN 수신 및 파싱.

app_can_cmd_send() 전송 포맷:
  [0:3]   cmd      (3 bytes, BE)  = CAN_CMD_HEART_BEAT (0x404200)
  [3:7]   uniq_id  (4 bytes, BE)
  [7:55]  mon_packet_t (48 bytes, LE, packed)

mon_packet_t (LE, packed, 48 bytes):
  Offset  0: tv_sec        (uint32)
  Offset  4: tv_nsec       (uint32)
  Offset  8: error_type    (uint32)  -- mon_error_e
  Offset 12: error_code[4] (uint32 × 4 = 16 bytes)
  Offset 28: rfTxTemp[4]   (float  × 4 = 16 bytes)
  Offset 44: a53CoresTemp  (float)
  Total:  48 bytes  →  payload 55 bytes, CAN FD 64B 이내

CAN ID: (device_id << 8) | 0x42  (MS_TP_ID_SHORT_MSG)
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import List, Optional

from PyQt6.QtCore import QThread, pyqtSignal

from pcan_manager import PCANManager
from swu_constants import DEFAULT_PCAN_CHANNEL, DEFAULT_BITRATE_FD, DEFAULT_IFG_US

# ---------------------------------------------------------------------------
# CAN ID 상수 (command_def.h)
# ---------------------------------------------------------------------------
MS_TP_ID_SHORT_MSG = 0x42
CAN_CMD_HEART_BEAT = 0x404200

NUM_RF = 4   # NUM_FRONTEND_CFG

# ---------------------------------------------------------------------------
# mon_packet_t 파싱 포맷 (LE, packed)
# '<': little-endian
# II  = tv_sec, tv_nsec          (8 bytes)
# I   = error_type               (4 bytes)
# 4I  = error_code[4]            (16 bytes)
# 4f  = rfTxTemp[4]              (16 bytes)
# f   = a53CoresTemp             (4 bytes)
# Total = 48 bytes
# ---------------------------------------------------------------------------
_PKT_OFFSET = 7             # payload 내 mon_packet_t 시작 위치 (cmd 3B + uniq_id 4B)
_PKT_FMT    = '<II I 4I 4f f'
_PKT_SIZE   = struct.calcsize(_PKT_FMT)   # = 48


# ---------------------------------------------------------------------------
# Data class
# ---------------------------------------------------------------------------
@dataclass
class MonPacket:
    recv_time:    float
    tv_sec:       int
    tv_nsec:      int
    error_type:   int
    error_code:   List[int]    # [NUM_RF]
    rfTxTemp:     List[float]  # [NUM_RF]  — NaN if missing
    a53CoresTemp: float        # NaN if missing
    uniq_id:      int = 0


# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------
def parse_mon_payload(payload: bytes) -> Optional[MonPacket]:
    """payload: pcan_manager rx 큐에서 꺼낸 CAN FD 64B raw bytes."""
    if len(payload) < 7:
        return None

    cmd = (payload[0] << 16) | (payload[1] << 8) | payload[2]
    if cmd != CAN_CMD_HEART_BEAT:
        return None

    uniq_id = struct.unpack_from('<I', payload, 3)[0]

    pkt = payload[_PKT_OFFSET:]
    if len(pkt) < _PKT_SIZE:
        return None

    (tv_sec, tv_nsec,
     error_type,
     ec0, ec1, ec2, ec3,
     rf0, rf1, rf2, rf3,
     a53) = struct.unpack_from(_PKT_FMT, pkt)

    return MonPacket(
        recv_time    = time.time(),
        tv_sec       = tv_sec,
        tv_nsec      = tv_nsec,
        error_type   = error_type,
        error_code   = [ec0, ec1, ec2, ec3],
        rfTxTemp     = [rf0, rf1, rf2, rf3],
        a53CoresTemp = a53,
        uniq_id      = uniq_id,
    )


# ---------------------------------------------------------------------------
# Receiver thread
# ---------------------------------------------------------------------------
class MonReceiver(QThread):
    packet_received = pyqtSignal(object)   # MonPacket
    status_changed  = pyqtSignal(str)
    error_occurred  = pyqtSignal(str)

    def __init__(self, channel: str, bitrate_fd: str, ifg_us: int, device_id: int):
        super().__init__()
        self.channel    = channel
        self.bitrate_fd = bitrate_fd
        self.ifg_us     = ifg_us
        self.device_id  = device_id
        self._stop      = False

    def request_stop(self) -> None:
        self._stop = True

    def run(self) -> None:
        from queue import Empty
        mgr       = None
        hb_can_id = (self.device_id << 8) | MS_TP_ID_SHORT_MSG
        try:
            mgr = PCANManager()
            mgr.open(
                channel    = self.channel,
                bitrate_fd = self.bitrate_fd,
                is_std     = True,
                use_brs    = True,
                ifg_us     = self.ifg_us,
            )
            mgr.start_rx()
            self.status_changed.emit("수신 중...")

            while not self._stop:
                try:
                    payload, can_id = mgr._rx_q.get(timeout=0.1)
                except Empty:
                    continue

                if can_id != hb_can_id:
                    continue

                pkt = parse_mon_payload(bytes(payload))
                if pkt is not None:
                    self.packet_received.emit(pkt)

        except Exception as e:
            self.error_occurred.emit(str(e))
        finally:
            if mgr:
                try:
                    mgr.close()
                except Exception:
                    pass
            self.status_changed.emit("중지됨")
