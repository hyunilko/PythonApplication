#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_custom_tp_send_profilecfg.py

- MMWave_ProfileComCfg 구조체를 Application PDU 형태로 전송:
    APP_PDU = [Len(4B, LE)] + [MsgID(1B)] + Payload

- Transport:
    Custom-TP (can_custom_tp_sender.CanCustomTpSender)
    각 CAN FD 프레임 (64B):
        [HDR1,HDR2] + 62B 데이터
    (분할/프레이밍은 CanCustomTpSender 내부에서 처리)

Usage:
    python can_custom_tp_send_profilecfg.py --msg-id 0x24 [옵션들...]

Requirements:
    - can_custom_tp_sender.py (Custom-TP 구현, send_app_pdu 사용)
    - mmwave_profile_cfg.py   (MMWave_ProfileComCfg 정의)
"""

from __future__ import annotations
import argparse
import logging
import time
import sys
import os

# Add parent directory to path for cross-package imports
_parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _parent_dir not in sys.path:
    sys.path.insert(0, _parent_dir)

# Project modules
try:
    from can_customtp.can_custom_tp_sender import (
        CanCustomTpSender,
        DEFAULT_CHANNEL,
        DEFAULT_BITRATE_FD,
        DEFAULT_FRAME_GAP_US,
        DEFAULT_CMD_GAP_MS,
    )
except Exception as e:
    print(f"[ERR] can_custom_tp_sender import failed: {e}", file=sys.stderr)
    raise

try:
    from can_customtp.mmwave_profile_cfg import MMWave_ProfileComCfg, hexdump, WIRE_SIZE
except Exception as e:
    print(f"[ERR] mmwave_profile_cfg import failed: {e}", file=sys.stderr)
    raise


log = logging.getLogger("can_custom_tp_send_profilecfg")


def build_args():
    p = argparse.ArgumentParser(
        description="Send MMWave_ProfileComCfg as APP_PDU over Custom-TP (PCAN FD, HDR2B+62B)"
    )

    # --- Application level ---
    p.add_argument(
        "--msg-id",
        type=lambda x: int(x, 0),
        default=0x24,
        help="Application Msg ID (1B, default 0x24)",
    )

    # --- Profile fields (예시 기본값, 실제 프로젝트에 맞게 수정 가능) ---
    p.add_argument("--samp",  type=int,   default=1,    help="digOutputSampRate (uint8)")
    p.add_argument("--bits",  type=int,   default=2,    help="digOutputBitsSel  (uint8)")
    p.add_argument("--fir",   type=int,   default=3,    help="dfeFirSel         (uint8)")
    p.add_argument("--adc",   type=int,   default=4,    help="numOfAdcSamples   (uint16)")
    p.add_argument("--ramp",  type=float, default=5.0,  help="chirpRampEndTimeus (float)")
    p.add_argument("--rxhpf", type=int,   default=6,    help="chirpRxHpfSel     (uint8)")
    p.add_argument("--mimo",  type=int,   default=7,    help="chirpTxMimoPatSel (uint8)")

    # --- Transport/Link params ---
    p.add_argument(
        "--channel",
        default = DEFAULT_CHANNEL,
        help    = "PCAN channel (e.g., PCAN_USBBUS1)",
    )
    p.add_argument(
        "--bitrate-fd",
        default=DEFAULT_BITRATE_FD,
        help="PCAN FD bitrate string",
    )
    p.add_argument(
        "--id",
        dest="can_id",
        type=lambda x: int(x, 0),
        default=0xC0,
        help="11-bit CAN ID (link-layer)",
    )
    p.add_argument(
        "--ifg-us",
        type=int,
        default=DEFAULT_FRAME_GAP_US,
        help="inter-frame gap (us) for PCAN",
    )
    p.add_argument(
        "--cmd-gap-ms",
        type=int,
        default=DEFAULT_CMD_GAP_MS,
        help="gap after one logical APP_PDU (ms)",
    )

    # --- Logging/behavior ---
    p.add_argument(
        "--verbose",
        action="store_true",
        help="enable debug logging",
    )
    p.add_argument(
        "--sleep-after",
        type=float,
        default=0.05,
        help="sleep after TX to let PCAN drain (seconds)",
    )
    return p


def main() -> int:
    args = build_args().parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    app_msg_id = args.msg_id & 0xFF

    # 1) Profile payload 구성
    cfg = MMWave_ProfileComCfg(
        digOutputSampRate=args.samp,
        digOutputBitsSel=args.bits,
        dfeFirSel=args.fir,
        numOfAdcSamples=args.adc,
        chirpRampEndTimeus=args.ramp,
        chirpRxHpfSel=args.rxhpf,
        chirpTxMimoPatSel=args.mimo,
    )
    payload = cfg.to_bytes()

    if len(payload) != WIRE_SIZE:
        log.warning(
            "Payload size = %d (expected %d) - 계속 진행하지만 구조체 정의를 확인하세요.",
            len(payload),
            WIRE_SIZE,
        )

    log.info(
        "APP_PDU: MsgID=0x%02X, PayloadLen=%d, Payload=%s",
        app_msg_id,
        len(payload),
        hexdump(payload),
    )

    # 2) Custom-TP Sender 준비
    sender = CanCustomTpSender(
        channel=args.channel,
        bitrate_fd=args.bitrate_fd,
        can_id_11bit=args.can_id,
        app_msg_id=app_msg_id,     # 텍스트 모드용 기본 MsgID (여기선 큰 의미 X)
        append_lf=False,           # binary payload: LF 붙이면 안됨
        frame_gap_us=args.ifg_us,
        cmd_gap_ms=args.cmd_gap_ms,
    )

    try:
        # 3) 연결
        sender.connect()
        log.info(
            "Connected: channel=%s, link_id=0x%X, frame_gap_us=%d, cmd_gap_ms=%d",
            sender.channel,
            sender.can_id_11bit,
            sender.frame_gap_us,
            sender.cmd_gap_ms,
        )

        # 4) APP_PDU 전송
        #    내부에서:
        #      APP_PDU = [msg_id] + payload
        #      → 62B 조각으로 나눠 HDR(SEQ, LAST/MIDDLE) 붙여 여러 프레임 전송
        sender.send_app_pdu(app_msg_id, payload)

        # 참고용: 예상 프레임 수 로깅 (디버그)
        total_len = 4 + 1 + len(payload)  # Len(4) + MsgID + payload
        frames_expected = (total_len + CHUNK_DATA_MAX - 1) // CHUNK_DATA_MAX
        log.info(
            "TX done: APP_PDU %d bytes -> %d frame(s) via Custom-TP",
            total_len,
            frames_expected,
        )

        time.sleep(max(0.0, float(args.sleep_after)))

    except Exception as e:
        log.error("Transmission failed: %s", e)
        return 1
    finally:
        try:
            sender.disconnect()
        except Exception:
            pass

    return 0


# CHUNK_DATA_MAX를 여기서도 쓰므로 sender와 동일 값 유지
CHUNK_DATA_MAX = 62

if __name__ == "__main__":
    raise SystemExit(main())
