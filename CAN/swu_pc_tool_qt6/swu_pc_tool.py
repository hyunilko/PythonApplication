  #!/usr/bin/env python3
  # -*- coding: utf-8 -*-
"""
swu_pc_tool.py

SWU(Software Update) 전용 PC Tool (CustomTP 적용)
- Control/Verify: Raw CAN FD (Standard ID 0x32)
- Data blocks: CustomTP segmentation over CAN FD (Standard ID 0x33)
- ACK: Raw CAN FD (Standard ID 0x50)

의존 파일(첨부 파일 기반):
- pcan_manager.py
- PCANBasic.py

사용 예:
  python swu_pc_tool.py --bin firmware.bin --model-id 0x68440001 --sw-version 0x00010002 --date 20260108 --block-size 65536

주의:
- CommandID/AckID는 3바이트 Big Endian(MSB first)로 전송합니다.
- BlockIdx/CRC/Meta 필드 endianness는 기본 BIG로 두었지만, --field-endian little 옵션으로 변경 가능합니다.
- SWU_REQUEST는 ModelID, SWVersion을 포함합니다.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
import struct
import zlib
from dataclasses import dataclass
from typing import Optional, Tuple

# --- local imports (same folder) ---
try:
    from pcan_manager import PCANManager
except Exception as e:
    print("ERROR: pcan_manager.py import 실패. 같은 폴더에 두세요.", file=sys.stderr)
    raise

from swu_protocol import (
    MsgID, AckID, Status, FailReason, ByteOrder,
    build_swu_request, build_swu_start, build_download_start,
    build_data_payload, build_verify_image,
    parse_ack, parse_ack_data, parse_ack_verify, parse_ack_status_reason,
)

from swu_customtp import build_frames

from swu_constants import (
    DEFAULT_PCAN_CHANNEL,
    DEFAULT_BITRATE_FD,
    DEFAULT_IFG_US,
    TIMEOUT_ACK_DEFAULT_S,
    TIMEOUT_ACK_DATA_S,
    RETRY_SWU_REQUEST_ATTEMPTS,
    RETRY_SWU_REQUEST_INTERVAL_S,
    RETRY_CTRL_ATTEMPTS,
    RETRY_CTRL_INTERVAL_S,
    RETRY_DOWNLOAD_START_ATTEMPTS,
    RETRY_DOWNLOAD_START_INTERVAL_S,
    RETRY_VERIFY_ATTEMPTS,
    RETRY_VERIFY_INTERVAL_S,
    RETRY_DATA_BLOCK_ATTEMPTS,
)

from swu_utils import (
    parse_int_auto,
    extract_sw_version_from_filename,
    format_fail_reason,
)

from swu_logging import setup_logging, get_logger


@dataclass
class RetryPolicy:
    attempts: int
    interval_s: float


class SWUTool:
    def __init__(
        self,
        mgr: PCANManager,
        *,
        field_order: ByteOrder = ByteOrder.BIG,
        ack_timeout_s: float = 0.2,
        data_ack_timeout_s: float = 2.0,
        max_block_retries: int = 3,
    ):
        self.mgr = mgr
        self.field_order = field_order
        self.ack_timeout_s = float(ack_timeout_s)

        # Used by send_block()
        self.data_ack_timeout_s = float(data_ack_timeout_s)
        self.max_block_retries = int(max_block_retries) if int(max_block_retries) > 0 else 1

    # ---------------- RX helpers ----------------
    def _drain_rx_queue(self) -> None:
        """이전 ACK/노이즈를 버리고 새 요청과 매칭을 단순화."""
        q = getattr(self.mgr, "_rx_q", None)
        if q is None:
            return
        try:
            while True:
                q.get_nowait()
        except Exception:
            pass

    def _raise_if_cmd_failed(self, ack_raw: bytes, stage: str) -> None:
        """Handle optional Status/FailReason payload on command ACKs.

        Some ACKs carry:
          AckID(3B) + Status(1B) + FailReason(1B)

        If those bytes are absent, CAN-FD padding makes them 0x00; we ignore unless status is valid.
        """
        if ack_raw is None or len(ack_raw) < 5:
            return
        status = ack_raw[3]
        reason = ack_raw[4]
        # Only treat as meaningful if status matches spec
        if status not in (int(Status.SUCCESS), int(Status.FAIL)):
            return
        if status == int(Status.SUCCESS):
            return
        # FAIL
        try:
            reason_name = FailReason(reason).name
        except Exception:
            reason_name = f"0x{reason:02X}"
        raise RuntimeError(f"{stage} failed: {reason_name} (0x{reason:02X})")

    def _wait_ack(self, expected_ack_id: int, *, timeout_s: float) -> Optional[bytes]:
        """
        CAN_ID=0x50 ACK 프레임을 기다립니다.
        - timeout_s 내에 expected_ack_id가 오면 payload(64B) 반환
        - 타임아웃이면 None
        """
        q = getattr(self.mgr, "_rx_q", None)
        if q is None:
            raise RuntimeError("PCANManager._rx_q가 없습니다. pcan_manager.py 버전을 확인하세요.")

        deadline = time.time() + timeout_s
        while time.time() < deadline:
            remain = max(0.0, deadline - time.time())
            try:
                payload, can_id = q.get(timeout=remain)
            except Exception:
                return None
            if can_id != int(MsgID.ACK):
                continue
            try:
                ack = parse_ack(payload)
            except Exception:
                continue
            if ack.ack_id == expected_ack_id:
                return payload
        return None

    # ---------------- TX helpers ----------------
    def send_control(self, cmd_payload: bytes) -> None:
        """Control Request Frame: CAN_ID=0x32"""
        self.mgr.send_bytes(int(MsgID.CONTROL), cmd_payload)

    def send_verify(self, cmd_payload: bytes) -> None:
        """VERIFY_IMAGE also uses CONTROL CAN_ID=0x32"""
        self.mgr.send_bytes(int(MsgID.CONTROL), cmd_payload)

    def send_data_customtp(self, data_upper_msg: bytes) -> None:
        """DATA Frame: CustomTP segmentation over CAN_ID=0x33.

        Target-side CustomTP expects the reassembled App-PDU layout:
          [payload_len(4B, big-endian)] + [msg_id(1B)] + [payload] + [padding]

        Here, `payload` is the SWU DATA "upper message":
          CommandID + BlockIdx + ExpectedCRC32 + PayloadLength + ImagePayload
        """
        msg_id = int(MsgID.DATA)  # 0x33
        app_pdu = struct.pack('>I', len(data_upper_msg)) + bytes([msg_id]) + data_upper_msg

        frames = build_frames(app_pdu, start_seq=0)
        for f in frames:
            self.mgr.send_bytes(int(MsgID.DATA), f.payload_64b)

    # ---------------- SWU steps ----------------
    def swu_request(self, model_id: int, sw_version: int, retry: RetryPolicy) -> Tuple[bool, int]:
        """Send SWU_REQUEST and wait for ACK_SWU_REQUEST.

        Returns:
            (True, 0x00) on SUCCESS
            (False, FailReason) on FAIL (e.g. VERSION_MATCH / MODEL_MISMATCH / RX_TIMEOUT ...)
        """
        self._drain_rx_queue()
        payload = build_swu_request(model_id, sw_version, order=self.field_order)

        for _ in range(retry.attempts):
            self.send_control(payload)
            ack_payload = self._wait_ack(int(AckID.ACK_SWU_REQUEST), timeout_s=retry.interval_s)
            if ack_payload is None:
                continue

            ack = parse_ack(ack_payload)
            sr = parse_ack_status_reason(ack)
            if sr is None:
                # Backward compatibility: if status not provided, treat as success.
                return True, int(FailReason.SUCCESS)

            status, reason = sr
            if status == int(Status.SUCCESS):
                return True, int(FailReason.SUCCESS)
            return False, int(reason)

        return False, int(FailReason.RX_TIMEOUT)

    def swu_start(
        self,
        model_id: int,
        sw_version: int,
        date_yyyymmdd: int,
        image_size: int,
        block_size: int,
        retry: RetryPolicy,
    ) -> bool:
        self._drain_rx_queue()
        payload = build_swu_start(model_id, sw_version, date_yyyymmdd, image_size, block_size, self.field_order)

        for _ in range(retry.attempts):
            self.send_control(payload)
            ack = self._wait_ack(int(AckID.ACK_SWU_START), timeout_s=retry.interval_s)
            if ack is not None:
                self._raise_if_cmd_failed(ack, "SWU_START")
                return True
        return False

    def download_start(self, retry: RetryPolicy) -> bool:
        """Send DOWNLOAD_START and wait for ACK_DOWNLOAD_START.

        Safety note:
            DOWNLOAD_START typically triggers a flash erase on the Target.
            Re-sending DOWNLOAD_START can cause accidental duplicate erase
            or unexpected state changes. Therefore, this implementation sends
            DOWNLOAD_START **once** and only extends the ACK wait time
            (retry.interval_s) for safety.
        """
        self._drain_rx_queue()
        payload = build_download_start()

        # Send only once; just wait longer if flash erase takes time.
        self.send_control(payload)
        ack = self._wait_ack(int(AckID.ACK_DOWNLOAD_START), timeout_s=retry.interval_s)
        if ack is None:
            return False
        self._raise_if_cmd_failed(ack, "DOWNLOAD_START")
        return True

    def send_block(
        self,
        block_idx: int,
        expected_crc32: int,
        data: bytes,
        ack_timeout_s: float | None = None,
        max_retries: int | None = None,
    ) -> tuple[bool, int]:
        """Send one SWU DATA block (MSG_ID=0x33) over CustomTP."""
        if block_idx <= 0:
            raise ValueError("block_idx must be 1-based (>0)")
        if not isinstance(data, (bytes, bytearray)):
            raise TypeError("data must be bytes/bytearray")

        data_pdu = build_data_payload(block_idx, expected_crc32, bytes(data), order=self.field_order)

        def _send_once() -> tuple[bool, int]:
            # DATA is transported via CustomTP/IsoTP; this tool uses CustomTP.
            self.send_data_customtp(data_pdu)

            # Wait ACK_DATA (single CAN frame)
            timeout = self.data_ack_timeout_s if ack_timeout_s is None else float(ack_timeout_s)
            ack_payload = self._wait_ack(int(AckID.ACK_DATA), timeout_s=timeout)
            if ack_payload is None:
                return False, int(FailReason.RX_TIMEOUT)

            ack = parse_ack(ack_payload)
            ad = parse_ack_data(ack, order=self.field_order)

            if ad.block_idx != block_idx:
                return False, int(FailReason.UNEXPECTED_BLOCKIDX)

            if ad.status == int(Status.SUCCESS):
                return True, int(FailReason.SUCCESS)

            return False, int(ad.fail_reason)

        retries = self.max_block_retries if max_retries is None else int(max_retries)
        if retries <= 0:
            retries = 1

        last_reason = int(FailReason.SUCCESS)
        for _attempt in range(1, retries + 1):
            ok, reason = _send_once()
            last_reason = int(reason)
            if ok:
                return True, int(FailReason.SUCCESS)

            # retry-able reasons
            if last_reason in (
                int(FailReason.CRC_MISMATCH),
                int(FailReason.RX_TIMEOUT),
                int(FailReason.LENGTH_MISMATCH),
                int(FailReason.UNEXPECTED_BLOCKIDX),
            ):
                continue

            # non retry-able: abort early
            return False, last_reason

        return False, last_reason

    def verify_image(self, expected_crc32: int, retry: RetryPolicy) -> Tuple[bool, int]:
        self._drain_rx_queue()
        payload = build_verify_image(expected_crc32, self.field_order)

        for _ in range(retry.attempts):
            self.send_verify(payload)
            ack_payload = self._wait_ack(int(AckID.ACK_VERIFY), timeout_s=retry.interval_s)
            if ack_payload is None:
                continue
            ack = parse_ack(ack_payload)
            av = parse_ack_verify(ack)
            if av.status == int(Status.SUCCESS):
                return True, int(FailReason.SUCCESS)
            return False, int(av.fail_reason)

        return False, int(FailReason.RX_TIMEOUT)


def compute_crc_table(bin_path: str, block_size: int) -> Tuple[int, list[int], list[int]]:
    """
    Returns (image_size, block_crc_end[1..N], block_len[1..N])
    - block_crc_end[i] = streaming CRC32 from start to end of block i (inclusive)
    """
    size = os.path.getsize(bin_path)
    total_blocks = (size + block_size - 1) // block_size
    crc = 0
    crc_end = [0] * (total_blocks + 1)  # 1-base
    blen = [0] * (total_blocks + 1)

    with open(bin_path, "rb") as f:
        for i in range(1, total_blocks + 1):
            data = f.read(block_size)
            if not data:
                data = b""
            blen[i] = len(data)
            crc = zlib.crc32(data, crc) & 0xFFFFFFFF
            crc_end[i] = crc

    return size, crc_end, blen


def read_block(bin_path: str, block_idx: int, block_size: int, expected_len: int) -> bytes:
    with open(bin_path, "rb") as f:
        f.seek((block_idx - 1) * block_size)
        data = f.read(expected_len)
    if len(data) != expected_len:
        raise RuntimeError(f"read_block length mismatch: idx={block_idx}, got={len(data)}, expected={expected_len}")
    return data





def parse_arguments() -> argparse.Namespace:
    """CLI 인자 파싱

    Returns:
        파싱된 인자 Namespace
    """
    ap = argparse.ArgumentParser(description="SWU PC Tool (CustomTP over CAN FD)")

    # 필수 인자
    ap.add_argument("--bin", required=True, help="Firmware binary path")
    ap.add_argument("--model-id", required=True, type=str, help="Model ID (uint32, ex 0x68440001)")
    ap.add_argument("--date", required=True, type=str, help="Date YYYYMMDD (uint32)")
    ap.add_argument("--block-size", required=True, type=int, help="Transfer Block Size (bytes), ex 65536")

    # 선택 인자 - PCAN 설정
    ap.add_argument("--channel", default=DEFAULT_PCAN_CHANNEL, help=f"PCAN channel (default: {DEFAULT_PCAN_CHANNEL})")
    ap.add_argument("--bitrate-fd", default=DEFAULT_BITRATE_FD, help="PCAN bitrate FD string")
    ap.add_argument("--ifg-us", type=int, default=DEFAULT_IFG_US, help=f"Inter-frame gap us (default {DEFAULT_IFG_US})")

    # 선택 인자 - SWU 설정
    ap.add_argument("--sw-version", required=False, type=str, default=None,
                    help="SW Version (uint32). If omitted, try to parse from filename like _0x00010001.*")
    ap.add_argument("--field-endian", choices=["big", "little"], default="big",
                    help="Endianness for meta fields / BlockIdx / CRC32 (default big). CommandID/AckID are fixed big.")
    ap.add_argument("--ack-data-timeout", type=float, default=TIMEOUT_ACK_DATA_S,
                    help=f"ACK_DATA wait timeout seconds (default {TIMEOUT_ACK_DATA_S}; SBL reassembly timeout is 1.0s)")

    # 선택 인자 - 로깅 설정
    ap.add_argument("--log-file", type=str, default=None, help="Log file path (optional)")
    ap.add_argument("--verbose", "-v", action="store_true", help="Enable verbose (DEBUG) logging")

    # 동작 모드
    ap.add_argument("--dry-run", action="store_true", help="Compute CRC table only, don't send")

    return ap.parse_args()


@dataclass
class SWUToolParams:
    """SWU Tool 실행에 필요한 파라미터"""
    bin_path: str
    model_id: int
    sw_version: int
    date_yyyymmdd: int
    block_size: int
    channel: str
    bitrate_fd: str
    ifg_us: int
    field_order: ByteOrder
    ack_data_timeout_s: float


def validate_and_resolve_params(args: argparse.Namespace) -> Optional[SWUToolParams]:
    """인자 검증 및 버전 추론

    Args:
        args: 파싱된 CLI 인자

    Returns:
        검증된 파라미터 또는 None (오류 시)
    """
    logger = get_logger()

    # 파일 존재 확인
    bin_path = args.bin
    if not os.path.exists(bin_path):
        logger.error(f"파일을 찾을 수 없습니다: {bin_path}")
        return None

    # Model ID 파싱
    try:
        model_id = parse_int_auto(args.model_id)
    except ValueError as e:
        logger.error(f"Model ID 파싱 오류: {e}")
        return None

    # SW Version 파싱 또는 파일명에서 추론
    if args.sw_version is None or str(args.sw_version).strip() == "":
        inferred = extract_sw_version_from_filename(bin_path)
        if inferred:
            logger.info(f"파일명에서 SW Version 추론: {inferred}")
            sw_version = parse_int_auto(inferred)
        else:
            logger.error("--sw-version이 필요합니다 (또는 파일명에 0x########를 포함)")
            return None
    else:
        try:
            sw_version = parse_int_auto(args.sw_version)
        except ValueError as e:
            logger.error(f"SW Version 파싱 오류: {e}")
            return None

    # Date 파싱
    try:
        date_yyyymmdd = parse_int_auto(args.date)
    except ValueError as e:
        logger.error(f"Date 파싱 오류: {e}")
        return None

    # Block size 검증
    block_size = int(args.block_size)
    if block_size <= 0:
        logger.error("block-size는 0보다 커야 합니다")
        return None

    # Endianness
    field_order = ByteOrder.BIG if args.field_endian == "big" else ByteOrder.LITTLE

    return SWUToolParams(
        bin_path=bin_path,
        model_id=model_id,
        sw_version=sw_version,
        date_yyyymmdd=date_yyyymmdd,
        block_size=block_size,
        channel=args.channel,
        bitrate_fd=args.bitrate_fd,
        ifg_us=args.ifg_us,
        field_order=field_order,
        ack_data_timeout_s=float(args.ack_data_timeout),
    )


def run_swu_sequence(params: SWUToolParams) -> int:
    """SWU 프로토콜 실행

    Args:
        params: SWU Tool 파라미터

    Returns:
        종료 코드 (0: 성공, 3: 프로토콜 오류, 4: 데이터 전송 오류, 5: 검증 오류)
    """
    logger = get_logger()

    # CRC 테이블 계산
    image_size, crc_end, blen = compute_crc_table(params.bin_path, params.block_size)
    total_blocks = len(crc_end) - 1
    total_crc = crc_end[total_blocks] if total_blocks > 0 else 0

    logger.info(f"ImageSize={image_size} bytes, BlockSize={params.block_size}, TotalBlocks={total_blocks}")
    logger.info(f"CRC32(whole image)=0x{total_crc:08X} (CRC-32/ISO-HDLC)")

    # PCAN 연결
    mgr = PCANManager()
    mgr.open(channel=params.channel, bitrate_fd=params.bitrate_fd, ifg_us=params.ifg_us, is_std=True, use_brs=True)
    mgr.start_rx()

    tool = SWUTool(
        mgr,
        field_order=params.field_order,
        data_ack_timeout_s=params.ack_data_timeout_s,
        max_block_retries=RETRY_DATA_BLOCK_ATTEMPTS
    )

    # 재시도 정책 (상수 사용)
    rp_request = RetryPolicy(attempts=RETRY_SWU_REQUEST_ATTEMPTS, interval_s=RETRY_SWU_REQUEST_INTERVAL_S)
    rp_ctrl = RetryPolicy(attempts=RETRY_CTRL_ATTEMPTS, interval_s=RETRY_CTRL_INTERVAL_S)
    rp_download_start = RetryPolicy(attempts=RETRY_DOWNLOAD_START_ATTEMPTS, interval_s=RETRY_DOWNLOAD_START_INTERVAL_S)
    rp_verify = RetryPolicy(attempts=RETRY_VERIFY_ATTEMPTS, interval_s=RETRY_VERIFY_INTERVAL_S)

    try:
        # Phase 1: SWU_REQUEST
        logger.info("SWU_REQUEST ...")
        ok, reason = tool.swu_request(params.model_id, params.sw_version, rp_request)
        if not ok:
            if reason == int(FailReason.VERSION_MATCH):
                logger.info("SWU 미수행: VERSION_MATCH (이미 최신 버전)")
                return 0
            logger.error(f"SWU_REQUEST 거절: {format_fail_reason(reason)}")
            return 3
        logger.info("ACK_SWU_REQUEST 수신")

        # Phase 2: SWU_START
        logger.info("SWU_START ...")
        if not tool.swu_start(params.model_id, params.sw_version, params.date_yyyymmdd, image_size, params.block_size, rp_ctrl):
            logger.error("SWU_START 타임아웃 (ACK_SWU_START 미수신)")
            return 3
        logger.info("ACK_SWU_START 수신")

        # Phase 3: DOWNLOAD_START
        logger.info("DOWNLOAD_START ...")
        if not tool.download_start(rp_download_start):
            logger.error("DOWNLOAD_START 타임아웃 (ACK_DOWNLOAD_START 미수신)")
            return 3
        logger.info("ACK_DOWNLOAD_START 수신")

        # Phase 4: DATA blocks
        logger.info("DATA 블록 전송 시작...")
        t0 = time.time()
        for idx in range(1, total_blocks + 1):
            expected_crc = crc_end[idx]
            expected_len = blen[idx]
            data = read_block(params.bin_path, idx, params.block_size, expected_len)

            ok, reason = tool.send_block(
                idx,
                expected_crc,
                data,
                ack_timeout_s=params.ack_data_timeout_s,
                max_retries=RETRY_DATA_BLOCK_ATTEMPTS,
            )
            if not ok:
                logger.error(f"Block {idx}/{total_blocks} 전송 실패: {format_fail_reason(reason)}")
                return 4

            # 진행률 표시
            if idx == 1 or idx == total_blocks or (idx % 10 == 0):
                elapsed = time.time() - t0
                sent = min(idx * params.block_size, image_size)
                pct = (sent / image_size * 100.0) if image_size else 100.0
                logger.info(f"Block {idx}/{total_blocks} OK | {pct:.1f}% | elapsed {elapsed:.1f}s")

        # Phase 5: VERIFY_IMAGE
        logger.info("VERIFY_IMAGE ...")
        ok, reason = tool.verify_image(total_crc, rp_verify)
        if not ok:
            logger.error(f"VERIFY_IMAGE 실패: {format_fail_reason(reason)}")
            return 5
        logger.info("ACK_VERIFY_SUCCESS 수신")
        logger.info("SWU 완료. 타겟이 리셋/앱 점프합니다.")
        return 0

    finally:
        try:
            mgr.stop_rx()
        except Exception:
            pass
        try:
            mgr.close()
        except Exception:
            pass


def main() -> int:
    """CLI 메인 함수"""
    # CLI 인자 파싱
    args = parse_arguments()

    # 로깅 설정
    setup_logging(log_file=args.log_file, verbose=args.verbose)
    logger = get_logger()

    # 파라미터 검증
    params = validate_and_resolve_params(args)
    if params is None:
        return 2

    # Dry-run 모드
    if args.dry_run:
        image_size, crc_end, blen = compute_crc_table(params.bin_path, params.block_size)
        total_blocks = len(crc_end) - 1
        total_crc = crc_end[total_blocks] if total_blocks > 0 else 0
        logger.info(f"ImageSize={image_size} bytes, BlockSize={params.block_size}, TotalBlocks={total_blocks}")
        logger.info(f"CRC32(whole image)=0x{total_crc:08X} (CRC-32/ISO-HDLC)")
        return 0

    # SWU 실행
    return run_swu_sequence(params)


if __name__ == "__main__":
    raise SystemExit(main())
