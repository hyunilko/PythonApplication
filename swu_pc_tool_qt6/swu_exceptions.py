#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
swu_exceptions.py

SWU PC Tool 커스텀 예외 클래스
구조화된 에러 처리를 위한 예외 계층
"""

from __future__ import annotations

from typing import Optional

from swu_protocol import FailReason


class SWUError(Exception):
    """SWU 작업 기본 예외 클래스"""

    def __init__(self, message: str, fail_reason: Optional[int] = None):
        super().__init__(message)
        self.fail_reason = fail_reason

    @property
    def fail_reason_name(self) -> str:
        """FailReason 코드를 이름으로 변환"""
        if self.fail_reason is None:
            return "UNKNOWN"
        try:
            return FailReason(self.fail_reason).name
        except ValueError:
            return f"0x{self.fail_reason:02X}"


class SWUTimeoutError(SWUError):
    """ACK 타임아웃 예외

    지정된 시간 내에 ACK 프레임을 수신하지 못한 경우
    """

    def __init__(self, stage: str, timeout_s: float):
        message = f"{stage} 타임아웃: {timeout_s}초 내에 ACK 미수신"
        super().__init__(message, int(FailReason.RX_TIMEOUT))
        self.stage = stage
        self.timeout_s = timeout_s


class SWUProtocolError(SWUError):
    """프로토콜 에러 예외

    프로토콜 규격에 맞지 않는 응답을 수신한 경우
    """

    def __init__(self, message: str, fail_reason: int):
        super().__init__(message, fail_reason)


class SWUVersionMatchError(SWUError):
    """버전 일치 예외 (업데이트 불필요)

    타겟 디바이스에 이미 동일 또는 최신 버전이 설치된 경우
    """

    def __init__(self, current_version: Optional[int] = None, requested_version: Optional[int] = None):
        message = "이미 동일/최신 SW Version입니다. 업데이트가 필요 없습니다."
        if current_version is not None and requested_version is not None:
            message = f"버전 일치: 현재=0x{current_version:08X}, 요청=0x{requested_version:08X}"
        super().__init__(message, int(FailReason.VERSION_MATCH))
        self.current_version = current_version
        self.requested_version = requested_version


class SWUModelMismatchError(SWUError):
    """모델 불일치 예외

    타겟 디바이스의 Model ID가 요청된 Model ID와 다른 경우
    """

    def __init__(self, device_model: Optional[int] = None, requested_model: Optional[int] = None):
        message = "Model ID가 일치하지 않습니다."
        if device_model is not None and requested_model is not None:
            message = f"모델 불일치: 디바이스=0x{device_model:08X}, 요청=0x{requested_model:08X}"
        super().__init__(message, int(FailReason.MODEL_MISMATCH))
        self.device_model = device_model
        self.requested_model = requested_model


class SWUCancelledError(SWUError):
    """사용자 취소 예외

    사용자가 SWU 작업을 취소한 경우
    """

    def __init__(self, stage: Optional[str] = None):
        message = "사용자에 의해 중단되었습니다."
        if stage:
            message = f"{stage} 단계에서 사용자에 의해 중단되었습니다."
        super().__init__(message, None)
        self.stage = stage


class SWUBlockTransferError(SWUError):
    """블록 전송 실패 예외

    데이터 블록 전송 중 오류가 발생한 경우
    """

    def __init__(self, block_idx: int, total_blocks: int, fail_reason: int):
        from swu_utils import format_fail_reason
        reason_str = format_fail_reason(fail_reason)
        message = f"블록 {block_idx}/{total_blocks} 전송 실패: {reason_str}"
        super().__init__(message, fail_reason)
        self.block_idx = block_idx
        self.total_blocks = total_blocks


class SWUVerifyError(SWUError):
    """이미지 검증 실패 예외

    VERIFY_IMAGE 단계에서 CRC 불일치 등의 오류가 발생한 경우
    """

    def __init__(self, expected_crc: int, fail_reason: int):
        from swu_utils import format_fail_reason
        reason_str = format_fail_reason(fail_reason)
        message = f"이미지 검증 실패 (예상 CRC=0x{expected_crc:08X}): {reason_str}"
        super().__init__(message, fail_reason)
        self.expected_crc = expected_crc


class PCANError(SWUError):
    """PCAN 하드웨어 에러

    PCAN 어댑터 초기화, 송수신 중 오류가 발생한 경우
    """

    def __init__(self, message: str, pcan_status: Optional[int] = None):
        if pcan_status is not None:
            message = f"{message} (PCAN 상태: 0x{pcan_status:X})"
        super().__init__(message, None)
        self.pcan_status = pcan_status


class PCANInitError(PCANError):
    """PCAN 초기화 실패"""

    def __init__(self, channel: str, pcan_status: int):
        message = f"PCAN 초기화 실패: 채널 {channel}"
        super().__init__(message, pcan_status)
        self.channel = channel


class PCANWriteError(PCANError):
    """PCAN 전송 실패"""

    def __init__(self, can_id: int, pcan_status: int):
        message = f"PCAN 전송 실패: CAN_ID=0x{can_id:02X}"
        super().__init__(message, pcan_status)
        self.can_id = can_id


class PCANReadError(PCANError):
    """PCAN 수신 실패"""

    def __init__(self, pcan_status: int):
        message = "PCAN 수신 실패"
        super().__init__(message, pcan_status)
