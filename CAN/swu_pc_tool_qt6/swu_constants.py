  #!/usr/bin/env python3
  # -*- coding: utf-8 -*-
"""
swu_constants.py

SWU PC Tool 전역 상수 정의
모든 매직 넘버와 기본값을 중앙 집중화
"""

from __future__ import annotations

# ==============================================================================
# CAN ID 상수
# ==============================================================================
CAN_ID_CONTROL = 0x32   # Control/Verify 프레임
CAN_ID_DATA = 0x33      # Data 프레임 (CustomTP)
CAN_ID_ACK = 0x50       # ACK 프레임


# ==============================================================================
# 타임아웃 값 (초)
# ==============================================================================
# ACK 대기 타임아웃
TIMEOUT_ACK_DEFAULT_S = 0.2
TIMEOUT_ACK_DATA_S = 2.0
TIMEOUT_ACK_DOWNLOAD_START_S = 60.0  # Flash erase 대기 시간
TIMEOUT_ACK_VERIFY_S = 0.2

# SWU_REQUEST 타임아웃
TIMEOUT_SWU_REQUEST_INTERVAL_S = 0.1


# ==============================================================================
# 재시도 정책 기본값
# ==============================================================================
# SWU_REQUEST: 100ms 간격으로 50회 재시도 (총 5초)
RETRY_SWU_REQUEST_ATTEMPTS = 50
RETRY_SWU_REQUEST_INTERVAL_S = 0.1

# 일반 제어 명령: 100ms 간격으로 3회 재시도
RETRY_CTRL_ATTEMPTS = 3
RETRY_CTRL_INTERVAL_S = 0.1

# DOWNLOAD_START: 1회만 전송, 긴 대기 (flash erase)
RETRY_DOWNLOAD_START_ATTEMPTS = 1
RETRY_DOWNLOAD_START_INTERVAL_S = 10.0

# VERIFY_IMAGE: 200ms 간격으로 3회 재시도
RETRY_VERIFY_ATTEMPTS = 3
RETRY_VERIFY_INTERVAL_S = 0.2

# DATA 블록 전송 재시도
RETRY_DATA_BLOCK_ATTEMPTS = 3


# ==============================================================================
# PCAN 기본 설정
# ==============================================================================
DEFAULT_PCAN_CHANNEL = "PCAN_USBBUS1"
DEFAULT_BITRATE_FD = (
    "f_clock=80000000,nom_brp=2,nom_tseg1=33,nom_tseg2=6,nom_sjw=1,"
    "data_brp=2,data_tseg1=6,data_tseg2=1,data_sjw=1,data_ssp_offset=14"
)
DEFAULT_IFG_US = 1500  # 인터프레임 갭 (마이크로초)

# PCAN 수신 폴링 타임아웃 (밀리초)
PCAN_READ_TIMEOUT_MS = 10

# PCAN 전송 재시도 횟수
PCAN_WRITE_MAX_RETRY = 100


# ==============================================================================
# GUI 진행률 상수 (%)
# ==============================================================================
PROGRESS_START = 0
PROGRESS_ACK_SWU_START = 5
PROGRESS_ACK_DOWNLOAD_START = 20
PROGRESS_DATA_START = 20
PROGRESS_DATA_END = 95
PROGRESS_ACK_VERIFY = 100


# ==============================================================================
# 블록 크기 제한값
# ==============================================================================
BLOCK_SIZE_MIN = 256
BLOCK_SIZE_MAX = 4 * 1024 * 1024  # 4MB
BLOCK_SIZE_DEFAULT = 65536        # 64KB

# CAN FD 최대 페이로드
CANFD_MAX_PAYLOAD = 64


# ==============================================================================
# CustomTP 상수
# ==============================================================================
CUSTOMTP_HDR_SIZE = 2
CUSTOMTP_CHUNK_SIZE = CANFD_MAX_PAYLOAD - CUSTOMTP_HDR_SIZE  # 62 bytes
CUSTOMTP_SEQ_MAX = 0x3FFF
