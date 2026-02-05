  #!/usr/bin/env python3
  # -*- coding: utf-8 -*-
"""
swu_utils.py

SWU PC Tool 공통 유틸리티 함수
중복 함수 통합
"""

from __future__ import annotations

import os
import re
import time
from typing import Optional

from swu_protocol import FailReason


def now_ms() -> int:
    """현재 시간을 밀리초로 반환"""
    return int(time.time() * 1000)


def parse_int_auto(s: str) -> int:
    """16진수(0x...) 또는 10진수 문자열을 자동 파싱하여 정수 반환

    Args:
        s: 파싱할 문자열 (예: "0x68440001", "123")

    Returns:
        파싱된 정수

    Raises:
        ValueError: 유효하지 않은 숫자 형식
    """
    s = (s or "").strip()
    if s.lower().startswith("0x"):
        return int(s, 16)
    return int(s, 10)


def extract_sw_version_from_filename(file_path: str) -> Optional[str]:
    """파일명에서 SW Version 추출

    파일명에 포함된 '0x00010001' 형태(8-hex)를 SW Version으로 추출.
    예) filename_0x00010001.release.appimage -> "0x00010001"
        filename_0x00010001.debug.appimage -> "0x00010001"

    Args:
        file_path: 펌웨어 파일 경로

    Returns:
        SW Version 문자열 또는 None (찾지 못한 경우)
    """
    if not file_path:
        return None
    name = os.path.basename(file_path)
    m = re.search(r"(0x[0-9a-fA-F]{8})", name)
    return m.group(1) if m else None


def format_fail_reason(code: int) -> str:
    """FailReason 코드를 사람이 읽을 수 있는 문자열로 변환

    Args:
        code: FailReason 코드

    Returns:
        포맷된 문자열 (예: "CRC_MISMATCH(0x01)")
    """
    try:
        fr = FailReason(code)
        return f"{fr.name}(0x{code:02X})"
    except ValueError:
        return f"UNKNOWN(0x{code:02X})"


def compute_block_count(image_size: int, block_size: int) -> int:
    """이미지 크기와 블록 크기로 총 블록 수 계산

    Args:
        image_size: 이미지 크기 (바이트)
        block_size: 블록 크기 (바이트)

    Returns:
        총 블록 수
    """
    if block_size <= 0:
        return 0
    return (image_size + block_size - 1) // block_size


def validate_block_size(block_size: int, min_size: int = 256, max_size: int = 4 * 1024 * 1024) -> bool:
    """블록 크기 유효성 검증

    Args:
        block_size: 검증할 블록 크기
        min_size: 최소 블록 크기 (기본: 256)
        max_size: 최대 블록 크기 (기본: 4MB)

    Returns:
        유효하면 True
    """
    return min_size <= block_size <= max_size
