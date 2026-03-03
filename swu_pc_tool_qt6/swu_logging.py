#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
swu_logging.py

SWU PC Tool 로깅 인프라
Python logging 모듈 통합
"""

from __future__ import annotations

import logging
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional, Callable

# 로거 이름 상수
LOGGER_NAME = "swu"


def setup_logging(
    *,
    log_file: Optional[str] = None,
    verbose: bool = False,
    console: bool = True,
    log_format: Optional[str] = None,
) -> logging.Logger:
    """로깅 설정 초기화

    Args:
        log_file: 로그 파일 경로 (None이면 파일 로깅 비활성화)
        verbose: True면 DEBUG 레벨, False면 INFO 레벨
        console: True면 콘솔 출력 활성화
        log_format: 커스텀 로그 포맷 (None이면 기본값 사용)

    Returns:
        설정된 로거 인스턴스
    """
    logger = logging.getLogger(LOGGER_NAME)

    # 기존 핸들러 제거 (중복 방지)
    logger.handlers.clear()

    # 로그 레벨 설정
    level = logging.DEBUG if verbose else logging.INFO
    logger.setLevel(level)

    # 로그 포맷
    if log_format is None:
        log_format = "[%(asctime)s] %(levelname)-8s %(message)s"

    formatter = logging.Formatter(log_format, datefmt="%Y-%m-%d %H:%M:%S")

    # 콘솔 핸들러
    if console:
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(level)
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)

    # 파일 핸들러
    if log_file:
        try:
            file_handler = logging.FileHandler(log_file, encoding="utf-8")
            file_handler.setLevel(level)
            file_handler.setFormatter(formatter)
            logger.addHandler(file_handler)
        except OSError as e:
            logger.warning(f"로그 파일 생성 실패: {log_file} ({e})")

    return logger


def get_logger() -> logging.Logger:
    """로거 인스턴스 반환

    Returns:
        SWU 로거 인스턴스
    """
    return logging.getLogger(LOGGER_NAME)


def generate_log_filename(prefix: str = "swu", ext: str = "log") -> str:
    """타임스탬프 기반 로그 파일명 생성

    Args:
        prefix: 파일명 접두사
        ext: 파일 확장자

    Returns:
        생성된 파일명 (예: swu_20260204_153045.log)
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{prefix}_{timestamp}.{ext}"


class GUILogHandler(logging.Handler):
    """Qt 시그널 연동 로그 핸들러

    PyQt6 GUI의 시그널을 통해 로그 메시지를 전달합니다.

    Usage:
        handler = GUILogHandler(log_signal.emit)
        logger.addHandler(handler)
    """

    def __init__(self, emit_callback: Callable[[str], None], level: int = logging.NOTSET):
        """
        Args:
            emit_callback: 로그 메시지를 전달할 콜백 함수 (예: pyqtSignal.emit)
            level: 로그 레벨
        """
        super().__init__(level)
        self._emit = emit_callback

        # GUI용 간단한 포맷
        self.setFormatter(logging.Formatter("[%(levelname)s] %(message)s"))

    def emit(self, record: logging.LogRecord) -> None:
        """로그 레코드를 콜백으로 전달"""
        try:
            message = self.format(record)
            self._emit(message)
        except Exception:
            # 콜백 오류가 로깅 시스템을 중단시키지 않도록 함
            self.handleError(record)


class ProgressLogHandler(logging.Handler):
    """진행률 추적 로그 핸들러

    특정 패턴의 로그 메시지를 분석하여 진행률을 추출합니다.

    Usage:
        handler = ProgressLogHandler(progress_signal.emit)
        logger.addHandler(handler)
    """

    def __init__(self, progress_callback: Callable[[int], None], level: int = logging.NOTSET):
        """
        Args:
            progress_callback: 진행률(0-100)을 전달할 콜백 함수
            level: 로그 레벨
        """
        super().__init__(level)
        self._progress = progress_callback

    def emit(self, record: logging.LogRecord) -> None:
        """진행률 정보가 있는 로그 레코드 처리"""
        try:
            # 로그 레코드에 progress 속성이 있으면 콜백 호출
            progress = getattr(record, "progress", None)
            if progress is not None and isinstance(progress, int):
                self._progress(progress)
        except Exception:
            self.handleError(record)


def log_with_progress(logger: logging.Logger, level: int, message: str, progress: int) -> None:
    """진행률 정보와 함께 로그 출력

    Args:
        logger: 로거 인스턴스
        level: 로그 레벨
        message: 로그 메시지
        progress: 진행률 (0-100)
    """
    extra = {"progress": progress}
    logger.log(level, message, extra=extra)
