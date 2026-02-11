#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
파일명: ft4232h_cl_usb_gui.py
설명: AWRL6844 레이더 센서에서 SPI 통신으로 데이터를 캡처하는 GUI 도구
================================================================================

【프로그램 개요 - Python 초보자를 위한 설명】

이 프로그램은 무엇을 하나요?
- 레이더 센서(AWRL6844)에서 나오는 데이터를 컴퓨터로 가져옵니다
- FT4232H라는 USB 장치를 통해 SPI 통신을 합니다
- GUI(그래픽 사용자 인터페이스)로 쉽게 조작할 수 있습니다

주요 용어 설명:
- SPI: Serial Peripheral Interface, 장치들이 데이터를 주고받는 통신 방식
- FTDI: USB를 다른 통신 방식으로 변환해주는 칩을 만드는 회사
- FT4232H: FTDI사의 USB-SPI 변환 칩
- MPSSE: FT4232H 칩의 고속 통신 모드
- GPIO: General Purpose Input/Output, 범용 입출력 핀
- HOST_INTR: 센서가 "데이터 준비됐어요"라고 알려주는 신호

데이터 흐름:
1. 레이더 센서가 데이터를 준비합니다
2. 센서가 HOST_INTR 신호를 LOW로 만들어 "준비됐다"고 알립니다
3. 이 프로그램이 SPI로 데이터를 읽어옵니다
4. 읽은 데이터를 파일로 저장합니다

펌웨어(main_full_mss.c) 동작 방식:
- 한 프레임 = adcDataPerFrame 바이트 (기본값 131,072바이트 = 128KB)
- 한 번에 65,536바이트(64KB)씩 chunk 단위로 전송
- 각 chunk 전송 시:
    1) HOST_INTR을 LOW로 설정 (데이터 준비됨 신호)
    2) FW가 MCSPI_transfer() 호출하여 SPI 전송 수행
    3) HOST_INTR을 HIGH로 설정 (전송 완료)
- 프레임 헤더 구조:
    byte[0..3] = 프레임 카운터 (big-endian 형식의 32비트 숫자)
    byte[4..] = (i-4) & 0xFF (테스트 패턴: 0, 1, 2, 3, ...)

호스트(이 프로그램) 동작 방식:
- 각 chunk 전에 HOST_INTR이 LOW가 될 때까지 대기
- LOW가 되면 정확히 해당 chunk 크기만큼 SPI로 읽기
- FW가 uint32_t* (32비트 단위)로 MSB-first 전송하므로 바이트 순서가 뒤집힘
  -> byteswap32로 원래 바이트 순서 복원 (00 00 01 00 00 01 02 03 ...)

주요 수정 사항:
1) spi_read_exact(): 부분 읽기로 인한 정렬 오류 방지
2) resync_on_start: 시작 시 chunk0을 찾을 때까지 버리고 동기화
3) HOST_INTR은 항상 사용자 선택 마스크 사용 (하드코딩 없음)
================================================================================
"""

# ============================================================================
# 라이브러리 임포트 (Import) 섹션
# ============================================================================
# Python에서 다른 사람이 만든 코드를 가져다 쓰려면 import를 사용합니다.
# 마치 레고 블록을 가져다 쓰는 것과 같습니다.

import sys      # 시스템 관련 기능 (프로그램 종료 등)
import time     # 시간 관련 기능 (대기, 시간 측정 등)
import os       # 운영체제 기능 (폴더 생성, 파일 경로 등)
import glob     # 파일 패턴 검색 (*.bin 같은 파일 찾기)
import array    # 배열 처리 (바이트 스왑에 사용)
from datetime import datetime  # 날짜/시간 처리 (로그에 타임스탬프 찍기)
from dataclasses import dataclass  # 데이터 클래스 (설정값 묶어서 관리)

# ftd2xx: FTDI 칩을 제어하는 라이브러리
# 설치 방법: pip install ftd2xx
try:
    import ftd2xx as ftd  # ftd2xx 라이브러리를 ftd라는 짧은 이름으로 사용
except ImportError:
    # ImportError: 라이브러리가 설치되어 있지 않을 때 발생하는 에러
    print("ERROR: ftd2xx library not found. Install with: pip install ftd2xx")
    sys.exit(1)  # 프로그램 종료 (1 = 에러로 종료)

# PyQt6: GUI(그래픽 사용자 인터페이스)를 만드는 라이브러리
# 버튼, 텍스트 입력창, 체크박스 등을 만들 수 있습니다
from PyQt6.QtCore import QThread, pyqtSignal, QObject, pyqtSlot
# QThread: 별도의 작업 스레드 (메인 화면이 멈추지 않게 함)
# pyqtSignal: 스레드 간 통신용 신호
# QObject: Qt 객체의 기본 클래스
# pyqtSlot: 신호를 받는 함수 표시

from PyQt6.QtWidgets import (
    QApplication,    # Qt 애플리케이션 객체
    QMainWindow,     # 메인 윈도우 (프로그램 창)
    QWidget,         # 기본 위젯 (UI 요소의 부모)
    QVBoxLayout,     # 세로 배치 레이아웃
    QHBoxLayout,     # 가로 배치 레이아웃
    QLabel,          # 텍스트 라벨
    QPushButton,     # 버튼
    QComboBox,       # 드롭다운 선택 박스
    QPlainTextEdit,  # 여러 줄 텍스트 편집기 (로그 표시용)
    QLineEdit,       # 한 줄 텍스트 입력
    QCheckBox,       # 체크박스
    QMessageBox,     # 메시지 팝업 창
    QGroupBox,       # 그룹 박스 (관련 항목 묶기)
    QFileDialog,     # 파일/폴더 선택 대화상자
    QSpinBox,        # 숫자 입력 (화살표로 증감)
    QProgressBar     # 진행 상황 바
)

# ============================================================================
# FTDI MPSSE 설정 상수 (Configuration Constants)
# ============================================================================
# MPSSE = Multi-Protocol Synchronous Serial Engine
# FT4232H 칩이 SPI, I2C, JTAG 등 다양한 통신을 할 수 있게 해주는 모드입니다.
# 아래 상수들은 FTDI 칩에 보내는 "명령어"입니다.
# 참고 문서: AN108 (FTDI Application Note)
#
# b"\x8A" 형식 설명:
# - b"..." : 바이트 문자열 (byte string)
# - \x8A   : 16진수 8A를 바이트로 표현 (10진수로 138)

# --- 클럭(Clock) 설정 ---
# 클럭: 디지털 회로의 "심장박동" 같은 것. 이 신호에 맞춰 데이터가 전송됩니다.
FTDI_CFG_60MHZ_SYSCLK = b"\x8A"   # 시스템 클럭을 60MHz로 설정 (5분주 비활성화)
                                  # 분주(Divide): 클럭 속도를 나누는 것
                                  # 5분주 비활성화 = 60MHz 그대로 사용

FTDI_CFG_NO_ADAPT_CLK = b"\x97"   # 적응형 클럭킹 끄기
                                  # 적응형: 상대방 속도에 맞추는 기능 (여기선 불필요)

FTDI_CFG_3PHAS_CLK    = b"\x8C"   # 3상(3-Phase) 클럭킹 켜기
                                  # 고속(30MHz)에서 데이터 안정성 향상
                                  # 3상: 클럭의 상승/유지/하강 3단계로 샘플링

FTDI_CFG_NO_3PHAS_CLK = b"\x8D"   # 3상 클럭킹 끄기 (저속에서는 불필요)

# --- 기타 설정 ---
FTDI_CFG_NO_LOOPBACK  = b"\x85"   # 루프백 비활성화
                                  # 루프백: 보낸 데이터가 다시 돌아오는 테스트 모드
                                  # 실제 통신에서는 꺼야 함

FTDI_CFG_SPI_4PIN_CFG = b"\x80\x08\x0B"  # SPI 4핀 설정
                                         # \x80 = GPIO 명령
                                         # \x08 = 출력값 (CS를 HIGH로)
                                         # \x0B = 방향 (SCK, MOSI, CS는 출력)
                                         # 4핀: SCK(클럭), MOSI(출력), MISO(입력), CS(선택)

# --- MPSSE 명령어 ---
# SPI 통신을 제어하는 명령어들

FTDI_CMD_CS_LOW     = b"\x80\x00\x0B"  # CS(Chip Select)를 LOW로
                                       # CS LOW = "이제 너랑 통신할거야"
                                       # \x80 = GPIO 명령, \x00 = 모든 출력 LOW
                                       # \x0B = 방향 설정

FTDI_CMD_CS_HIGH    = b"\x80\x08\x0B"  # CS를 HIGH로
                                       # CS HIGH = "통신 끝났어"
                                       # \x08 = CS핀만 HIGH

FTDI_CMD_READ_BYTES = b"\x20"          # SPI로 바이트 읽기 명령
                                        # +ve edge = 클럭이 올라갈 때 읽기
                                        # MSB first = 높은 비트부터 읽기

FTDI_CMD_READ_GPIO  = b"\x81"          # GPIO 핀 상태 읽기 명령
                                        # HOST_INTR 신호 확인에 사용

# --- 청크 크기 ---
FTDI_MAX_CHUNK = 65536  # 한 번에 읽을 수 있는 최대 바이트 수 (64KB)
                        # FTDI MPSSE의 제한이며, 펌웨어도 이 크기로 전송


# ============================================================================
# 저수준 헬퍼 함수 (Low-level Helper Functions)
# ============================================================================
# 이 함수들은 FTDI 칩과 직접 통신하는 기본 기능들입니다.
# 마치 자동차의 엔진과 바퀴 같은 핵심 부품입니다.


def set_clk(handle, hz: int) -> int:
    """
    ═══════════════════════════════════════════════════════════════════════════
    함수명: set_clk (set clock의 약자)
    목적: SPI 통신 속도(클럭)를 설정합니다
    ═══════════════════════════════════════════════════════════════════════════

    【개념 설명】
    클럭(Clock)이란?
    - 디지털 통신의 "박자"입니다
    - 1초에 몇 번 데이터를 주고받을지 결정합니다
    - Hz(헤르츠) = 1초당 횟수 (30MHz = 1초에 3천만 번)

    FT4232H의 클럭 계산 공식:
        실제_클럭 = 60MHz ÷ (2 × (분주값 + 1))

    가능한 클럭 속도 예시:
        분주값=0: 60÷(2×1) = 30MHz (최대)
        분주값=1: 60÷(2×2) = 15MHz
        분주값=2: 60÷(2×3) = 10MHz
        분주값=4: 60÷(2×5) = 6MHz

    【매개변수 (Parameters)】
    - handle: FTDI 장치 핸들 (열린 장치에 대한 참조)
    - hz: 원하는 클럭 속도 (단위: Hz)

    【반환값 (Returns)】
    - int: 실제로 설정된 클럭 속도 (Hz)
           (원하는 속도가 정확히 안 되면 가장 가까운 값)

    【예외 (Exceptions)】
    - ValueError: 30MHz를 초과하면 에러 발생
    ═══════════════════════════════════════════════════════════════════════════
    """

    # --- 1단계: 입력값 검증 ---
    if hz > 30_000_000:  # 30_000_000 = 30,000,000 (Python에서 _로 천 단위 구분 가능)
        raise ValueError("Max SCK rate is 30MHz")
        # raise: 에러를 발생시키는 명령어
        # ValueError: 값이 잘못되었다는 종류의 에러

    # --- 2단계: 분주값(divisor) 계산 ---
    # 목표: 요청한 속도 이하의 가장 빠른 속도를 찾기
    # 공식: div = ceil(60MHz / (2 * hz)) - 1
    # ceil = 올림 함수 (2.1 -> 3)
    # 아래는 정수 연산으로 올림을 구현한 것:
    # (a + b - 1) // b 는 a를 b로 나눈 올림값
    div = max(0, (60_000_000 + 2 * hz - 1) // (2 * hz) - 1)
    # max(0, x): x가 음수면 0을 사용 (분주값은 0 이상이어야 함)
    # // : 정수 나눗셈 (소수점 버림)

    # --- 3단계: 실제 클럭 계산 ---
    actual_hz = 60_000_000 // (2 * (div + 1))
    # 예: div=0이면, 60,000,000 // 2 = 30,000,000 Hz = 30MHz

    # --- 4단계: FTDI 칩에 명령 전송 ---
    # 0x86 = 클럭 분주값 설정 명령어
    # 분주값은 2바이트(16비트)로 전송: [하위 바이트, 상위 바이트]
    cmd = bytes((0x86, div & 0xFF, (div >> 8) & 0xFF))
    # div & 0xFF: 하위 8비트 추출 (예: 0x1234 -> 0x34)
    # (div >> 8) & 0xFF: 상위 8비트 추출 (예: 0x1234 -> 0x12)
    # >> : 비트 오른쪽 시프트 (8칸 이동 = 256으로 나눔)
    # & : 비트 AND 연산

    handle.write(cmd)  # FTDI 장치에 명령어 전송

    return actual_hz  # 실제 설정된 클럭 속도 반환


def set_device(handle, clk_speed: int = 15_000_000, latency_timer: int = 1, rw_timeout_ms: int = 5000,
                use_3phase_clk: bool = False) -> int:
    """
    ═══════════════════════════════════════════════════════════════════════════
    함수명: set_device
    목적: FTDI 장치를 MPSSE SPI 모드로 초기화합니다
    ═══════════════════════════════════════════════════════════════════════════

    【개념 설명】
    초기화(Initialize)란?
    - 장치를 사용하기 전에 필요한 설정을 하는 것
    - 마치 자동차 시동 걸기 전에 거울 조정, 안전벨트 착용하는 것과 같습니다

    이 함수가 하는 일:
    1. 장치 리셋 (이전 상태 지우기)
    2. USB 통신 설정
    3. MPSSE 모드 활성화
    4. 클럭 및 핀 설정

    【매개변수 (Parameters)】
    - handle: FTDI 장치 핸들 (ftd.open()으로 얻은 객체)
    - clk_speed: SPI 클럭 속도 (기본값: 15MHz)
    - latency_timer: USB 지연 시간 (ms, 기본값: 1)
                     작을수록 응답 빠름, 너무 작으면 CPU 부하 증가
    - rw_timeout_ms: 읽기/쓰기 타임아웃 (ms, 기본값: 5000 = 5초)
    - use_3phase_clk: 3상 클럭킹 사용 여부 (기본값: False)
                      30MHz에서 안정성을 위해 True 권장

    【반환값 (Returns)】
    - int: 실제 설정된 SPI 클럭 속도 (Hz)
    ═══════════════════════════════════════════════════════════════════════════
    """

    # --- 1단계: 장치 리셋 ---
    handle.resetDevice()
    # 이전 설정을 모두 지우고 초기 상태로 되돌림

    # --- 2단계: 남아있는 데이터 제거 ---
    # 이전 실행에서 남은 데이터가 있으면 읽어서 버림
    try:
        rx_bytes, tx_bytes, event_status = handle.getStatus()
        # rx_bytes: 수신 버퍼에 있는 바이트 수
        # tx_bytes: 송신 버퍼에 있는 바이트 수
        # event_status: 이벤트 상태

        if rx_bytes and rx_bytes > 0:
            handle.read(rx_bytes)  # 남은 데이터 읽어서 버림
    except Exception:
        pass  # 에러 무시 (없어도 됨)

    # --- 3단계: USB 통신 파라미터 설정 ---
    handle.setUSBParameters(65535, 65535)
    # USB 버퍼 크기 설정 (입력, 출력 각각 최대값)
    # 큰 버퍼 = 더 많은 데이터를 한 번에 전송 가능

    handle.setChars(False, 0, False, 0)
    # 특수 문자 처리 설정 (여기서는 사용 안 함)
    # (이벤트 문자, 에러 문자 비활성화)

    handle.setTimeouts(rw_timeout_ms, rw_timeout_ms)
    # 타임아웃 설정: 5초 내에 응답 없으면 에러
    # (읽기 타임아웃, 쓰기 타임아웃)

    handle.setLatencyTimer(latency_timer)
    # USB 패킷 전송 지연 시간 (1ms)
    # 작을수록 응답 빠름, 하지만 너무 작으면 효율 떨어짐

    # --- 4단계: MPSSE 모드 활성화 ---
    handle.setBitMode(0, 0)  # 비트 모드 리셋 (MPSSE 끄기)
    handle.setBitMode(0, 2)  # MPSSE 모드 활성화 (모드 2 = MPSSE)
    time.sleep(0.050)        # 50ms 대기 (모드 전환 안정화)
    # time.sleep(초): 지정된 시간만큼 대기

    # --- 5단계: MPSSE 설정 명령 전송 ---
    handle.write(FTDI_CFG_60MHZ_SYSCLK)  # 60MHz 시스템 클럭 설정
    handle.write(FTDI_CFG_NO_ADAPT_CLK)  # 적응형 클럭 끄기

    # 3상 클럭킹: 고속(30MHz)에서 데이터 안정성 향상
    if use_3phase_clk:
        handle.write(FTDI_CFG_3PHAS_CLK)   # 3상 클럭 켜기
    else:
        handle.write(FTDI_CFG_NO_3PHAS_CLK)  # 3상 클럭 끄기

    # --- 6단계: SPI 핀 및 클럭 설정 ---
    handle.write(FTDI_CFG_SPI_4PIN_CFG)  # SPI 핀 방향 설정
    actual_clk = set_clk(handle, clk_speed)  # SPI 클럭 속도 설정
    time.sleep(0.010)  # 10ms 대기

    handle.write(FTDI_CFG_NO_LOOPBACK)  # 루프백 끄기
    time.sleep(0.010)  # 10ms 대기

    return actual_clk  # 실제 설정된 클럭 속도 반환


def read_gpio(handle) -> int:
    """
    ═══════════════════════════════════════════════════════════════════════════
    함수명: read_gpio
    목적: GPIO 핀의 현재 상태를 읽습니다
    ═══════════════════════════════════════════════════════════════════════════

    【개념 설명】
    GPIO (General Purpose Input/Output)란?
    - 범용 입출력 핀으로, 디지털 신호(HIGH/LOW)를 읽거나 쓸 수 있습니다
    - 여기서는 HOST_INTR 신호를 읽는 데 사용합니다
    - HOST_INTR: 센서가 "데이터 준비됨"을 알려주는 신호

    【매개변수】
    - handle: FTDI 장치 핸들

    【반환값】
    - int: GPIO 상태 (8비트, 0~255)
           각 비트가 하나의 핀 상태를 나타냄
           예: 0xA0 = 10100000 (비트 5, 7이 HIGH)
    ═══════════════════════════════════════════════════════════════════════════
    """
    handle.write(FTDI_CMD_READ_GPIO)  # GPIO 읽기 명령 전송
    res = handle.read(1)               # 1바이트 응답 읽기

    # bytes를 정수로 변환
    # "big" = big-endian (높은 자리가 먼저 오는 방식)
    return int.from_bytes(res, "big")


def spi_read_exact(handle, length: int) -> bytes:
    """
    ═══════════════════════════════════════════════════════════════════════════
    함수명: spi_read_exact
    목적: SPI로 정확히 지정된 바이트 수만큼 읽습니다
    ═══════════════════════════════════════════════════════════════════════════

    【개념 설명】
    왜 "exact"가 중요한가?
    - ftd2xx.read(100)를 호출해도 100바이트 미만이 올 수 있습니다
    - USB 통신 특성상 데이터가 여러 번에 나눠서 올 수 있기 때문
    - 이 함수는 요청한 만큼 다 받을 때까지 계속 읽습니다

    SPI 통신 순서:
    1. CS(Chip Select)를 LOW로 → "통신 시작"
    2. 클럭 신호와 함께 데이터 읽기
    3. CS를 HIGH로 → "통신 종료"

    【매개변수】
    - handle: FTDI 장치 핸들
    - length: 읽을 바이트 수 (1~65536)

    【반환값】
    - bytes: 읽은 데이터 (정확히 length 바이트)

    【예외】
    - ValueError: length가 범위 밖일 때
    - TimeoutError: 데이터를 못 받았을 때
    ═══════════════════════════════════════════════════════════════════════════
    """

    # --- 입력값 검증 ---
    if length < 1 or length > 0x10000:  # 0x10000 = 65536
        raise ValueError("Length must be between 1 and 65536")

    # --- MPSSE 명령 생성 ---
    # MPSSE는 길이를 (실제길이-1)로 인코딩합니다
    # 예: 1바이트 읽기 = 0x0000, 2바이트 = 0x0001, ...
    len_bytes = int(length - 1).to_bytes(2, "little")
    # to_bytes(2, "little"): 2바이트로 변환, little-endian
    # little-endian: 낮은 자리가 먼저 (예: 0x1234 → 0x34, 0x12)

    # 명령어 조합: CS LOW + 읽기 명령 + 길이 + CS HIGH
    cmd = FTDI_CMD_CS_LOW + FTDI_CMD_READ_BYTES + len_bytes + FTDI_CMD_CS_HIGH
    # + 연산자로 bytes 이어붙이기

    handle.write(cmd)  # 명령어 전송

    # --- 데이터 읽기 (정확한 바이트 수 보장) ---
    out = bytearray()  # 가변 바이트 배열 (bytes는 불변)

    while len(out) < length:  # 아직 다 안 읽었으면 계속
        chunk = handle.read(length - len(out))  # 남은 만큼 읽기 시도
        if not chunk:  # 아무것도 못 읽었으면
            raise TimeoutError("SPI read timeout/empty")
        out.extend(chunk)  # 읽은 데이터 추가

    return bytes(out)  # bytearray를 bytes로 변환하여 반환


def byteswap32_fast(data: bytes) -> bytes:
    """
    ═══════════════════════════════════════════════════════════════════════════
    함수명: byteswap32_fast
    목적: 32비트(4바이트) 단위로 바이트 순서를 뒤집습니다
    ═══════════════════════════════════════════════════════════════════════════

    【개념 설명】
    왜 바이트 스왑이 필요한가?
    - 레이더 펌웨어는 32비트 단위로 MSB-first(큰 자리 먼저)로 전송합니다
    - 하지만 우리가 원하는 순서와 다릅니다
    - 그래서 4바이트씩 순서를 뒤집어야 원래 데이터가 됩니다

    예시:
    수신: [D, C, B, A, H, G, F, E]  ← 뒤집힌 상태
    변환: [A, B, C, D, E, F, G, H]  ← 원래 데이터

    더 구체적인 예시:
    수신: 0x04 0x03 0x02 0x01 (바이트 순서)
    변환: 0x01 0x02 0x03 0x04 (올바른 순서)

    【매개변수】
    - data: 변환할 바이트 데이터

    【반환값】
    - bytes: 32비트 단위로 바이트 순서가 뒤집힌 데이터
    ═══════════════════════════════════════════════════════════════════════════
    """

    # --- 4바이트 정렬 확인 ---
    # 32비트 = 4바이트이므로, 길이가 4의 배수여야 함
    if len(data) % 4 != 0:
        # 4의 배수가 아니면 0으로 패딩(채우기)
        data = data + b"\x00" * (4 - (len(data) % 4))
        # b"\x00" * 3 = b"\x00\x00\x00" (0을 3개)

    # --- 빠른 방법: array 모듈 사용 ---
    a = array.array("I")  # "I" = unsigned int (보통 4바이트)

    # 플랫폼에 따라 int 크기가 다를 수 있어서 확인
    if a.itemsize != 4:
        # --- 대체 방법: 수동으로 뒤집기 (느리지만 확실함) ---
        b = bytearray(data)
        for i in range(0, len(b), 4):  # 0, 4, 8, 12, ... 씩 증가
            b[i:i+4] = b[i:i+4][::-1]  # 4바이트씩 뒤집기
            # [::-1] = 리스트/배열 뒤집기 파이썬 문법
            # 예: [1,2,3,4][::-1] = [4,3,2,1]
        return bytes(b)

    # --- array의 byteswap() 메서드 사용 (빠름) ---
    a.frombytes(data)  # bytes를 array로 변환
    a.byteswap()       # 각 요소의 바이트 순서 뒤집기
    return a.tobytes() # array를 bytes로 변환


def intr_active_low(gpio_val: int, mask: int) -> bool:
    """
    ═══════════════════════════════════════════════════════════════════════════
    함수명: intr_active_low
    목적: HOST_INTR 신호가 "준비됨(LOW)" 상태인지 확인합니다
    ═══════════════════════════════════════════════════════════════════════════

    【개념 설명】
    Active-Low란?
    - 신호가 LOW(0V)일 때 "활성화"된 것으로 간주하는 방식
    - HIGH(3.3V)일 때는 "비활성" 상태
    - 반대 개념: Active-High (HIGH일 때 활성)

    마스크(Mask)란?
    - 특정 비트만 확인하기 위한 "필터"
    - AND 연산으로 원하는 비트만 추출합니다

    예시:
    gpio_val = 0b10100101 (GPIO 상태)
    mask     = 0b10100000 (비트 5, 7 확인)
    결과     = 0b10100000 (마스크된 비트가 0이 아님 → 준비 안됨)

    gpio_val = 0b00000101 (GPIO 상태)
    mask     = 0b10100000 (비트 5, 7 확인)
    결과     = 0b00000000 (마스크된 비트가 0 → 준비됨!)

    【매개변수】
    - gpio_val: GPIO 핀 상태 (8비트 정수)
    - mask: 확인할 비트 마스크 (예: 0xA0)

    【반환값】
    - bool: True = 준비됨(LOW), False = 준비 안됨(HIGH)
    ═══════════════════════════════════════════════════════════════════════════
    """
    # & = 비트 AND 연산
    # 예: 0b1010 & 0b1100 = 0b1000
    return (gpio_val & mask) == 0


def wait_intr_low(dev_gpio, mask: int, timeout_s: float = 5.0, poll_sleep_us: int = 10, settle_us: int = 50) -> int:
    """
    ═══════════════════════════════════════════════════════════════════════════
    함수명: wait_intr_low
    목적: HOST_INTR 신호가 LOW가 될 때까지 대기합니다
    ═══════════════════════════════════════════════════════════════════════════

    【개념 설명】
    폴링(Polling)이란?
    - 상태를 계속 확인하며 기다리는 방식
    - "다됐어? 다됐어? 다됐어?" 반복 확인하는 것
    - 반대 개념: 인터럽트 (상대방이 알려줄 때까지 대기)

    settle_us가 필요한 이유:
    - 고속(30MHz)에서는 펌웨어가 GPIO를 LOW로 만든 직후
      데이터가 아직 완전히 준비 안 됐을 수 있습니다
    - 잠깐 기다려야 안정적으로 읽을 수 있습니다

    【매개변수】
    - dev_gpio: GPIO를 읽을 FTDI 장치 핸들
    - mask: HOST_INTR 마스크 (예: 0xA0)
    - timeout_s: 최대 대기 시간 (초, 기본값: 5초)
    - poll_sleep_us: 폴링 간격 (마이크로초, 기본값: 10)
                     0이면 최대 속도로 폴링 (CPU 사용률 높음)
    - settle_us: LOW 감지 후 추가 대기 시간 (마이크로초, 기본값: 50)

    【반환값】
    - int: LOW 감지 시점의 GPIO 값

    【예외】
    - TimeoutError: timeout_s 초과 시
    ═══════════════════════════════════════════════════════════════════════════
    """
    # 시작 시간 기록 (고정밀 타이머)
    t0 = time.perf_counter()  # perf_counter: 고정밀 시간 측정용 (초 단위)
    last = 0  # 마지막으로 읽은 GPIO 값

    # 마이크로초(us)를 초(s)로 변환
    # 1초 = 1,000,000 마이크로초
    sleep_s = max(poll_sleep_us, 0) / 1_000_000.0   # 폴링 간격
    settle_s = max(settle_us, 0) / 1_000_000.0      # 안정화 대기 시간
    # max(x, 0): 음수면 0으로 (음수 대기는 의미 없음)

    # --- 폴링 루프 ---
    while True:
        last = read_gpio(dev_gpio)  # GPIO 상태 읽기

        # LOW 상태 확인
        if intr_active_low(last, mask):
            # LOW 감지! 데이터 준비됨
            # 고속 SPI를 위해 잠깐 더 기다림 (settle time)
            if settle_s > 0:
                time.sleep(settle_s)
            return last  # GPIO 값 반환

        # 타임아웃 확인
        if (time.perf_counter() - t0) > timeout_s:
            # 시간 초과! 에러 발생
            raise TimeoutError(f"HOST_INTR LOW timeout (GPIO=0x{last:02X}, mask=0x{mask:02X})")
            # f"..." = f-string (변수를 문자열에 삽입)
            # 0x{last:02X} = 16진수로 표시, 최소 2자리, 대문자

        # 다음 폴링까지 잠깐 대기
        if sleep_s > 0:
            time.sleep(sleep_s)


def list_ftdi_devices() -> list[tuple[int, str, str]]:
    """
    ═══════════════════════════════════════════════════════════════════════════
    함수명: list_ftdi_devices
    목적: 연결된 FTDI 장치 목록을 반환합니다
    ═══════════════════════════════════════════════════════════════════════════

    【개념 설명】
    FTDI 장치 열거(Enumeration):
    - 컴퓨터에 연결된 모든 FTDI 장치를 찾습니다
    - FT4232H는 4개의 채널이 있어서 4개의 장치로 나타납니다
    - 예: [0] Channel A, [1] Channel B, [2] Channel C, [3] Channel D

    【매개변수】
    없음

    【반환값】
    - list[tuple[int, str, str]]: 장치 목록
      각 항목: (인덱스, 설명, 시리얼번호)
      예: [(0, "FT4232H A", "ABC123"), (1, "FT4232H B", "ABC123"), ...]
    ═══════════════════════════════════════════════════════════════════════════
    """
    devices = []  # 결과를 저장할 빈 리스트

    try:
        n = None  # 장치 개수

        # --- 장치 개수 확인 (두 가지 방법 시도) ---
        try:
            # 방법 1: createDeviceInfoList() 사용
            n = ftd.createDeviceInfoList()
        except Exception:
            # 방법 1 실패 시, 방법 2 시도
            devs = ftd.listDevices()

            if devs is None:
                n = 0  # 장치 없음
            elif isinstance(devs, (list, tuple)):
                # devs가 리스트나 튜플이면 그 길이가 장치 개수
                n = len(devs)
            else:
                # 가끔 정수로 반환되는 경우
                try:
                    n = int(devs)
                except Exception:
                    n = 0

        # --- 각 장치 정보 수집 ---
        for i in range(n or 0):  # n이 None이면 0으로
            try:
                dev = ftd.open(i)    # i번 장치 열기
                info = dev.getDeviceInfo()  # 장치 정보 가져오기

                # 정보 추출 (bytes → 문자열 변환)
                desc = info.get("description", b"Unknown").decode(errors="ignore")
                # .get("key", 기본값): 키가 없으면 기본값 사용
                # .decode(): bytes를 문자열로 변환
                # errors="ignore": 변환 실패한 문자 무시

                serial = info.get("serial", b"").decode(errors="ignore")

                # 결과 리스트에 추가 (튜플로)
                devices.append((i, desc, serial))
                # append(): 리스트 끝에 항목 추가
                # (a, b, c): 튜플 (수정 불가능한 리스트)

                dev.close()  # 장치 닫기 (열어보기만 했으니)
            except Exception:
                pass  # 특정 장치 열기 실패해도 계속 진행

    except Exception:
        pass  # 전체 실패 시 빈 리스트 반환

    return devices


# ============================================================================
# 프레임 경계 휴리스틱 (Frame Boundary Heuristics)
# ============================================================================
# 휴리스틱(Heuristic): 경험적 규칙. 100% 정확하지는 않지만 대부분 맞는 방법
#
# 【문제 상황】
# 프로그램을 시작할 때, 센서는 이미 데이터를 보내고 있을 수 있습니다.
# 만약 프레임 중간부터 읽기 시작하면 데이터가 엉망이 됩니다.
#
# 【해결 방법】
# 테스트 패턴의 특징을 이용해 "이게 프레임 시작인지" 판단합니다.
#
# 테스트 패턴 구조:
# - 프레임 = 128KB (=2개의 64KB 청크)
# - chunk0 (첫 번째 64KB): byte[0~3] = 프레임 카운터, byte[4~] = 0,1,2,3,...
# - chunk1 (두 번째 64KB): byte[65536~] = (i-4)&0xFF 계속...
#                          시작 부분이 0xFC, 0xFD, 0xFE, 0xFF로 고정!
#
# 따라서:
# - 앞 4바이트가 FC FD FE FF이면 → chunk1 (프레임 중간)
# - 그렇지 않으면 → chunk0 (프레임 시작)

# chunk1의 시작 패턴 (테스트 패턴 기준)
# byte[65536] = (65536-4) & 0xFF = 65532 & 255 = 252 = 0xFC
# byte[65537] = (65537-4) & 0xFF = 65533 & 255 = 253 = 0xFD
# byte[65538] = (65538-4) & 0xFF = 65534 & 255 = 254 = 0xFE
# byte[65539] = (65539-4) & 0xFF = 65535 & 255 = 255 = 0xFF
CHUNK1_HEAD = b"\xFC\xFD\xFE\xFF"


def looks_like_chunk0(chunk_byteswapped: bytes) -> bool:
    """
    ═══════════════════════════════════════════════════════════════════════════
    함수명: looks_like_chunk0
    목적: 이 청크가 프레임의 첫 번째 청크(chunk0)처럼 보이는지 판단합니다
    ═══════════════════════════════════════════════════════════════════════════

    【판단 기준】
    - chunk0: 프레임 카운터로 시작 (FC FD FE FF가 아님)
    - chunk1: FC FD FE FF로 시작 (테스트 패턴의 특성)

    chunk0이면 True를 반환합니다.
    이것은 "프레임 시작점을 찾았다"는 의미입니다.

    【매개변수】
    - chunk_byteswapped: byteswap 처리된 청크 데이터

    【반환값】
    - bool: True = chunk0처럼 보임 (프레임 시작)
            False = chunk1처럼 보임 (프레임 중간)
    ═══════════════════════════════════════════════════════════════════════════
    """
    # 데이터가 너무 짧으면 판단 불가
    if len(chunk_byteswapped) < 4:
        return False

    # 앞 4바이트가 CHUNK1_HEAD가 아니면 chunk0
    # [:4] = 처음 4개 요소 (슬라이싱)
    # != : 같지 않음
    return chunk_byteswapped[:4] != CHUNK1_HEAD


def looks_like_chunk1(chunk_byteswapped: bytes) -> bool:
    """
    ═══════════════════════════════════════════════════════════════════════════
    함수명: looks_like_chunk1
    목적: 이 청크가 프레임의 두 번째 청크(chunk1)처럼 보이는지 판단합니다
    ═══════════════════════════════════════════════════════════════════════════

    chunk1은 FC FD FE FF로 시작합니다.
    resync에서 chunk0 다음에 chunk1이 오는지 확인하는 데 사용합니다.

    【매개변수】
    - chunk_byteswapped: byteswap 처리된 청크 데이터

    【반환값】
    - bool: True = chunk1처럼 보임 (FC FD FE FF로 시작)
    ═══════════════════════════════════════════════════════════════════════════
    """
    if len(chunk_byteswapped) < 4:
        return False

    # 앞 4바이트가 CHUNK1_HEAD와 같으면 chunk1
    return chunk_byteswapped[:4] == CHUNK1_HEAD


# ============================================================================
# 캡처 워커 (Capture Worker) - 실제 데이터 수집을 담당
# ============================================================================
# 워커(Worker): 별도의 스레드에서 실행되는 작업자
# GUI가 멈추지 않고 반응하도록 데이터 캡처는 별도 스레드에서 실행합니다.


@dataclass
class CaptureSettings:
    """
    ═══════════════════════════════════════════════════════════════════════════
    클래스명: CaptureSettings
    목적: 캡처에 필요한 모든 설정값을 하나로 묶어서 관리합니다
    ═══════════════════════════════════════════════════════════════════════════

    【개념 설명】
    @dataclass란?
    - Python의 데코레이터(장식자)
    - 클래스에 __init__, __repr__ 등을 자동 생성해줍니다
    - 여러 설정값을 깔끔하게 묶어서 전달할 때 편리합니다

    사용 예:
        settings = CaptureSettings(
            spi_dev_index=0,
            gpio_dev_index=1,
            clock_hz=30000000,
            ...
        )
        print(settings.clock_hz)  # 30000000

    이렇게 하면 함수에 수십 개의 매개변수를 넘기는 대신,
    settings 하나만 넘기면 됩니다.
    ═══════════════════════════════════════════════════════════════════════════
    """

    # --- 장치 설정 ---
    spi_dev_index: int     # SPI 통신에 사용할 FTDI 장치 번호 (보통 0)
    gpio_dev_index: int    # GPIO 읽기에 사용할 FTDI 장치 번호 (보통 1)
    clock_hz: int          # SPI 클럭 속도 (Hz). 예: 30_000_000 = 30MHz
    num_frames: int        # 캡처할 프레임 수 (0 = 무한)

    # --- 레이더 설정 (프레임 크기 계산에 사용) ---
    # 프레임 크기 = chirps × bursts × adc_samples × rx_antennas × 2바이트
    adc_samples: int       # ADC 샘플 수 (기본값: 256)
    chirps_per_burst: int  # 버스트당 처프 수 (기본값: 64)
    bursts_per_frame: int  # 프레임당 버스트 수 (기본값: 1)
    rx_antennas: int       # 수신 안테나 수 (기본값: 4)
    # 기본값으로 계산: 64 × 1 × 256 × 4 × 2 = 131,072바이트 = 128KB

    # --- HOST_INTR 설정 ---
    host_intr_mask: int    # HOST_INTR 마스크 (예: 0xA0)
    poll_sleep_us: int     # GPIO 폴링 간격 (마이크로초)
    settle_us: int         # LOW 감지 후 대기 시간 (고속 SPI용)

    # --- 캡처 옵션 ---
    byteswap32: bool       # 32비트 바이트 스왑 적용 여부
    resync_on_start: bool  # 시작 시 프레임 경계 동기화 여부
    use_3phase_clk: bool   # 3상 클럭킹 사용 (고속 안정성)

    # --- 출력 설정 ---
    save_to_file: bool     # 파일 저장 여부
    out_path: str          # 저장 폴더 경로

    # --- 로깅/미리보기 설정 ---
    flush_every: int       # N 프레임마다 버퍼 플러시 (0=끄기)
    preview_every: int     # N 프레임마다 미리보기 출력 (0=끄기)
    log_every: int         # N 프레임마다 로그 출력

    # --- 기타 ---
    device_type: str       # 장치 타입 표시용 ("AOP" 또는 "FCCSP")


class SpiCaptureWorker(QObject):
    """
    ═══════════════════════════════════════════════════════════════════════════
    클래스명: SpiCaptureWorker
    목적: 별도 스레드에서 SPI 데이터 캡처를 수행하는 워커 클래스
    ═══════════════════════════════════════════════════════════════════════════

    【개념 설명】
    왜 별도 스레드가 필요한가?
    - GUI 프로그램에서 오래 걸리는 작업을 메인 스레드에서 하면
      화면이 "응답 없음" 상태가 됩니다
    - 별도 스레드에서 실행하면 GUI는 계속 반응합니다

    QObject를 상속하는 이유:
    - Qt의 시그널/슬롯 메커니즘을 사용하기 위해
    - 시그널: "이벤트가 발생했어요!"라고 알리는 것
    - 슬롯: 시그널을 받아서 처리하는 함수

    pyqtSignal이란?
    - 스레드 간 안전하게 데이터를 주고받는 방법
    - 워커가 "상태 변경됨!"이라고 시그널을 보내면
      GUI가 받아서 화면을 업데이트합니다
    ═══════════════════════════════════════════════════════════════════════════
    """

    # --- 시그널 정의 ---
    # 시그널은 클래스 레벨에서 정의해야 합니다 (인스턴스가 아님)

    status = pyqtSignal(str)      # 상태 메시지 전송 (문자열)
                                   # 예: "Frame 10 captured"

    error = pyqtSignal(str)       # 에러 메시지 전송 (문자열)
                                   # 예: "Connection failed"

    progress = pyqtSignal(int, int)  # 진행 상황 전송 (현재, 전체)
                                      # 예: (10, 100) = 10/100 완료

    preview = pyqtSignal(bytes)   # 미리보기 데이터 전송 (바이트)
                                   # 프레임의 첫 64바이트

    finished = pyqtSignal()       # 작업 완료 알림 (매개변수 없음)

    def __init__(self, settings: CaptureSettings):
        """
        생성자: 워커 객체를 초기화합니다

        【매개변수】
        - settings: CaptureSettings 객체 (모든 설정값 포함)
        """
        super().__init__()  # 부모 클래스(QObject) 초기화
        # super(): 부모 클래스를 참조

        self.settings = settings    # 설정 저장
        self._running = False       # 실행 중 플래그 (stop 요청 확인용)
        self._dev_spi = None        # SPI 장치 핸들
        self._dev_gpio = None       # GPIO 장치 핸들
        # _로 시작하는 변수: "내부용"이라는 관례 (외부에서 직접 접근 자제)

    @pyqtSlot()
    def start(self):
        """
        ═══════════════════════════════════════════════════════════════════════
        메서드명: start
        목적: 데이터 캡처를 시작합니다 (메인 작업 함수)
        ═══════════════════════════════════════════════════════════════════════

        【개념 설명】
        @pyqtSlot()이란?
        - 이 함수가 Qt 시그널의 "슬롯"임을 표시
        - 다른 스레드에서 호출될 때 Qt가 적절히 처리합니다

        이 함수의 전체 흐름:
        1. 프레임 크기 계산
        2. FTDI 장치 열기 및 초기화
        3. (선택) resync: 프레임 경계 찾기
        4. 캡처 루프: 프레임 읽기 → 저장 → 반복
        5. 정리 및 종료
        ═══════════════════════════════════════════════════════════════════════
        """
        self._running = True  # 실행 중 플래그 설정

        try:
            # ================================================================
            # 1단계: 프레임 크기 및 청크 수 계산
            # ================================================================
            # 프레임 크기 공식:
            # frame_size = chirps × bursts × adc_samples × rx_antennas × 2바이트
            # 2바이트 = I/Q 각각 1바이트 (복소수 신호)
            frame_size = (self.settings.chirps_per_burst *
                          self.settings.bursts_per_frame *
                          self.settings.adc_samples *
                          self.settings.rx_antennas * 2)

            # 프레임을 몇 개의 청크로 나눠야 하는지 계산
            # 올림 나눗셈: (a + b - 1) // b
            chunks_per_frame = (frame_size + FTDI_MAX_CHUNK - 1) // FTDI_MAX_CHUNK
            if chunks_per_frame < 1:
                chunks_per_frame = 1  # 최소 1개

            # GUI에 상태 정보 출력 (emit = 시그널 발생)
            self.status.emit(f"Frame Size      : {frame_size:,} bytes")
            # {:,} = 천 단위 쉼표 (예: 131,072)
            self.status.emit(f"Chunks/Frame    : {chunks_per_frame}")
            self.status.emit(f"HOST_INTR mask  : 0x{self.settings.host_intr_mask:02X}  (ready when GPIO&mask==0)")
            self.status.emit(f"Byteswap32      : {'ON' if self.settings.byteswap32 else 'OFF'}")
            # 'A' if 조건 else 'B' = 조건부 표현식 (삼항 연산자)
            self.status.emit(f"Resync          : {'ON' if self.settings.resync_on_start else 'OFF'}")
            self.status.emit(f"Settle time     : {self.settings.settle_us} us")

            # ================================================================
            # 2단계: FTDI 장치 열기 및 초기화
            # ================================================================

            # --- SPI 장치 열기 ---
            self._dev_spi = ftd.open(self.settings.spi_dev_index)
            # ftd.open(인덱스): FTDI 장치를 열고 핸들 반환
            self.status.emit(f"SPI device opened: index {self.settings.spi_dev_index}")

            # SPI 장치 초기화 및 클럭 설정
            actual_clk = set_device(self._dev_spi, self.settings.clock_hz, latency_timer=1, rw_timeout_ms=5000,
                                    use_3phase_clk=self.settings.use_3phase_clk)

            # 클럭 설정 결과 메시지 생성
            clk_msg = f"SPI configured: {actual_clk/1e6:.1f} MHz"
            # actual_clk/1e6 = Hz를 MHz로 변환 (1e6 = 1,000,000)
            # :.1f = 소수점 1자리까지 표시

            if actual_clk != self.settings.clock_hz:
                # 요청한 속도와 실제 설정된 속도가 다르면 둘 다 표시
                clk_msg = f"SPI configured: requested {self.settings.clock_hz/1e6:.1f} MHz -> actual {actual_clk/1e6:.1f} MHz"

            if self.settings.use_3phase_clk:
                clk_msg += " (3-phase)"  # 3상 클럭 사용 중 표시
            self.status.emit(clk_msg)

            # --- GPIO 장치 열기 ---
            # FT4232H는 4개의 채널이 있어서, SPI와 GPIO를 다른 채널로 쓸 수 있습니다
            # AOP 보드는 보통 채널 A(0)=SPI, 채널 B(1)=GPIO 사용
            if self.settings.gpio_dev_index != self.settings.spi_dev_index:
                # SPI와 GPIO가 다른 채널인 경우 별도로 열기
                self._dev_gpio = ftd.open(self.settings.gpio_dev_index)
                # GPIO는 속도가 중요하지 않아서 1MHz로 설정
                set_device(self._dev_gpio, 1_000_000, latency_timer=1, rw_timeout_ms=5000)
                self.status.emit(f"GPIO device opened: index {self.settings.gpio_dev_index} (separate)")
            else:
                # SPI와 GPIO가 같은 채널인 경우 (FCCSP 보드 등)
                self._dev_gpio = self._dev_spi  # 같은 핸들 사용
                self.status.emit("GPIO device: same as SPI device")

            # ================================================================
            # 3단계: 출력 폴더 준비
            # ================================================================
            # 각 프레임은 별도의 .bin 파일로 저장됩니다
            # 예: frame_0000.bin, frame_0001.bin, ...

            out_folder = None  # 저장 폴더 경로 (저장 안 하면 None)

            if self.settings.save_to_file and self.settings.out_path:
                out_folder = self.settings.out_path

                # 폴더가 없으면 생성
                os.makedirs(out_folder, exist_ok=True)
                # exist_ok=True: 이미 있어도 에러 안 냄

                # --- 기존 .bin 파일 삭제 ---
                # glob.glob(): 패턴에 맞는 파일 목록 찾기
                # *.bin = 확장자가 .bin인 모든 파일
                old_files = glob.glob(os.path.join(out_folder, "*.bin"))
                # os.path.join(): 폴더 경로와 파일명을 운영체제에 맞게 연결

                if old_files:
                    removed = 0   # 삭제 성공 카운터
                    failed = 0    # 삭제 실패 카운터

                    for f in old_files:  # 각 파일에 대해
                        try:
                            os.remove(f)  # 파일 삭제
                            removed += 1
                        except Exception as e:  # 삭제 실패 시
                            failed += 1
                            self.status.emit(f"Failed to remove: {f} ({e})")

                    if removed > 0:
                        self.status.emit(f"Removed {removed} existing .bin files")
                    if failed > 0:
                        self.status.emit(f"Failed to remove {failed} files")

                self.status.emit(f"Saving to: {out_folder} (1 file per frame)")
            else:
                self.status.emit("Output          : (not saving)")

            self.status.emit("Waiting for sensor streaming...")

            # --- 캡처 목표 설정 ---
            # num_frames=0이면 무한 캡처 (float("inf") = 무한대)
            target_frames = self.settings.num_frames if self.settings.num_frames > 0 else float("inf")
            # 진행률 표시용 (무한이면 0으로 설정)
            total_for_progress = int(target_frames) if target_frames != float("inf") else 0

            frame_count = 0      # 캡처한 프레임 수
            total_bytes = 0      # 총 캡처 바이트 수
            start_t = time.perf_counter()  # 시작 시간 (속도 계산용)

            # ================================================================
            # 4단계: RESYNC ON START (프레임 경계 동기화)
            # ================================================================
            # 【왜 필요한가?】
            # 프로그램 시작 시 센서는 이미 데이터를 보내고 있을 수 있습니다.
            # 만약 프레임 중간(chunk1)부터 읽기 시작하면:
            # - 미리보기가 이상하게 나옴 (프레임 카운터가 아닌 데이터)
            # - 데이터 정렬이 어긋남
            #
            # 【해결 방법】
            # chunk0(프레임 시작)을 찾을 때까지 데이터를 버리고,
            # 제대로 된 프레임 시작점부터 캡처를 시작합니다.

            pending_chunks: list[bytes] = []  # resync에서 미리 읽은 청크 보관
            # list[bytes]: bytes 타입 요소를 담는 리스트

            if self.settings.resync_on_start:
                self.status.emit("Syncing to frame boundary (looking for chunk0 header)...")
                discards = 0  # 버린 청크 수

                # --- resync 루프: chunk0을 찾을 때까지 반복 ---
                while self._running:  # stop 버튼 누르면 _running=False가 됨
                    # 짧은 타임아웃(0.25초)으로 폴링하여 stop 버튼 반응성 유지
                    try:
                        wait_intr_low(self._dev_gpio, self.settings.host_intr_mask,
                                      timeout_s=0.25,
                                      poll_sleep_us=self.settings.poll_sleep_us,
                                      settle_us=self.settings.settle_us)
                    except TimeoutError:
                        continue  # 타임아웃이면 다시 시도

                    # 청크 하나 읽기
                    raw0 = spi_read_exact(self._dev_spi, min(FTDI_MAX_CHUNK, frame_size))
                    # min(a, b): 둘 중 작은 값 (프레임이 64KB보다 작을 수 있음)

                    # byteswap 적용 (검사를 위해)
                    if self.settings.byteswap32:
                        raw0 = byteswap32_fast(raw0)

                    # --- chunk0인지 확인 ---
                    if not looks_like_chunk0(raw0):
                        # chunk0이 아니면 버리고 계속
                        discards += 1
                        if discards % 5 == 0:  # 5개마다 상태 출력
                            # 앞 8바이트를 16진수로 표시
                            head8 = " ".join(f"{b:02X}" for b in raw0[:8])
                            # 리스트 컴프리헨션: [f(...) for b in raw0[:8]]
                            # " ".join([...]): 리스트 요소를 공백으로 연결
                            self.status.emit(f"Resync: discarded {discards} chunks (head={head8})")
                        continue  # 다음 청크로

                    # --- chunk0 발견! ---

                    # 프레임이 1청크짜리면 바로 완료
                    if chunks_per_frame == 1:
                        pending_chunks = [raw0]
                        break  # while 루프 탈출

                    # --- 2청크 이상: chunk1도 확인 ---
                    # 더 확실하게 하려면 다음 청크가 chunk1 패턴인지 확인
                    try:
                        wait_intr_low(self._dev_gpio, self.settings.host_intr_mask,
                                      timeout_s=0.25,
                                      poll_sleep_us=self.settings.poll_sleep_us,
                                      settle_us=self.settings.settle_us)
                    except TimeoutError:
                        # 다음 청크를 못 읽었으면 버리고 다시 시도
                        discards += 1
                        continue

                    # chunk1 후보 읽기
                    raw1 = spi_read_exact(self._dev_spi, min(FTDI_MAX_CHUNK, frame_size - len(raw0)))
                    # frame_size - len(raw0): 남은 바이트 수
                    if self.settings.byteswap32:
                        raw1 = byteswap32_fast(raw1)

                    # chunk1 패턴(FC FD FE FF) 확인
                    if looks_like_chunk1(raw1):
                        # 성공! chunk0과 chunk1을 모두 확보
                        pending_chunks = [raw0, raw1]
                        break  # while 루프 탈출

                    # chunk1이 아니면 잘못 정렬된 것 → 버리고 다시 시도
                    discards += 1
                    if discards % 5 == 0:
                        head8 = " ".join(f"{b:02X}" for b in raw0[:8])
                        self.status.emit(f"Resync: discarded {discards} chunks (head={head8})")

                # resync 결과 확인
                if not pending_chunks and self._running:
                    raise RuntimeError("Resync failed: no frame boundary detected")
                    # raise: 에러 발생 → except 블록으로 점프

            # ================================================================
            # 5단계: 캡처 루프 (메인 작업)
            # ================================================================
            # 【작업 흐름】
            # 반복:
            #   1. 프레임의 모든 청크 읽기
            #   2. 파일로 저장
            #   3. 미리보기/로그 출력
            #   4. 다음 프레임으로

            while self._running and frame_count < target_frames:
                # --- 프레임 단위 변수 초기화 ---
                frame_data = bytearray()  # 현재 프레임 데이터 (청크들을 모음)
                first64 = None            # 미리보기용 (프레임 앞 64바이트)

                # resync에서 미리 읽은 청크 사용 (첫 프레임에만 해당)
                chunks_to_use = pending_chunks
                pending_chunks = []  # 사용 후 비우기

                remain = frame_size  # 남은 바이트 수

                # ===== chunk 0 읽기 (프레임의 첫 번째 청크) =====
                if chunks_to_use:
                    # resync에서 이미 읽어둔 청크 사용
                    chunk0 = chunks_to_use.pop(0)  # 첫 요소 꺼내기
                    # pop(0): 리스트의 첫 요소를 제거하고 반환
                else:
                    # 새로 읽기: HOST_INTR LOW 대기 후 SPI 읽기
                    while self._running:
                        try:
                            wait_intr_low(self._dev_gpio, self.settings.host_intr_mask,
                                          timeout_s=0.25,
                                          poll_sleep_us=self.settings.poll_sleep_us,
                                          settle_us=self.settings.settle_us)
                            break
                        except TimeoutError:
                            continue

                    if not self._running:
                        break  # stop 요청 시 루프 탈출

                    chunk0 = spi_read_exact(self._dev_spi, min(FTDI_MAX_CHUNK, remain))
                    if self.settings.byteswap32:
                        chunk0 = byteswap32_fast(chunk0)

                remain -= len(chunk0)      # 남은 바이트 갱신
                first64 = chunk0[:64]      # 미리보기용 저장
                frame_data.extend(chunk0)  # 프레임 데이터에 추가
                # extend(): 리스트/배열 끝에 여러 요소 추가

                # ===== 나머지 청크들 읽기 =====
                while self._running and remain > 0:
                    if chunks_to_use:
                        # resync에서 미리 읽은 청크 사용
                        chunk = chunks_to_use.pop(0)
                    else:
                        # 새로 읽기
                        while self._running:
                            try:
                                wait_intr_low(self._dev_gpio, self.settings.host_intr_mask,
                                              timeout_s=0.25,
                                              poll_sleep_us=self.settings.poll_sleep_us,
                                              settle_us=self.settings.settle_us)
                                break
                            except TimeoutError:
                                continue

                        if not self._running:
                            break

                        chunk = spi_read_exact(self._dev_spi, min(FTDI_MAX_CHUNK, remain))
                        if self.settings.byteswap32:
                            chunk = byteswap32_fast(chunk)

                    remain -= len(chunk)
                    frame_data.extend(chunk)

                if not self._running:
                    break  # stop 요청 시 종료

                # ===== 파일 저장 =====
                if out_folder:
                    # 파일명: frame_0000.bin, frame_0001.bin, ...
                    frame_file = os.path.join(out_folder, f"frame_{frame_count:04d}.bin")
                    # :04d = 4자리 정수, 앞에 0 채움 (0001, 0012, 0123, 1234)

                    with open(frame_file, "wb") as f:
                        f.write(frame_data)
                    # with 문: 파일을 자동으로 닫아줌
                    # "wb" = write binary (바이너리 쓰기 모드)

                frame_count += 1              # 프레임 카운터 증가
                total_bytes += len(frame_data)  # 총 바이트 갱신

                # ===== 미리보기 출력 =====
                # preview_every > 0 이고 N번째 프레임이면 출력
                if (self.settings.preview_every > 0 and
                    (frame_count % self.settings.preview_every) == 0 and
                    first64 is not None):
                    self.preview.emit(first64)  # GUI로 시그널 전송
                    # %: 나머지 연산자 (10 % 3 = 1)
                    # frame_count % N == 0 : N의 배수일 때

                # ===== 진행률 업데이트 =====
                self.progress.emit(frame_count, total_for_progress)

                # ===== 로그 출력 =====
                if (self.settings.log_every > 0 and
                    (frame_count % self.settings.log_every) == 0):
                    elapsed = time.perf_counter() - start_t  # 경과 시간 (초)
                    # 전송 속도 계산 (MB/s)
                    mbps = (total_bytes / (1024 * 1024)) / elapsed if elapsed > 0 else 0.0
                    # 1024 * 1024 = 1MB
                    self.status.emit(
                        f"Frame {frame_count}: {frame_size} bytes | Total: {total_bytes/1024:.1f} KB | {mbps:.2f} MB/s"
                    )

            # ================================================================
            # 6단계: 캡처 완료
            # ================================================================
            elapsed = time.perf_counter() - start_t
            mbps = (total_bytes / (1024 * 1024)) / elapsed if elapsed > 0 else 0.0
            self.status.emit(f"Capture complete: {frame_count} frames, {total_bytes} bytes, {mbps:.2f} MB/s")

        except Exception as e:
            # ================================================================
            # 에러 처리
            # ================================================================
            # 어떤 에러가 발생하면 여기로 점프합니다
            import traceback
            # traceback: 에러 발생 위치 추적 정보

            self.error.emit(f"Capture error: {e}\n{traceback.format_exc()}")
            # traceback.format_exc(): 에러 스택 트레이스를 문자열로

        finally:
            # ================================================================
            # 정리 작업 (finally: 에러 여부와 관계없이 항상 실행)
            # ================================================================
            self._cleanup()      # 장치 닫기
            self.finished.emit() # 완료 시그널 (GUI에 알림)

    @pyqtSlot()
    def stop(self):
        """
        ═══════════════════════════════════════════════════════════════════════
        메서드명: stop
        목적: 캡처를 중지합니다 (Stop 버튼 클릭 시 호출)
        ═══════════════════════════════════════════════════════════════════════

        _running 플래그를 False로 설정하면,
        캡처 루프가 다음 반복에서 이를 확인하고 종료합니다.
        (바로 종료되지 않고 현재 청크 처리 후 종료)
        """
        self._running = False

    def _cleanup(self):
        """
        ═══════════════════════════════════════════════════════════════════════
        메서드명: _cleanup (내부용 메서드)
        목적: FTDI 장치를 닫고 리소스를 정리합니다
        ═══════════════════════════════════════════════════════════════════════

        【왜 try/except로 감싸는가?】
        장치를 닫을 때 에러가 발생해도 프로그램이 멈추지 않도록 합니다.
        이미 닫힌 장치를 다시 닫으려 하면 에러가 날 수 있습니다.
        """
        # SPI 장치 닫기
        try:
            if self._dev_spi:  # 핸들이 있으면
                self._dev_spi.close()
        except Exception:
            pass  # 에러 무시

        # GPIO 장치 닫기 (SPI와 다른 경우에만)
        try:
            if self._dev_gpio and self._dev_gpio != self._dev_spi:
                # SPI와 GPIO가 같은 장치면 이미 닫혔으므로 건너뜀
                self._dev_gpio.close()
        except Exception:
            pass

        # 핸들 초기화 (None으로 설정하여 다시 사용하지 않도록)
        self._dev_spi = None
        self._dev_gpio = None

        self.status.emit("Devices closed")  # GUI에 알림


# ============================================================================
# GUI (Graphical User Interface) - 그래픽 사용자 인터페이스
# ============================================================================
# 사용자가 버튼 클릭, 값 입력 등으로 프로그램을 조작하는 화면입니다.
# PyQt6 라이브러리를 사용합니다.


class MainWindow(QMainWindow):
    """
    ═══════════════════════════════════════════════════════════════════════════
    클래스명: MainWindow
    목적: 프로그램의 메인 윈도우 (GUI 전체)
    ═══════════════════════════════════════════════════════════════════════════

    【개념 설명】
    QMainWindow란?
    - Qt의 메인 윈도우 클래스
    - 메뉴바, 툴바, 상태바 등을 기본 제공
    - 대부분의 데스크톱 프로그램 창의 기반

    GUI 프로그램의 구조:
    - 위젯(Widget): 버튼, 텍스트박스 등 UI 요소
    - 레이아웃(Layout): 위젯들을 배치하는 방식
    - 시그널/슬롯: 이벤트 처리 (버튼 클릭 → 함수 호출)
    ═══════════════════════════════════════════════════════════════════════════
    """

    def __init__(self):
        """
        생성자: 메인 윈도우를 초기화합니다

        【실행 순서】
        1. 부모 클래스 초기화
        2. 창 제목 및 크기 설정
        3. UI 빌드 (_build_ui)
        4. FTDI 장치 목록 새로고침 (_refresh_devices)
        """
        super().__init__()  # 부모 클래스(QMainWindow) 초기화

        # 창 제목 설정
        self.setWindowTitle("AWRL6844 SPI Data Capture Tool (FT4232H / u32_to_be(cnt) / resync+exact-read)")

        # 창 크기 설정 (가로 1050, 세로 820 픽셀)
        self.resize(1050, 820)

        # --- 스레드 관련 변수 ---
        self.capture_thread: QThread | None = None
        # QThread | None : QThread 타입이거나 None (타입 힌트)

        self.capture_worker: SpiCaptureWorker | None = None

        # UI 구축 및 장치 목록 새로고침
        self._build_ui()
        self._refresh_devices()

    def _build_ui(self):
        """
        ═══════════════════════════════════════════════════════════════════════
        메서드명: _build_ui
        목적: 모든 UI 요소(위젯)를 생성하고 배치합니다
        ═══════════════════════════════════════════════════════════════════════

        【GUI 구조】
        ┌─────────────────────────────────────────┐
        │ FTDI Device Selection (장치 선택)        │
        ├─────────────────────────────────────────┤
        │ Radar Configuration (레이더 설정)        │
        ├─────────────────────────────────────────┤
        │ HOST_INTR Configuration (인터럽트 설정)  │
        ├─────────────────────────────────────────┤
        │ Capture Options (캡처 옵션)              │
        ├─────────────────────────────────────────┤
        │ Output Folder (출력 폴더)                │
        ├─────────────────────────────────────────┤
        │ [Start] [Stop] ▓▓▓▓░░░░ Progress        │
        ├─────────────────────────────────────────┤
        │ Log (로그 출력 영역)                      │
        │                                         │
        │                                         │
        └─────────────────────────────────────────┘

        【레이아웃 설명】
        - QVBoxLayout: 위젯을 세로로 배치 (V = Vertical)
        - QHBoxLayout: 위젯을 가로로 배치 (H = Horizontal)
        - QGroupBox: 관련 위젯을 그룹으로 묶어 테두리 표시
        """

        # --- 중앙 위젯 설정 ---
        # QMainWindow는 반드시 중앙 위젯을 가져야 합니다
        central = QWidget()
        self.setCentralWidget(central)

        # 루트 레이아웃: 모든 요소를 세로로 배치
        root = QVBoxLayout(central)
        # QVBoxLayout(central): central 위젯에 세로 레이아웃 적용

        # ════════════════════════════════════════════════════════════════════
        # 섹션 1: FTDI 장치 선택
        # ════════════════════════════════════════════════════════════════════
        dev_group = QGroupBox("FTDI Device Selection")
        # QGroupBox: 테두리와 제목이 있는 그룹 박스
        dev_layout = QHBoxLayout(dev_group)

        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self._refresh_devices)

        self.device_type_combo = QComboBox()
        self.device_type_combo.addItems(["AOP", "FCCSP"])
        self.device_type_combo.setCurrentText("AOP")  # default as requested

        self.spi_combo = QComboBox()
        self.gpio_combo = QComboBox()

        dev_layout.addWidget(self.btn_refresh)
        dev_layout.addWidget(QLabel("Type:"))
        dev_layout.addWidget(self.device_type_combo)
        dev_layout.addWidget(QLabel("SPI Device:"))
        dev_layout.addWidget(self.spi_combo, 2)
        dev_layout.addWidget(QLabel("GPIO Device:"))
        dev_layout.addWidget(self.gpio_combo, 2)
        root.addWidget(dev_group)

        # ---- Radar Configuration ----
        radar_group = QGroupBox("Radar Configuration (must match firmware - main_full_mss.c)")
        radar_layout = QVBoxLayout(radar_group)

        row1 = QHBoxLayout()
        self.spin_adc = QSpinBox()
        self.spin_adc.setRange(64, 4096)
        self.spin_adc.setValue(256)

        self.spin_chirps = QSpinBox()
        self.spin_chirps.setRange(1, 1024)
        self.spin_chirps.setValue(64)

        self.spin_bursts = QSpinBox()
        self.spin_bursts.setRange(1, 256)
        self.spin_bursts.setValue(1)

        self.spin_rx = QSpinBox()
        self.spin_rx.setRange(1, 4)
        self.spin_rx.setValue(4)

        row1.addWidget(QLabel("ADC Samples:"))
        row1.addWidget(self.spin_adc)
        row1.addWidget(QLabel("Chirps/Burst:"))
        row1.addWidget(self.spin_chirps)
        row1.addWidget(QLabel("Bursts/Frame:"))
        row1.addWidget(self.spin_bursts)
        row1.addWidget(QLabel("RX Antennas:"))
        row1.addWidget(self.spin_rx)
        row1.addStretch(1)
        radar_layout.addLayout(row1)

        row2 = QHBoxLayout()
        self.spin_frames = QSpinBox()
        self.spin_frames.setRange(0, 1_000_000)
        self.spin_frames.setValue(20)
        self.spin_frames.setSpecialValueText("Infinite")

        self.clock_combo = QComboBox()
        # FT4232H MPSSE: actual_clk = 60MHz / (2 * (div + 1))
        # Only these exact frequencies are achievable:
        self.clock_combo.addItems([
            "30000000",  # div=0: 30MHz (max)
            "24000000",  # div=1: 24MH
            "15000000",  # div=1: 15MHz
            "10000000",  # div=2: 10MHz
            "6000000",   # div=4: 6MHz
            "3000000",   # div=9: 3MHz
            "1000000",   # div=29: 1MHz
        ])
        self.clock_combo.setCurrentText("30000000")  # 15MHz default (stable)

        self.lbl_frame_size = QLabel("Frame size: --")
        self.lbl_frame_size.setStyleSheet("font-weight: bold;")

        row2.addWidget(QLabel("Frames (0=inf):"))
        row2.addWidget(self.spin_frames)
        row2.addWidget(QLabel("SPI Clock (Hz):"))
        row2.addWidget(self.clock_combo)
        row2.addStretch(1)
        row2.addWidget(self.lbl_frame_size)
        radar_layout.addLayout(row2)

        help_label = QLabel(
            "Firmware formula: frame_size = chirps × bursts × adc_samples × rx × 2\n"
            "Default: 64 × 1 × 256 × 4 × 2 = 131,072 bytes (2 × 64KB chunks)"
        )
        help_label.setStyleSheet("color: gray; font-size: 10px;")
        radar_layout.addWidget(help_label)

        for w in (self.spin_adc, self.spin_chirps, self.spin_bursts, self.spin_rx):
            w.valueChanged.connect(self._update_frame_size)
        self._update_frame_size()

        root.addWidget(radar_group)

        # ---- HOST_INTR Configuration ----
        intr_group = QGroupBox("HOST_INTR GPIO Configuration")
        intr_layout = QHBoxLayout(intr_group)

        self.intr_mask_combo = QComboBox()
        self.intr_mask_combo.addItems([
            "Bit 4 (0x10)",
            "Bit 5 (0x20)",
            "Bit 7 (0x80)",
            "Bits 5+7 (0xA0)  <-- default"
        ])
        self.intr_mask_combo.setCurrentIndex(3)  # default 0xA0 as requested

        self.spin_poll_us = QSpinBox()
        self.spin_poll_us.setRange(0, 5000)
        self.spin_poll_us.setValue(10)

        self.spin_settle_us = QSpinBox()
        self.spin_settle_us.setRange(0, 5000)
        self.spin_settle_us.setValue(100)  # 100us default - FW needs time to call MCSPI_transfer() after GPIO LOW
        self.spin_settle_us.setToolTip("Settle time after HOST_INTR LOW detection.\nFW sets GPIO LOW then calls MCSPI_transfer().\nIncrease if 24MHz data is ignored (try 200-500us).")

        intr_layout.addWidget(QLabel("HOST_INTR mask:"))
        intr_layout.addWidget(self.intr_mask_combo, 2)
        intr_layout.addWidget(QLabel("GPIO poll (us):"))
        intr_layout.addWidget(self.spin_poll_us)
        intr_layout.addWidget(QLabel("Settle (us):"))
        intr_layout.addWidget(self.spin_settle_us)
        intr_layout.addStretch(1)
        root.addWidget(intr_group)

        # ---- Options ----
        opt_group = QGroupBox("Capture Options")
        opt_layout = QHBoxLayout(opt_group)

        self.cb_byteswap = QCheckBox("byteswap32 (ON recommended)")
        self.cb_byteswap.setChecked(True)

        self.cb_resync = QCheckBox("Resync on start (fix PREVIEW after Stop/Start)")
        self.cb_resync.setChecked(True)

        self.cb_3phase_clk = QCheckBox("3-Phase Clock (for 30MHz stability)")
        self.cb_3phase_clk.setChecked(True)  # Enable by default for high-speed stability
        self.cb_3phase_clk.setToolTip("Enable 3-phase clocking for more stable data sampling.\nRecommended for 30MHz. Slightly reduces throughput.")

        self.spin_flush = QSpinBox()
        self.spin_flush.setRange(0, 1000)
        self.spin_flush.setValue(16)

        self.spin_preview_every = QSpinBox()
        self.spin_preview_every.setRange(0, 1000)
        self.spin_preview_every.setValue(1)

        self.spin_log_every = QSpinBox()
        self.spin_log_every.setRange(1, 1000)
        self.spin_log_every.setValue(1)

        opt_layout.addWidget(self.cb_byteswap)
        opt_layout.addWidget(self.cb_resync)
        opt_layout.addWidget(self.cb_3phase_clk)
        opt_layout.addWidget(QLabel("Flush every N frames (0=off):"))
        opt_layout.addWidget(self.spin_flush)
        opt_layout.addWidget(QLabel("Preview every N frames (0=off):"))
        opt_layout.addWidget(self.spin_preview_every)
        opt_layout.addWidget(QLabel("Log every N frames:"))
        opt_layout.addWidget(self.spin_log_every)
        opt_layout.addStretch(1)
        root.addWidget(opt_group)

        # ---- Output Folder ----
        file_group = QGroupBox("Output Folder (1 file per frame)")
        file_layout = QHBoxLayout(file_group)

        self.cb_save = QCheckBox("Save frames")
        self.cb_save.setChecked(True)

        self.out_path = QLineEdit()
        self.out_path.setText(r"Z:\Texas_Instruments\AWRL6844\Python_App\capture_out")

        self.btn_browse = QPushButton("Browse...")
        self.btn_browse.clicked.connect(self._browse_folder)

        file_layout.addWidget(self.cb_save)
        file_layout.addWidget(self.out_path, 2)
        file_layout.addWidget(self.btn_browse)
        root.addWidget(file_group)

        # ---- Control Buttons ----
        ctrl_layout = QHBoxLayout()

        self.btn_start = QPushButton("Start Capture")
        self.btn_start.clicked.connect(self._start_capture)
        self.btn_start.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")

        self.btn_stop = QPushButton("Stop")
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self._stop_capture)
        self.btn_stop.setStyleSheet("background-color: #f44336; color: white;")

        self.progress = QProgressBar()
        self.progress.setTextVisible(True)

        ctrl_layout.addWidget(self.btn_start)
        ctrl_layout.addWidget(self.btn_stop)
        ctrl_layout.addWidget(self.progress, 2)
        root.addLayout(ctrl_layout)

        # ---- Log ----
        log_header = QHBoxLayout()
        log_header.addWidget(QLabel("Log:"))
        log_header.addStretch(1)
        btn_clear = QPushButton("Clear")
        btn_clear.clicked.connect(lambda: self.log_view.clear())
        log_header.addWidget(btn_clear)
        root.addLayout(log_header)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        root.addWidget(self.log_view, 1)

        self.status_label = QLabel("Ready")
        self.statusBar().addWidget(self.status_label, 1)

    def _log(self, msg: str):
        """
        로그 메시지를 타임스탬프와 함께 로그 뷰에 추가합니다

        【매개변수】
        - msg: 표시할 메시지

        【출력 형식】
        [HH:MM:SS.mmm] 메시지
        예: [14:30:25.123] Frame 10 captured
        """
        # 현재 시간을 문자열로 변환
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        # strftime: 시간을 지정된 형식의 문자열로 변환
        # %H: 시, %M: 분, %S: 초, %f: 마이크로초 (6자리)
        # [:-3]: 마이크로초 6자리 중 뒤 3자리 제거 → 밀리초(3자리)

        # 로그에 추가
        self.log_view.appendPlainText(f"[{ts}] {msg}")

        # 스크롤을 맨 아래로 이동 (새 메시지가 보이도록)
        c = self.log_view.textCursor()  # 현재 커서 가져오기
        c.movePosition(c.MoveOperation.End)  # 커서를 끝으로 이동
        self.log_view.setTextCursor(c)  # 커서 적용

    def _refresh_devices(self):
        """
        FTDI 장치 목록을 새로고침하고 콤보박스에 표시합니다

        【동작】
        1. list_ftdi_devices()로 장치 목록 가져오기
        2. SPI/GPIO 콤보박스에 장치 추가
        3. 기본값 설정 (SPI=0, GPIO=1)
        4. 로그에 장치 목록 출력
        5. 장치가 없으면 경고 메시지 표시
        """
        # 장치 목록 가져오기
        devices = list_ftdi_devices()

        # 콤보박스 초기화 (기존 항목 제거)
        self.spi_combo.clear()
        self.gpio_combo.clear()

        # 각 장치를 콤보박스에 추가
        for idx, desc, serial in devices:
            # 튜플 언패킹: (0, "FT4232H A", "ABC") → idx=0, desc="FT4232H A", serial="ABC"
            label = f"[{idx}] {desc} (SN: {serial})"
            self.spi_combo.addItem(label, idx)   # addItem(표시텍스트, 데이터)
            self.gpio_combo.addItem(label, idx)

        # 기본 선택: SPI=채널0, GPIO=채널1 (가능한 경우)
        if len(devices) >= 2:
            self.spi_combo.setCurrentIndex(0)   # 첫 번째 항목 선택
            self.gpio_combo.setCurrentIndex(1)  # 두 번째 항목 선택
        elif len(devices) == 1:
            self.spi_combo.setCurrentIndex(0)
            self.gpio_combo.setCurrentIndex(0)  # 하나뿐이면 같은 것 선택

        # 로그 출력
        self._log(f"Found {len(devices)} FTDI device(s)")
        for idx, desc, serial in devices:
            self._log(f"  [{idx}] {desc} (SN: {serial})")

        # 장치가 없으면 경고 메시지 표시
        if not devices:
            QMessageBox.warning(
                self, "Warning",  # 부모 윈도우, 제목
                "No FTDI devices found.\n\n"
                "Check:\n"
                "1) FT4232H cable connected\n"
                "2) FTDI D2XX drivers installed\n"
                "3) Device shows in Device Manager"
            )
            # QMessageBox.warning(): 경고 팝업 표시

    def _browse_folder(self):
        path = QFileDialog.getExistingDirectory(
            self, "Select Output Folder", self.out_path.text()
        )
        if path:
            self.out_path.setText(path)

    def _get_host_intr_mask(self) -> int:
        idx = self.intr_mask_combo.currentIndex()
        masks = [0x10, 0x20, 0x80, 0xA0]
        return masks[idx] if idx < len(masks) else 0xA0

    def _get_frame_size(self) -> int:
        return (self.spin_chirps.value() *
                self.spin_bursts.value() *
                self.spin_adc.value() *
                self.spin_rx.value() * 2)

    def _update_frame_size(self):
        size = self._get_frame_size()
        chunks = (size + FTDI_MAX_CHUNK - 1) // FTDI_MAX_CHUNK
        self.lbl_frame_size.setText(f"Frame size: {size:,} bytes  |  chunks: {chunks}")

    def _start_capture(self):
        """
        ═══════════════════════════════════════════════════════════════════════
        메서드명: _start_capture
        목적: 캡처를 시작합니다 (Start 버튼 클릭 시 호출)
        ═══════════════════════════════════════════════════════════════════════

        【동작 순서】
        1. 장치 확인
        2. GUI에서 설정값 수집 → CaptureSettings 생성
        3. 설정 정보 로그 출력
        4. UI 상태 변경 (Start 비활성화, Stop 활성화)
        5. 캡처 스레드 생성 및 시작

        【스레드 구조】
        MainWindow (GUI 스레드)
            │
            └── capture_thread (작업 스레드)
                    │
                    └── capture_worker.start() (실제 캡처 작업)
        """

        # --- 1. 장치 확인 ---
        if self.spi_combo.count() == 0:
            QMessageBox.warning(self, "Warning", "No FTDI devices found!")
            return  # 함수 종료

        # --- 2. GUI에서 설정값 수집 ---
        settings = CaptureSettings(
            spi_dev_index=self.spi_combo.currentData(),
            gpio_dev_index=self.gpio_combo.currentData(),
            clock_hz=int(self.clock_combo.currentText()),
            num_frames=self.spin_frames.value(),

            adc_samples=self.spin_adc.value(),
            chirps_per_burst=self.spin_chirps.value(),
            bursts_per_frame=self.spin_bursts.value(),
            rx_antennas=self.spin_rx.value(),

            host_intr_mask=self._get_host_intr_mask(),
            poll_sleep_us=self.spin_poll_us.value(),
            settle_us=self.spin_settle_us.value(),

            byteswap32=self.cb_byteswap.isChecked(),
            resync_on_start=self.cb_resync.isChecked(),
            use_3phase_clk=self.cb_3phase_clk.isChecked(),

            save_to_file=self.cb_save.isChecked(),
            out_path=self.out_path.text(),

            flush_every=self.spin_flush.value(),
            preview_every=self.spin_preview_every.value(),
            log_every=self.spin_log_every.value(),

            device_type=self.device_type_combo.currentText(),
        )

        frame_size = self._get_frame_size()
        chunks = (frame_size + FTDI_MAX_CHUNK - 1) // FTDI_MAX_CHUNK

        self._log("===== Capture Start =====")
        self._log(f"Device Type     : {settings.device_type}")
        self._log(f"SPI index       : {settings.spi_dev_index}")
        self._log(f"GPIO index      : {settings.gpio_dev_index}{' (separate)' if settings.gpio_dev_index!=settings.spi_dev_index else ''}")
        self._log(f"SPI Clock       : {settings.clock_hz/1e6:.1f} MHz")
        self._log(f"Frame Size      : {frame_size:,} bytes")
        self._log(f"Chunks/Frame    : {chunks}")
        self._log(f"HOST_INTR mask  : 0x{settings.host_intr_mask:02X}  (ready when GPIO&mask==0)")
        self._log(f"Byteswap32      : {'ON' if settings.byteswap32 else 'OFF'}")
        self._log(f"Resync          : {'ON' if settings.resync_on_start else 'OFF'}")
        self._log(f"3-Phase Clock   : {'ON' if settings.use_3phase_clk else 'OFF'}")
        self._log(f"Settle time     : {settings.settle_us} us")
        self._log(f"Flush every     : {settings.flush_every} frame(s)")
        self._log(f"Preview every   : {settings.preview_every} frame(s)")
        self._log(f"Output folder   : {settings.out_path if settings.save_to_file else '(not saving)'}")
        self._log("Waiting for sensor streaming...")

        # --- 3. UI 상태 변경 ---
        self.btn_start.setEnabled(False)  # Start 버튼 비활성화
        self.btn_stop.setEnabled(True)    # Stop 버튼 활성화

        # 진행률 바 설정
        if settings.num_frames > 0:
            self.progress.setMaximum(settings.num_frames)  # 최대값 설정
            self.progress.setValue(0)                       # 현재값 0
        else:
            # 무한 캡처: 최대값 0 = "무한 진행" 모드 (애니메이션)
            self.progress.setMaximum(0)
            self.progress.setValue(0)

        # --- 4. 캡처 스레드 생성 및 시작 ---
        # 【Qt 스레드 패턴: Worker + Thread】
        #
        # Worker 객체: 실제 작업을 수행하는 객체
        # QThread: Worker가 실행될 별도의 스레드
        #
        # 이렇게 분리하면:
        # - GUI는 메인 스레드에서 계속 반응
        # - 오래 걸리는 작업은 별도 스레드에서 실행

        self.capture_thread = QThread()  # 새 스레드 생성
        self.capture_worker = SpiCaptureWorker(settings)  # 워커 생성

        # 워커를 스레드로 이동 (이제 워커는 capture_thread에서 실행됨)
        self.capture_worker.moveToThread(self.capture_thread)

        # --- 시그널-슬롯 연결 ---
        # connect(): 시그널이 발생하면 지정된 함수를 호출하도록 연결
        #
        # 시그널                 → 슬롯 (호출될 함수)
        # ─────────────────────────────────────────────
        # 스레드 시작됨          → 워커.start() 호출
        # 워커.상태 메시지       → _on_status() (로그 출력)
        # 워커.에러 발생         → _on_error() (에러 표시)
        # 워커.진행률 변경       → _on_progress() (진행률 바 업데이트)
        # 워커.미리보기 데이터   → _on_preview() (미리보기 출력)
        # 워커.작업 완료         → _on_finished() (정리)

        self.capture_thread.started.connect(self.capture_worker.start)
        self.capture_worker.status.connect(self._on_status)
        self.capture_worker.error.connect(self._on_error)
        self.capture_worker.progress.connect(self._on_progress)
        self.capture_worker.preview.connect(self._on_preview)
        self.capture_worker.finished.connect(self._on_finished)

        # 스레드 시작! (이 시점부터 워커가 별도 스레드에서 실행됨)
        self.capture_thread.start()

    def _stop_capture(self):
        """Stop 버튼 클릭 시: 캡처 중지 요청"""
        if self.capture_worker:
            self.capture_worker.stop()  # _running = False 설정
        self._log("Stop requested...")

    # ════════════════════════════════════════════════════════════════════════
    # 시그널 핸들러 (슬롯) - 워커로부터 시그널을 받아 GUI 업데이트
    # ════════════════════════════════════════════════════════════════════════

    def _on_status(self, msg: str):
        """상태 메시지 시그널 핸들러: 상태바와 로그에 메시지 표시"""
        self.status_label.setText(msg)  # 하단 상태바 업데이트
        self._log(msg)                   # 로그에도 추가

    def _on_error(self, msg: str):
        """에러 시그널 핸들러: 로그 출력 + 팝업 표시"""
        self._log(f"ERROR: {msg}")
        QMessageBox.critical(self, "Error", msg)
        # QMessageBox.critical(): 에러 팝업 (빨간 X 아이콘)

    def _on_progress(self, current: int, total: int):
        """진행률 시그널 핸들러: 진행률 바 업데이트"""
        if total > 0:
            self.progress.setMaximum(total)
            self.progress.setValue(current)
        else:
            # 무한 캡처 모드: 0~99 반복하여 움직이는 것처럼 표시
            self.progress.setValue(current % 100)

    def _on_preview(self, data: bytes):
        """미리보기 시그널 핸들러: 프레임 데이터를 16진수로 로그에 출력"""
        # 각 바이트를 2자리 16진수로 변환하여 공백으로 연결
        hex_str = " ".join(f"{b:02X}" for b in data)
        # 예: bytes([0, 1, 255]) → "00 01 FF"
        self._log(f"[PREVIEW] {hex_str}")

    def _on_finished(self):
        """작업 완료 시그널 핸들러: 스레드 정리 및 UI 복원"""
        # 스레드 종료 대기
        if self.capture_thread:
            self.capture_thread.quit()       # 스레드에 종료 요청
            self.capture_thread.wait(2000)   # 최대 2초 대기

        # 참조 정리
        self.capture_thread = None
        self.capture_worker = None

        # UI 상태 복원
        self.btn_start.setEnabled(True)   # Start 버튼 활성화
        self.btn_stop.setEnabled(False)   # Stop 버튼 비활성화

        self._log("Capture finished")


def main():
    """
    ═══════════════════════════════════════════════════════════════════════════
    함수명: main
    목적: 프로그램의 진입점 (Entry Point)
    ═══════════════════════════════════════════════════════════════════════════

    【프로그램 실행 흐름】
    1. QApplication 생성 (Qt 초기화)
    2. MainWindow 생성 (GUI 구축)
    3. 창 표시 (show)
    4. 이벤트 루프 시작 (exec) - 사용자 입력 대기
    5. 창이 닫히면 종료
    """

    # --- Qt 애플리케이션 객체 생성 ---
    app = QApplication(sys.argv)
    # sys.argv: 명령줄 인자 (프로그램 실행 시 전달된 매개변수)
    # 예: python script.py --arg1 → sys.argv = ['script.py', '--arg1']

    # --- 메인 윈도우 생성 ---
    window = MainWindow()

    # --- 창 표시 ---
    window.show()
    # show(): 창을 화면에 표시

    # --- 이벤트 루프 시작 ---
    sys.exit(app.exec())
    # app.exec(): Qt 이벤트 루프 시작 (프로그램이 여기서 대기)
    #             - 마우스 클릭, 키보드 입력 등 이벤트 처리
    #             - 창이 닫히면 이 함수가 반환됨
    # sys.exit(): app.exec()의 반환값으로 프로그램 종료


# ============================================================================
# 프로그램 시작점
# ============================================================================
# Python에서 스크립트가 직접 실행될 때만 아래 코드가 실행됩니다.
# 다른 파일에서 import하면 실행되지 않습니다.
#
# 예:
# python ft4232h_cl_usb_gui.py  → __name__ == "__main__" → main() 실행
# import ft4232h_cl_usb_gui     → __name__ == "ft4232h_cl_usb_gui" → main() 안 실행

if __name__ == "__main__":
    main()
