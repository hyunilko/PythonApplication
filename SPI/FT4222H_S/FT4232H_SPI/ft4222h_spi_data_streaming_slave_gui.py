#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ft4222h_spi_data_streaming_slave_gui.py
======================================

FT4222H SPI SLAVE MODE GUI 애플리케이션

AWRL6844 레이더 센서(=SPI Master)로부터 SPI를 통해 ADC 데이터를 수신하는
GUI 기반 캡처 도구입니다. FT4222H는 SPI Slave로 동작하며, HOST_INTR(GPIO) 신호로
프레임 경계를 동기화합니다.

Firmware spec
-------------

- ``adcDataPerFrame`` = 32768 bytes (기본 32KB, 설정 가능)
- HOST_INTR LOW 신호당 단일 프레임 읽기

Host configuration (FT4222H as SPI Slave)
-----------------------------------------

- AWRL6844가 SPI Master (SCLK/CS 구동)
- HOST_INTR LOW 대기 → 프레임 데이터 읽기 → (옵션) HIGH 대기
- (옵션) ``byteswap32`` 적용하여 원본 바이트 스트림 복원

Frame pattern
-------------

- 패턴: ``XX XX XX XX 00 01 02 03 04 05 06 07``
- 바이트 0-3: 가변 (프레임 카운터 4바이트)
- 바이트 4-11: ``00 01 02 03 04 05 06 07`` (순차 패턴)

Dependencies
------------

- ``pip install ft4222 PyQt6``
- Windows: FTDI LibFT4222 / D2XX 드라이버 설치 필요

:author:
:date: 2026
"""

import sys
import time
import os
import glob
import array
from datetime import datetime
from dataclasses import dataclass
from typing import Optional, Any, Dict, List

import ft4222
from ft4222.SPI import Cpha, Cpol
from ft4222.GPIO import Dir, Port

from PyQt6.QtCore import QThread, pyqtSignal, QObject, pyqtSlot
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QPlainTextEdit, QLineEdit,
    QCheckBox, QMessageBox, QGroupBox, QFileDialog, QSpinBox,
    QProgressBar
)

# ---------------- Default sizes ----------------
DEFAULT_FRAME_SIZE = 32 * 1024   # 32KB default (configurable)
API_SAFE_MAX_READ = 65535


# ======================== FT4222 helpers ========================

def list_ft4222_devices() -> List[Dict[str, Any]]:
    """
    시스템에 연결된 FT4222 장치 목록을 반환합니다.

    FTDI FT4222 라이브러리를 사용하여 시스템에 연결된 모든 FT4222 장치를
    검색하고 정보를 수집합니다.

    :returns: 장치 정보 딕셔너리 리스트. 각 항목은 아래 키를 가집니다.

              - ``index`` (int): 장치 인덱스
              - ``desc`` (str): 장치 설명 문자열
              - ``serial`` (str): 시리얼 번호
    :rtype: list[dict[str, Any]]
    """
    out: List[Dict[str, Any]] = []
    try:
        n = ft4222.createDeviceInfoList()
    except Exception:
        return out

    for i in range(n):
        try:
            info = ft4222.getDeviceInfoDetail(i, update=False)
        except TypeError:
            info = ft4222.getDeviceInfoDetail(i)
        except Exception:
            continue

        desc_b = info.get("description", b"")
        ser_b = info.get("serial", b"")
        desc = desc_b.decode(errors="ignore") if isinstance(desc_b, (bytes, bytearray)) else str(desc_b)
        serial = ser_b.decode(errors="ignore") if isinstance(ser_b, (bytes, bytearray)) else str(ser_b)
        out.append({"index": i, "desc": desc, "serial": serial})
    return out


def open_by_index_compat(index: int):
    """
    인덱스로 FT4222 장치를 엽니다(호환성 폴백 포함).

    일부 환경에서는 ``ft4222.open(index)``가 지원되지 않을 수 있습니다.
    그 경우 device info의 ``location``을 사용해 ``openByLocation(location)`` 방식으로
    폴백합니다.

    :param int index: 열고자 하는 장치 인덱스
    :returns: 열린 FT4222 장치 핸들
    :raises RuntimeError: 장치 열기 실패 시
    """
    if hasattr(ft4222, "open"):
        return ft4222.open(index)
    try:
        try:
            info = ft4222.getDeviceInfoDetail(index, update=False)
        except TypeError:
            info = ft4222.getDeviceInfoDetail(index)
        loc = info.get("location") if isinstance(info, dict) else getattr(info, "location", None)
        if loc is None:
            raise RuntimeError(f"location not found for device index={index}")
        return ft4222.openByLocation(loc)
    except Exception as e:
        raise RuntimeError(f"open device by index={index} failed: {e}")


def open_by_desc_or_index(desc: str, index: Optional[int]):
    """
    FT4222 장치를 description으로 열고, 실패 시 index로 폴백합니다.

    :param str desc: 장치 description (예: ``"FT4222 A"``)
    :param int | None index: 폴백용 장치 인덱스 (None이면 폴백 없음)
    :returns: 열린 FT4222 장치 핸들
    :raises Exception: 모든 방법으로 열기 실패 시
    """
    try:
        return ft4222.openByDescription(desc)
    except TypeError:
        return ft4222.openByDescription(desc.encode("utf-8"))
    except Exception:
        if index is None:
            raise
        return open_by_index_compat(index)


def byteswap32_fast(data: bytes) -> bytes:
    """
    32비트 단위로 바이트 오더를 스왑합니다(빠른 버전).

    SPI로 수신한 big-endian 워드 스트림을 little-endian 호스트에서 동일한
    워드 값으로 해석하도록 만들기 위해 4바이트 단위로 뒤집습니다.

    .. note::
       입력 길이가 4의 배수가 아니면 ``0x00``으로 패딩 후 처리합니다.

    :param bytes data: 변환할 데이터
    :returns: 32비트 워드 단위로 byteswap된 데이터
    :rtype: bytes
    """
    if len(data) % 4 != 0:
        data = data + b"\x00" * (4 - (len(data) % 4))

    a = array.array("I")
    if a.itemsize != 4:
        b = bytearray(data)
        for i in range(0, len(b), 4):
            b[i:i + 4] = b[i:i + 4][::-1]
        return bytes(b)

    a.frombytes(data)
    a.byteswap()
    return a.tobytes()


def spi_mode_to_cpol_cpha(mode: int):
    """
    SPI mode(0~3)를 CPOL/CPHA 열거형 값으로 변환합니다.

    - Mode 0: CPOL=0, CPHA=0 (Idle LOW, Leading edge)
    - Mode 1: CPOL=0, CPHA=1 (Idle LOW, Trailing edge)
    - Mode 2: CPOL=1, CPHA=0 (Idle HIGH, Leading edge)
    - Mode 3: CPOL=1, CPHA=1 (Idle HIGH, Trailing edge)

    :param int mode: SPI 모드 (0~3)
    :returns: (CPOL, CPHA)
    :rtype: tuple[ft4222.SPI.Cpol, ft4222.SPI.Cpha]
    :raises ValueError: mode가 0~3 범위를 벗어난 경우
    """
    if mode == 0:
        return (Cpol.IDLE_LOW, Cpha.CLK_LEADING)
    if mode == 1:
        return (Cpol.IDLE_LOW, Cpha.CLK_TRAILING)
    if mode == 2:
        return (Cpol.IDLE_HIGH, Cpha.CLK_LEADING)
    if mode == 3:
        return (Cpol.IDLE_HIGH, Cpha.CLK_TRAILING)
    raise ValueError("SPI mode must be 0..3")


def port_enum(p: int) -> Port:
    """
    정수 포트 번호(0~3)를 FT4222 GPIO ``Port`` 열거형으로 변환합니다.

    :param int p: GPIO 포트 번호 (0~3)
    :returns: 해당 GPIO 포트 열거형 값
    :rtype: ft4222.GPIO.Port
    :raises ValueError: p가 0~3 범위를 벗어난 경우
    """
    if p == 0:
        return Port.P0
    if p == 1:
        return Port.P1
    if p == 2:
        return Port.P2
    if p == 3:
        return Port.P3
    raise ValueError("GPIO port must be 0..3")


def delete_existing_frame_files(out_dir: str, pattern: str = "adc_data_*.bin") -> int:
    """
    출력 디렉토리에서 이전 캡처 파일을 삭제합니다.

    지정된 디렉토리에서 ``pattern``에 매칭되는 파일을 모두 삭제합니다.
    디렉토리가 없으면 자동으로 생성합니다.

    :param str out_dir: 출력 디렉토리 경로
    :param str pattern: 삭제할 파일 패턴 (기본: ``adc_data_*.bin``)
    :returns: 삭제된 파일 개수
    :rtype: int
    """
    if not out_dir:
        return 0
    os.makedirs(out_dir, exist_ok=True)
    removed = 0
    for p in glob.glob(os.path.join(out_dir, pattern)):
        try:
            os.remove(p)
            removed += 1
        except Exception:
            pass
    return removed


def make_frame_path(out_dir: str, frame_idx_1based: int) -> str:
    """
    프레임 저장 파일 경로를 생성합니다.

    파일명 형식: ``adc_data_NNNNNN.bin`` (6자리, 1-based)

    :param str out_dir: 출력 디렉토리
    :param int frame_idx_1based: 프레임 번호(1부터 시작)
    :returns: 전체 파일 경로
    :rtype: str
    """
    return os.path.join(out_dir, f"adc_data_{frame_idx_1based:06d}.bin")


# ======================== GPIO helpers ========================

def gpio_init_input_all(dev_gpio):
    """
    GPIO P0~P3를 모두 입력 모드로 초기화합니다.

    FT4222 Python API 버전에 따라 ``gpio_Init`` 시그니처가 다를 수 있어
    keyword/positional 호출을 모두 지원합니다.

    :param dev_gpio: FT4222 GPIO 장치 핸들
    """
    try:
        dev_gpio.gpio_Init(gpio0=Dir.INPUT, gpio1=Dir.INPUT,
                           gpio2=Dir.INPUT, gpio3=Dir.INPUT)
    except TypeError:
        dev_gpio.gpio_Init(Dir.INPUT, Dir.INPUT, Dir.INPUT, Dir.INPUT)


def gpio_wait_level(dev_gpio, port, level_low, timeout_s,
                    stable_reads=0, stable_sleep_us=20):
    """
    GPIO 핀이 특정 레벨(LOW/HIGH)이 될 때까지 대기합니다.

    - 하드웨어 ``gpio_Wait``가 있으면 우선 사용합니다.
    - 없으면 소프트웨어 폴링으로 대기합니다.
    - ``stable_reads``를 지정하면 목표 레벨이 연속으로 유지되는지 추가 확인합니다.

    :param dev_gpio: FT4222 GPIO 장치 핸들
    :param port: 대기할 GPIO 포트
    :param bool level_low: True면 LOW 대기, False면 HIGH 대기
    :param float timeout_s: 타임아웃(초)
    :param int stable_reads: 연속 동일 레벨 확인 횟수(0이면 미사용)
    :param int stable_sleep_us: 안정화 확인 사이 sleep(마이크로초)
    :raises TimeoutError: 타임아웃 내에 목표 레벨에 도달하지 못한 경우
    """
    timeout_ms = int(timeout_s * 1000)
    target_level = level_low  # True = LOW, False = HIGH

    def is_level():
        v = dev_gpio.gpio_Read(port)
        is_low = (v is False) or (v == 0)
        return is_low == target_level

    # Hardware GPIO wait if available
    if hasattr(dev_gpio, "gpio_Wait"):
        deadline = time.perf_counter() + timeout_s
        while True:
            try:
                if hasattr(dev_gpio.gpio_Wait, '__call__'):
                    dev_gpio.gpio_Wait(port, target_level, timeout=timeout_ms, sleep=0)
                else:
                    dev_gpio.gpio_Wait(port, target_level, timeout_ms)
            except Exception:
                pass

            if stable_reads > 0:
                all_match = True
                for _ in range(stable_reads):
                    if not is_level():
                        all_match = False
                        break
                    if stable_sleep_us > 0:
                        time.sleep(stable_sleep_us / 1_000_000.0)
                if all_match:
                    return
                remaining = deadline - time.perf_counter()
                if remaining <= 0:
                    raise TimeoutError(f"HOST_INTR {'LOW' if target_level else 'HIGH'} timeout (port={port})")
                timeout_ms = int(remaining * 1000)
                continue
            else:
                return

    # Software polling fallback
    else:
        t0 = time.perf_counter()
        while True:
            if is_level():
                if stable_reads > 0:
                    ok = True
                    for _ in range(stable_reads):
                        if stable_sleep_us > 0:
                            time.sleep(stable_sleep_us / 1_000_000.0)
                        if not is_level():
                            ok = False
                            break
                    if ok:
                        return
                else:
                    return
            if (time.perf_counter() - t0) > timeout_s:
                state = "LOW" if target_level else "HIGH"
                raise TimeoutError(f"HOST_INTR {state} timeout (port={port})")
            time.sleep(0.0001)  # 100µs polling


def gpio_wait_low(dev_gpio, port: Port, timeout_s: float,
                  stable_reads: int = 0, stable_sleep_us: int = 10):
    """
    GPIO가 LOW 상태가 될 때까지 대기합니다(프레임 시작 신호).

    :param dev_gpio: FT4222 GPIO 장치 핸들
    :param ft4222.GPIO.Port port: 대기할 포트
    :param float timeout_s: 타임아웃(초)
    :param int stable_reads: 안정화 확인 횟수
    :param int stable_sleep_us: 안정화 확인 사이 sleep(마이크로초)
    :raises TimeoutError: 타임아웃 발생 시
    """
    return gpio_wait_level(dev_gpio, port, True, timeout_s,
                           stable_reads, stable_sleep_us)


def gpio_wait_high(dev_gpio, port: Port, timeout_s: float,
                   stable_reads: int = 0, stable_sleep_us: int = 10):
    """
    GPIO가 HIGH 상태가 될 때까지 대기합니다(프레임 종료/idle 신호).

    :param dev_gpio: FT4222 GPIO 장치 핸들
    :param ft4222.GPIO.Port port: 대기할 포트
    :param float timeout_s: 타임아웃(초)
    :param int stable_reads: 안정화 확인 횟수
    :param int stable_sleep_us: 안정화 확인 사이 sleep(마이크로초)
    :raises TimeoutError: 타임아웃 발생 시
    """
    return gpio_wait_level(dev_gpio, port, False, timeout_s,
                           stable_reads, stable_sleep_us)


# ======================== SPI Slave helpers ========================

def _find_spi_obj(dev_spi):
    """
    FT4222 디바이스에서 SPI Slave 기능을 제공하는 객체를 찾습니다(호환성).

    ft4222 바인딩/버전에 따라 SPI 관련 메서드가 다른 위치에 있을 수 있으므로,
    대표적인 후보 속성명을 검색하여 SPI Slave 메서드를 가진 객체를 반환합니다.

    :param dev_spi: FT4222 SPI 장치 핸들
    :returns: SPI Slave 메서드를 가진 객체(없으면 ``dev_spi`` 자체)
    """
    for name in ("spiSlave", "spislave", "SPI", "spi"):
        if hasattr(dev_spi, name):
            obj = getattr(dev_spi, name)
            if any(hasattr(obj, m) for m in ("spiSlave_InitEx", "spiSlave_Init",
                                             "spiSlave_Read", "spiSlave_GetRxStatus")):
                return obj
    return dev_spi


def spi_slave_get_rx_available(spi) -> Optional[int]:
    """
    SPI Slave RX 버퍼에서 읽을 수 있는 바이트 수를 조회합니다.

    다양한 API 버전과 호환되도록 여러 메서드명을 시도합니다
    (예: ``spiSlave_GetRxStatus``, ``spiSlave_GetRxSize``).

    :param spi: SPI Slave 객체
    :returns: 가용 바이트 수(조회 실패 시 None)
    :rtype: int | None
    """
    for name in ("spiSlave_GetRxStatus", "spiSlave_GetRxSize",
                 "spiSlave_GetRxBytes", "spiSlave_RxBytes"):
        if hasattr(spi, name):
            try:
                v = getattr(spi, name)()
                if isinstance(v, tuple):
                    for item in v[::-1]:
                        if isinstance(item, int):
                            return item
                    return None
                if isinstance(v, int):
                    return v
            except Exception:
                return None
    return None


def spi_slave_read_once(spi, nbytes: int) -> bytes:
    """
    SPI Slave RX 버퍼에서 데이터를 한 번 읽습니다(다양한 API 호환).

    :param spi: SPI Slave 객체
    :param int nbytes: 읽을 최대 바이트 수
    :returns: 읽은 데이터(가용 데이터가 없으면 빈 bytes)
    :rtype: bytes
    :raises RuntimeError: 읽기 API를 찾을 수 없거나 호출 실패 시
    """
    candidates = ("spiSlave_Read", "spiSlave_ReadEx",
                  "spiSlave_ReadBytes", "spiSlave_Receive")
    last_err = None
    for name in candidates:
        if hasattr(spi, name):
            fn = getattr(spi, name)
            try:
                r = fn(nbytes)
                if isinstance(r, tuple):
                    for item in r:
                        if isinstance(item, (bytes, bytearray)):
                            return bytes(item)
                    if len(r) >= 2 and isinstance(r[-1], (bytes, bytearray)):
                        return bytes(r[-1])
                    return b""
                return bytes(r)
            except TypeError:
                try:
                    r = fn(nbytes, 0)
                    if isinstance(r, tuple):
                        for item in r:
                            if isinstance(item, (bytes, bytearray)):
                                return bytes(item)
                        if len(r) >= 2 and isinstance(r[-1], (bytes, bytearray)):
                            return bytes(r[-1])
                        return b""
                    return bytes(r)
                except Exception as e:
                    last_err = e
            except Exception as e:
                last_err = e
    if last_err:
        raise RuntimeError(f"spi slave read failed: {last_err}")
    raise RuntimeError("no working spi slave read API found")


def spi_slave_read_exact(spi, nbytes: int, timeout_s: float) -> bytes:
    """
    지정된 바이트 수만큼 정확히 읽습니다(타임아웃 내).

    USB/드라이버 특성상 한 번의 read로 ``nbytes``가 모두 반환되지 않을 수 있어,
    내부적으로 반복 읽기를 수행하여 정확히 ``nbytes``를 수집합니다.

    :param spi: SPI Slave 객체
    :param int nbytes: 읽을 정확한 바이트 수
    :param float timeout_s: 타임아웃(초)
    :returns: 길이가 정확히 ``nbytes``인 데이터
    :rtype: bytes
    :raises TimeoutError: 타임아웃 내에 필요한 바이트를 모두 읽지 못한 경우
    """
    out = bytearray()
    t0 = time.perf_counter()
    while len(out) < nbytes:
        if (time.perf_counter() - t0) > timeout_s:
            raise TimeoutError(f"read_exact: got {len(out)}/{nbytes} bytes")
        remain = nbytes - len(out)
        want = min(remain, 4096, API_SAFE_MAX_READ)
        b = spi_slave_read_once(spi, want)
        if not b:
            time.sleep(0.0001)
            continue
        out.extend(b)
    return bytes(out)


def spi_slave_flush(spi, max_bytes: int = 2_000_000) -> int:
    """
    SPI Slave RX 버퍼의 잔여 데이터를 읽어서 폐기합니다(플러시).

    :param spi: SPI Slave 객체
    :param int max_bytes: 최대 플러시 바이트 수(무한 루프 방지)
    :returns: 플러시된 총 바이트 수
    :rtype: int
    """
    flushed = 0
    while flushed < max_bytes:
        avail = spi_slave_get_rx_available(spi)
        if avail is None or avail <= 0:
            break
        take = min(avail, 4096)
        b = spi_slave_read_once(spi, take)
        if not b:
            break
        flushed += len(b)
    return flushed


def spi_slave_flush_aggressive(spi, attempts: int = 5, delay_ms: float = 50,
                               max_bytes_per_attempt: int = 500_000) -> int:
    """
    강화 플러시를 수행합니다(여러 번 반복 + 지연).

    USB 버퍼링으로 인해 flush 직후에도 잔여 데이터가 조금 늦게 도착하는 경우가 있어,
    여러 번 반복하고 중간에 잠깐 대기합니다.

    :param spi: SPI Slave 객체
    :param int attempts: 시도 횟수
    :param float delay_ms: 시도 사이 대기(ms)
    :param int max_bytes_per_attempt: 회당 최대 플러시 바이트 수
    :returns: 총 플러시된 바이트 수
    :rtype: int
    """
    total_flushed = 0
    for i in range(attempts):
        flushed = spi_slave_flush(spi, max_bytes_per_attempt)
        total_flushed += flushed
        if flushed == 0 and i > 0:
            break
        if delay_ms > 0:
            time.sleep(delay_ms / 1000.0)
    return total_flushed


def check_frame_header_simple(data: bytes) -> bool:
    """
    프레임 헤더/패턴이 기대 형식인지 간단히 확인합니다.

    프레임 구조(테스트 패턴 기반):

    - byte[0..3]: 프레임 카운터(가변, 4바이트)
    - byte[4..]: 순차 패턴 ``(i-4) & 0xFF``

    본 검증은 최소 12바이트에서 아래 패턴만 확인합니다:

    ``XX XX XX XX 00 01 02 03 04 05 06 07``

    :param bytes data: 확인할 프레임 데이터(최소 12바이트 필요)
    :returns: 패턴 일치 여부
    :rtype: bool
    """
    if len(data) < 12:
        return False
    return (data[4] == 0x00 and data[5] == 0x01 and
            data[6] == 0x02 and data[7] == 0x03 and
            data[8] == 0x04 and data[9] == 0x05 and
            data[10] == 0x06 and data[11] == 0x07)


def spi_slave_init_stable(dev_spi, cpol, cpha, initex_override: Optional[int] = None):
    """
    SPI Slave 모드를 초기화합니다(다양한 API 버전 호환).

    초기화 순서:

    1. (가능하면) SPI reset 호출 (``spi_Reset``, ``spi_ResetTransaction``)
    2. ``spiSlave_InitEx`` 또는 ``spiSlave_Init`` 호출
    3. (가능하면) ``spiSlave_SetMode``로 CPOL/CPHA 반영

    :param dev_spi: FT4222 SPI 장치 핸들
    :param cpol: Clock Polarity (Cpol)
    :param cpha: Clock Phase (Cpha)
    :param int | None initex_override: ``spiSlave_InitEx`` 강제 값(없으면 1→2→0 자동 시도)
    :returns: 초기화된 SPI Slave 객체
    :raises RuntimeError: 초기화 실패 시
    """
    spi = _find_spi_obj(dev_spi)

    for rn in ("spi_Reset", "spi_ResetTransaction"):
        if hasattr(spi, rn):
            try:
                getattr(spi, rn)()
            except Exception:
                pass

    if hasattr(spi, "spiSlave_InitEx"):
        if initex_override is not None:
            r = spi.spiSlave_InitEx(int(initex_override))
            print(f"[SPI] spiSlave_InitEx({initex_override}) OK ret={r}")
        else:
            ok = False
            for cand in (1, 2, 0):
                try:
                    r = spi.spiSlave_InitEx(int(cand))
                    print(f"[SPI] spiSlave_InitEx({cand}) OK ret={r}")
                    ok = True
                    break
                except Exception as e:
                    print(f"[SPI] spiSlave_InitEx({cand}) failed: {e}")
            if not ok:
                raise RuntimeError("spiSlave_InitEx failed for 1,2,0")
    elif hasattr(spi, "spiSlave_Init"):
        try:
            spi.spiSlave_Init()
            print("[SPI] spiSlave_Init() OK")
        except TypeError:
            spi.spiSlave_Init(0)
            print("[SPI] spiSlave_Init(mode=0) OK")
    else:
        raise RuntimeError("No SPI slave init API found")

    if hasattr(spi, "spiSlave_SetMode"):
        r = spi.spiSlave_SetMode(cpol, cpha)
        print(f"[SPI] spiSlave_SetMode(CPOL={0 if cpol==Cpol.IDLE_LOW else 1}, "
              f"CPHA={0 if cpha==Cpha.CLK_LEADING else 1}) OK ret={r}")

    return spi


# ======================== Capture Worker ========================

@dataclass
class CaptureSettings:
    """
    캡처 설정을 담는 데이터 클래스입니다.

    :ivar int spi_dev_index: SPI 장치 인덱스
    :ivar int gpio_dev_index: GPIO 장치 인덱스
    :ivar str spi_desc: SPI 장치 description
    :ivar str gpio_desc: GPIO 장치 description
    :ivar int spi_mode: SPI 모드(0~3)
    :ivar int | None initex_mode: ``spiSlave_InitEx`` 모드(None이면 자동)
    :ivar int num_frames: 캡처할 프레임 수(0이면 무한)
    :ivar int frame_size: 프레임 크기(바이트)
    :ivar int host_intr_port: HOST_INTR GPIO 포트 번호(0~3)
    :ivar float timeout_s: GPIO 대기 타임아웃(초)
    :ivar int intr_stable_reads: 인터럽트 안정화 연속 읽기 횟수
    :ivar int intr_stable_sleep_us: 안정화 읽기 사이 지연(us)
    :ivar bool wait_intr_high: 프레임 후 HIGH 대기 여부
    :ivar bool full_reset: 시작 시 장치 close/reopen 수행 여부
    :ivar bool flush_before_start: 시작 전 FIFO 플러시 여부
    :ivar bool flush_each_frame: 매 프레임 플러시 여부
    :ivar bool sync_frame_boundary: 시작 시 프레임 경계 동기화(1프레임 스킵) 여부
    :ivar bool save_to_file: 파일 저장 여부
    :ivar str out_dir: 출력 디렉토리
    :ivar bool overwrite_on_start: 시작 시 기존 파일 삭제 여부
    :ivar int preview_every: 프리뷰 출력 간격(프레임 단위)
    :ivar int log_every: 로그 출력 간격(프레임 단위)
    :ivar bool byteswap32: byteswap32 적용 여부
    :ivar float spi_read_timeout_s: SPI 읽기 타임아웃(초)
    :ivar bool fast_mode: True면 메모리 버퍼링 후 종료 시 일괄 저장
    """
    spi_dev_index: int
    gpio_dev_index: int
    spi_desc: str
    gpio_desc: str

    spi_mode: int
    initex_mode: Optional[int]

    num_frames: int  # 0=infinite
    frame_size: int

    host_intr_port: int
    timeout_s: float
    intr_stable_reads: int
    intr_stable_sleep_us: int

    wait_intr_high: bool
    full_reset: bool
    flush_before_start: bool
    flush_each_frame: bool
    sync_frame_boundary: bool

    save_to_file: bool
    out_dir: str
    overwrite_on_start: bool

    preview_every: int
    log_every: int
    byteswap32: bool
    spi_read_timeout_s: float
    fast_mode: bool  # 메모리에 버퍼링 후 나중에 저장


class SpiCaptureWorker(QObject):
    """
    SPI Slave 캡처 워커(스레드에서 실행되는 QObject)입니다.

    별도 스레드에서 실행되며 AWRL6844로부터 프레임 데이터를 수신합니다.
    Qt의 시그널-슬롯 메커니즘을 사용하여 GUI와 통신합니다.

    Signals
    -------

    - ``status(str)``: 상태 메시지
    - ``error(str)``: 오류 메시지(트레이스 포함 가능)
    - ``progress(int, int)``: 진행률(현재, 전체; 전체가 0이면 무한 모드)
    - ``preview(bytes)``: 프레임 미리보기 데이터(앞부분)
    - ``finished()``: 캡처 완료

    :ivar CaptureSettings settings: 캡처 설정
    :ivar int frame_timeout_cnt: 프레임 타임아웃 횟수
    :ivar int chunk_timeout_cnt: 읽기 타임아웃 횟수
    """
    status = pyqtSignal(str)
    error = pyqtSignal(str)
    progress = pyqtSignal(int, int)
    preview = pyqtSignal(bytes)
    finished = pyqtSignal()

    def __init__(self, settings: CaptureSettings):
        """
        :param CaptureSettings settings: 캡처 설정
        """
        super().__init__()
        self.settings = settings
        self._running = False
        self._dev_spi = None
        self._dev_gpio = None
        self._spi = None

        # Stats
        self.frame_timeout_cnt = 0
        self.chunk_timeout_cnt = 0

    @pyqtSlot()
    def start(self):
        """
        캡처를 시작합니다(스레드에서 실행).

        장치 초기화 → 동기화 → 프레임 수신 → (옵션) 파일 저장/버퍼링을 수행합니다.

        :raises RuntimeError: 초기 동기화 실패 등 치명적 조건에서 발생
        """
        self._running = True

        try:
            self.status.emit("=" * 60)
            self.status.emit("    FT4222H SPI SLAVE Capture (GUI Mode)")
            self.status.emit("=" * 60)
            self.status.emit("Mode           : SPI SLAVE")
            self.status.emit("                 AWRL6844 = Master (drives SCLK/CS)")
            self.status.emit("                 FT4222H  = Slave  (receives data)")
            self.status.emit("-" * 60)
            self.status.emit(f"SPI mode       : {self.settings.spi_mode} (CPOL/CPHA)")
            self.status.emit(f"HOST_INTR      : P{self.settings.host_intr_port} (LOW = data ready)")
            self.status.emit(f"Wait INTR HIGH : {'YES' if self.settings.wait_intr_high else 'NO'}")
            self.status.emit(f"Frame size     : {self.settings.frame_size:,} bytes")
            self.status.emit(f"Byteswap32     : {'ON' if self.settings.byteswap32 else 'OFF'} (host={sys.byteorder}-endian)")
            self.status.emit(f"Full reset     : {'ON' if self.settings.full_reset else 'OFF'}")
            self.status.emit("=" * 60)

            if self.settings.save_to_file:
                if not self.settings.out_dir:
                    raise RuntimeError("Output folder is empty. Please select an output folder.")
                os.makedirs(self.settings.out_dir, exist_ok=True)
                if self.settings.overwrite_on_start:
                    removed = delete_existing_frame_files(self.settings.out_dir, "adc_data_*.bin")
                    self.status.emit(f"Deleted {removed} existing file(s) in: {self.settings.out_dir}")
                self.status.emit(f"Saving per-frame files into: {self.settings.out_dir}")

            # Open devices
            self._dev_spi = open_by_desc_or_index(self.settings.spi_desc, self.settings.spi_dev_index)
            self._dev_gpio = open_by_desc_or_index(self.settings.gpio_desc, self.settings.gpio_dev_index)

            # Full reset if enabled
            if self.settings.full_reset:
                self.status.emit("[RESET] Full device close/reopen...")
                try:
                    self._dev_spi.close()
                except Exception:
                    pass
                try:
                    self._dev_gpio.close()
                except Exception:
                    pass
                time.sleep(1.0)  # 1초 대기 (USB 리셋 및 버퍼 완전 정리)

                self._dev_spi = open_by_desc_or_index(self.settings.spi_desc, self.settings.spi_dev_index)
                self._dev_gpio = open_by_desc_or_index(self.settings.gpio_desc, self.settings.gpio_dev_index)
                time.sleep(0.2)  # 재오픈 후 안정화
                self.status.emit("[RESET] Devices reopened.")

            # Set system clock
            if hasattr(self._dev_spi, "setClock") and hasattr(ft4222, "SysClock"):
                try:
                    self._dev_spi.setClock(ft4222.SysClock.CLK_80)
                    self.status.emit("[CLK] setClock(SysClock.CLK_80) OK")
                except Exception as e:
                    self.status.emit(f"[CLK] setClock failed (ignored): {e}")

            # GPIO init
            gpio_init_input_all(self._dev_gpio)
            intr_port = port_enum(self.settings.host_intr_port)

            # SPI Slave init
            cpol, cpha = spi_mode_to_cpol_cpha(self.settings.spi_mode)
            self._spi = spi_slave_init_stable(self._dev_spi, cpol, cpha, initex_override=self.settings.initex_mode)

            # Sync: wait for idle HIGH state
            try:
                gpio_wait_high(self._dev_gpio, intr_port, self.settings.timeout_s)
                self.status.emit("[SYNC] HOST_INTR is HIGH (idle) - OK")
            except TimeoutError:
                raise RuntimeError("HOST_INTR is not HIGH at start. "
                                   "Please ensure master is idle before running.")

            # Stabilization delay and aggressive flush
            time.sleep(0.01)  # 10ms 안정화 시간

            if self.settings.flush_before_start:
                flushed = spi_slave_flush_aggressive(self._spi, attempts=5, delay_ms=30)
                if flushed > 0:
                    self.status.emit(f"[SPI] Aggressive flush: {flushed} bytes discarded")

            # 프레임 경계 동기화 (중요!)
            if self.settings.sync_frame_boundary:
                self.status.emit("[SYNC] Synchronizing to frame boundary...")
                frame_size = self.settings.frame_size

                # Step 1: 현재 상태 확인
                current_level = self._dev_gpio.gpio_Read(intr_port)
                is_low = (current_level is False) or (current_level == 0)
                self.status.emit(f"[SYNC] Current HOST_INTR: {'LOW (frame active)' if is_low else 'HIGH (idle)'}")

                if is_low:
                    # 프레임 진행 중 - 끝날 때까지 대기
                    self.status.emit("[SYNC] Waiting for current frame to end (HIGH)...")
                    try:
                        gpio_wait_high(self._dev_gpio, intr_port, self.settings.timeout_s)
                        self.status.emit("[SYNC] Frame ended (HIGH)")
                    except TimeoutError:
                        self.status.emit("[WARNING] Timeout waiting for HIGH")

                # Step 2: HIGH 상태에서 FIFO 완전 플러시
                time.sleep(0.03)  # 30ms 대기 - USB 파이프라인 데이터 도착 대기
                flush_total = 0
                for _ in range(10):
                    flushed = spi_slave_flush(self._spi, 200_000)
                    flush_total += flushed
                    if flushed == 0:
                        break
                    time.sleep(0.02)

                if flush_total > 0:
                    self.status.emit(f"[SYNC] Flushed {flush_total} bytes (previous frame data)")

                # Step 3: 한 프레임 전체를 정확히 읽고 버림 (크기 기반 동기화)
                self.status.emit(f"[SYNC] Skipping one full frame ({frame_size} bytes)...")
                try:
                    # LOW 대기 (프레임 시작)
                    gpio_wait_low(self._dev_gpio, intr_port, self.settings.timeout_s)

                    # 정확히 frame_size만큼 읽기 (버림)
                    skip_data = spi_slave_read_exact(self._spi, frame_size, self.settings.spi_read_timeout_s)
                    self.status.emit(f"[SYNC] Read and discarded {len(skip_data)} bytes")

                    # 디버그: 스킵된 프레임의 첫 부분 출력
                    if self.settings.byteswap32:
                        skip_swapped = byteswap32_fast(skip_data[:32])
                    else:
                        skip_swapped = skip_data[:32]
                    self.status.emit(f"[SYNC] Skipped frame start: {' '.join(f'{b:02X}' for b in skip_swapped[:16])}")

                    # HIGH 대기 (프레임 종료 확인)
                    gpio_wait_high(self._dev_gpio, intr_port, self.settings.timeout_s)
                    self.status.emit("[SYNC] Frame boundary confirmed (HIGH)")

                except TimeoutError as e:
                    self.status.emit(f"[WARNING] Frame skip timeout: {e}")
                    spi_slave_flush(self._spi, 500_000)

                # Step 4: 최종 플러시
                time.sleep(0.02)
                final_flushed = spi_slave_flush(self._spi, 100_000)
                if final_flushed > 0:
                    self.status.emit(f"[SYNC] Final cleanup: {final_flushed} bytes")

                self.status.emit("[SYNC] Ready - aligned at frame boundary")

            self.status.emit("\nWaiting for AWRL6844 (Master) to send data...\n")

            # Capture loop
            frame_count = 0
            total_bytes = 0
            start_t = time.perf_counter()

            target_frames = self.settings.num_frames if self.settings.num_frames > 0 else float("inf")
            total_for_progress = int(target_frames) if target_frames != float("inf") else 0

            last_stat_time = start_t
            last_stat_bytes = 0
            last_stat_frames = 0

            # Fast mode: 메모리 버퍼링
            frame_buffer = [] if self.settings.fast_mode else None
            if self.settings.fast_mode:
                self.status.emit("[MODE] Fast mode ON - buffering frames in memory")

            while self._running and frame_count < target_frames:
                if self.settings.flush_each_frame:
                    try:
                        gpio_wait_high(self._dev_gpio, intr_port, self.settings.timeout_s)
                    except TimeoutError:
                        pass
                    spi_slave_flush(self._spi)

                # Wait for LOW (frame start)
                try:
                    gpio_wait_low(self._dev_gpio, intr_port, self.settings.timeout_s,
                                  stable_reads=self.settings.intr_stable_reads,
                                  stable_sleep_us=self.settings.intr_stable_sleep_us)
                except TimeoutError:
                    self.frame_timeout_cnt += 1
                    self.status.emit(f"[FRAME {frame_count+1}] Timeout waiting INTR LOW")
                    continue

                # Read frame data
                try:
                    raw_frame = spi_slave_read_exact(self._spi, self.settings.frame_size,
                                                     self.settings.spi_read_timeout_s)
                except TimeoutError as e:
                    self.chunk_timeout_cnt += 1
                    self.status.emit(f"[FRAME {frame_count+1}] Read timeout: {e}")
                    continue

                # Wait for HIGH (frame end) if enabled
                if self.settings.wait_intr_high:
                    try:
                        gpio_wait_high(self._dev_gpio, intr_port, self.settings.timeout_s,
                                       stable_reads=self.settings.intr_stable_reads,
                                       stable_sleep_us=self.settings.intr_stable_sleep_us)
                    except TimeoutError:
                        self.status.emit(f"[FRAME {frame_count+1}] Warning: INTR HIGH timeout")

                # Byteswap if enabled
                if self.settings.byteswap32:
                    frame = byteswap32_fast(raw_frame)
                else:
                    frame = raw_frame

                # 첫 프레임 디버그 출력
                if frame_count == 0:
                    self.status.emit(f"[DEBUG] First frame RAW  (32B): {' '.join(f'{b:02X}' for b in raw_frame[:32])}")
                    self.status.emit(f"[DEBUG] First frame SWAP (32B): {' '.join(f'{b:02X}' for b in frame[:32])}")
                    if check_frame_header_simple(frame):
                        self.status.emit("[DEBUG] Frame header pattern: OK")
                    else:
                        self.status.emit("[DEBUG] Frame header pattern: MISMATCH (data may be misaligned)")

                frame_count += 1
                total_bytes += len(frame)

                # Save to file or buffer
                if self.settings.save_to_file:
                    if self.settings.fast_mode:
                        frame_buffer.append(frame)
                    else:
                        out_path = make_frame_path(self.settings.out_dir, frame_count)
                        with open(out_path, "wb") as f:
                            f.write(frame)

                self.progress.emit(frame_count, total_for_progress)

                # Preview
                if self.settings.preview_every > 0 and (frame_count % self.settings.preview_every) == 0:
                    self.preview.emit(frame[:64])

                # Stats logging
                if self.settings.log_every > 0 and (frame_count % self.settings.log_every) == 0:
                    now = time.perf_counter()
                    elapsed_interval = now - last_stat_time
                    interval_bytes = total_bytes - last_stat_bytes
                    interval_frames = frame_count - last_stat_frames

                    interval_mbps = (interval_bytes / (1024 * 1024)) / elapsed_interval if elapsed_interval > 0 else 0.0
                    interval_fps = interval_frames / elapsed_interval if elapsed_interval > 0 else 0.0

                    total_elapsed = now - start_t
                    total_mbps = (total_bytes / (1024 * 1024)) / total_elapsed if total_elapsed > 0 else 0.0
                    total_fps = frame_count / total_elapsed if total_elapsed > 0 else 0.0

                    self.status.emit(
                        f"[SLAVE] Frame {frame_count}: "
                        f"Interval({interval_mbps:.2f} MB/s, {interval_fps:.1f} fps) | "
                        f"Total({total_mbps:.2f} MB/s, {total_fps:.1f} fps, {total_bytes/(1024*1024):.1f} MB) | "
                        f"TO_frm={self.frame_timeout_cnt} TO_chk={self.chunk_timeout_cnt}"
                    )

                    last_stat_time = now
                    last_stat_bytes = total_bytes
                    last_stat_frames = frame_count

            # Fast mode: 캡처 완료 후 일괄 저장
            if self.settings.fast_mode and self.settings.save_to_file and frame_buffer:
                self.status.emit(f"\n[SAVE] Writing {len(frame_buffer)} frames to disk...")
                save_start = time.perf_counter()
                for i, frm in enumerate(frame_buffer, start=1):
                    out_path = make_frame_path(self.settings.out_dir, i)
                    with open(out_path, "wb") as f:
                        f.write(frm)
                    if i % 20 == 0:
                        self.status.emit(f"[SAVE] Progress: {i}/{len(frame_buffer)}")
                        self.progress.emit(i, len(frame_buffer))
                save_elapsed = time.perf_counter() - save_start
                self.status.emit(f"[SAVE] Completed in {save_elapsed:.2f}s")
                frame_buffer.clear()  # 메모리 해제

            # Summary
            elapsed = time.perf_counter() - start_t
            fps = (frame_count / elapsed) if elapsed > 0 else 0.0
            mbps = (total_bytes / (1024 * 1024)) / elapsed if elapsed > 0 else 0.0

            self.status.emit("\n" + "=" * 60)
            self.status.emit("           CAPTURE SUMMARY (SPI SLAVE MODE)")
            self.status.emit("=" * 60)
            self.status.emit(f"Frames captured : {frame_count}")
            self.status.emit(f"Total bytes     : {total_bytes:,} ({total_bytes/(1024*1024):.2f} MB)")
            self.status.emit(f"Capture time    : {elapsed:.2f} s")
            self.status.emit(f"Throughput      : {mbps:.2f} MB/s")
            self.status.emit(f"Frame rate      : {fps:.2f} fps")
            self.status.emit(f"Frame timeouts  : {self.frame_timeout_cnt}")
            self.status.emit(f"Chunk timeouts  : {self.chunk_timeout_cnt}")
            self.status.emit(f"Output dir      : {self.settings.out_dir}")
            self.status.emit("=" * 60)

        except Exception as e:
            import traceback
            self.error.emit(f"Capture error: {e}\n{traceback.format_exc()}")

        finally:
            self._cleanup()
            self.finished.emit()

    @pyqtSlot()
    def stop(self):
        """
        캡처 중지를 요청합니다.

        캡처 루프를 중지하도록 플래그를 설정합니다.
        현재 진행 중인 프레임 처리가 완료된 후 중지됩니다.
        """
        self._running = False

    def _cleanup(self):
        """
        장치 정리 및 리소스 해제.

        SPI/GPIO 장치 핸들을 닫고 내부 참조를 초기화합니다.
        """
        try:
            if self._dev_spi:
                self._dev_spi.close()
        except Exception:
            pass
        try:
            if self._dev_gpio and self._dev_gpio != self._dev_spi:
                self._dev_gpio.close()
        except Exception:
            pass

        self._dev_spi = None
        self._dev_gpio = None
        self._spi = None
        self.status.emit("Devices closed")


# ======================== GUI ========================

class MainWindow(QMainWindow):
    """
    메인 윈도우 클래스입니다.

    AWRL6844 SPI 데이터 캡처 도구의 GUI를 구현합니다.
    FT4222H를 SPI Slave로 사용하여 데이터를 수신합니다.

    :ivar QThread | None capture_thread: 캡처 워커를 실행하는 QThread
    :ivar SpiCaptureWorker | None capture_worker: SPI 캡처 워커 인스턴스
    """

    def __init__(self):
        """윈도우를 초기화하고 UI를 구성한 뒤 장치 목록을 새로고침합니다."""
        super().__init__()
        self.setWindowTitle("AWRL6844 SPI Data Capture Tool (FT4222H SPI SLAVE)")
        self.resize(1100, 900)

        self.capture_thread: Optional[QThread] = None
        self.capture_worker: Optional[SpiCaptureWorker] = None

        self._build_ui()
        self._refresh_devices()

    def _build_ui(self):
        """UI 컴포넌트를 생성하고 레이아웃을 구성합니다."""
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # Device selection
        dev_group = QGroupBox("FT4222H Device Selection")
        dev_layout = QHBoxLayout(dev_group)

        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self._refresh_devices)

        self.spi_combo = QComboBox()
        self.gpio_combo = QComboBox()

        dev_layout.addWidget(self.btn_refresh)
        dev_layout.addWidget(QLabel("SPI Device:"))
        dev_layout.addWidget(self.spi_combo, 2)
        dev_layout.addWidget(QLabel("GPIO Device:"))
        dev_layout.addWidget(self.gpio_combo, 2)
        root.addWidget(dev_group)

        # Mode info
        mode_group = QGroupBox("SPI Slave Mode Info")
        mode_layout = QHBoxLayout(mode_group)
        lbl = QLabel("FT4222H = SPI SLAVE  |  AWRL6844 = SPI MASTER (drives SCLK/CS)")
        lbl.setStyleSheet("font-weight: bold; color: #0066cc;")
        mode_layout.addWidget(lbl)
        mode_layout.addStretch(1)
        root.addWidget(mode_group)

        # HOST_INTR
        intr_group = QGroupBox("HOST_INTR GPIO Configuration")
        intr_layout = QHBoxLayout(intr_group)

        self.intr_port_combo = QComboBox()
        self.intr_port_combo.addItems(["P0", "P1", "P2", "P3"])
        self.intr_port_combo.setCurrentIndex(3)

        self.spin_timeout = QSpinBox()
        self.spin_timeout.setRange(1, 60)
        self.spin_timeout.setValue(5)

        self.spin_stable_reads = QSpinBox()
        self.spin_stable_reads.setRange(0, 100)
        self.spin_stable_reads.setValue(0)
        self.spin_stable_reads.setToolTip("Number of consecutive reads to confirm level stability")

        self.spin_stable_sleep = QSpinBox()
        self.spin_stable_sleep.setRange(0, 1000)
        self.spin_stable_sleep.setValue(10)
        self.spin_stable_sleep.setToolTip("Delay (us) between stability reads")

        intr_layout.addWidget(QLabel("HOST_INTR Port:"))
        intr_layout.addWidget(self.intr_port_combo)
        intr_layout.addWidget(QLabel("Wait timeout (s):"))
        intr_layout.addWidget(self.spin_timeout)
        intr_layout.addWidget(QLabel("Stable reads:"))
        intr_layout.addWidget(self.spin_stable_reads)
        intr_layout.addWidget(QLabel("Stable sleep (us):"))
        intr_layout.addWidget(self.spin_stable_sleep)
        intr_layout.addStretch(1)
        root.addWidget(intr_group)

        # Capture Options
        opt_group = QGroupBox("Capture Options")
        opt_layout = QHBoxLayout(opt_group)

        self.spi_mode_combo = QComboBox()
        self.spi_mode_combo.addItems(["0", "1", "2", "3"])
        self.spi_mode_combo.setCurrentText("0")

        self.initex_combo = QComboBox()
        self.initex_combo.addItems(["Auto (1,2,0)", "0", "1", "2"])
        self.initex_combo.setCurrentIndex(0)
        self.initex_combo.setToolTip("spiSlave_InitEx protocol mode")

        self.spin_frame_size = QSpinBox()
        self.spin_frame_size.setRange(1024, 1024*1024)
        self.spin_frame_size.setValue(DEFAULT_FRAME_SIZE)
        self.spin_frame_size.setSingleStep(1024)
        self.spin_frame_size.setToolTip("Frame size in bytes (32KB = 32768)")

        self.spin_frames = QSpinBox()
        self.spin_frames.setRange(0, 1000000)
        self.spin_frames.setValue(100)
        self.spin_frames.setSpecialValueText("Infinite")

        self.spin_read_timeout = QSpinBox()
        self.spin_read_timeout.setRange(1, 60)
        self.spin_read_timeout.setValue(10)
        self.spin_read_timeout.setToolTip("Timeout for reading frame data")

        opt_layout.addWidget(QLabel("SPI Mode:"))
        opt_layout.addWidget(self.spi_mode_combo)
        opt_layout.addWidget(QLabel("InitEx:"))
        opt_layout.addWidget(self.initex_combo)
        opt_layout.addWidget(QLabel("Frame Size:"))
        opt_layout.addWidget(self.spin_frame_size)
        opt_layout.addWidget(QLabel("Frames (0=inf):"))
        opt_layout.addWidget(self.spin_frames)
        opt_layout.addWidget(QLabel("Read timeout (s):"))
        opt_layout.addWidget(self.spin_read_timeout)
        opt_layout.addStretch(1)
        root.addWidget(opt_group)

        # Slave-specific options
        slave_group = QGroupBox("Slave Mode Options")
        slave_layout = QHBoxLayout(slave_group)

        self.cb_wait_high = QCheckBox("Wait INTR HIGH after frame")
        self.cb_wait_high.setChecked(True)

        self.cb_full_reset = QCheckBox("Full reset on start")
        self.cb_full_reset.setChecked(True)

        self.cb_flush_before = QCheckBox("Flush RX before start")
        self.cb_flush_before.setChecked(True)

        self.cb_flush_each = QCheckBox("Flush each frame")
        self.cb_flush_each.setChecked(False)

        self.cb_sync_boundary = QCheckBox("Frame boundary sync")
        self.cb_sync_boundary.setChecked(True)
        self.cb_sync_boundary.setToolTip("Skip one frame at start for clean synchronization (recommended)")

        self.cb_byteswap = QCheckBox("Byteswap32")
        self.cb_byteswap.setChecked(False)

        self.spin_preview = QSpinBox()
        self.spin_preview.setRange(0, 1000)
        self.spin_preview.setValue(1)

        self.spin_log = QSpinBox()
        self.spin_log.setRange(1, 1000)
        self.spin_log.setValue(10)

        slave_layout.addWidget(self.cb_wait_high)
        slave_layout.addWidget(self.cb_full_reset)
        slave_layout.addWidget(self.cb_flush_before)
        slave_layout.addWidget(self.cb_flush_each)
        slave_layout.addWidget(self.cb_sync_boundary)
        slave_layout.addWidget(self.cb_byteswap)
        slave_layout.addWidget(QLabel("Preview every:"))
        slave_layout.addWidget(self.spin_preview)
        slave_layout.addWidget(QLabel("Log every:"))
        slave_layout.addWidget(self.spin_log)
        slave_layout.addStretch(1)
        root.addWidget(slave_group)

        # Output folder
        file_group = QGroupBox("Output Folder (per-frame files)")
        file_layout = QHBoxLayout(file_group)

        self.cb_save = QCheckBox("Save to folder")
        self.cb_save.setChecked(True)

        self.cb_fast_mode = QCheckBox("Fast mode")
        self.cb_fast_mode.setChecked(True)
        self.cb_fast_mode.setToolTip("Buffer frames in memory, write all at end (prevents I/O delays)")

        self.cb_overwrite = QCheckBox("Overwrite on start")
        self.cb_overwrite.setChecked(True)

        self.out_dir = QLineEdit()
        self.out_dir.setText(os.path.abspath("capture_out"))

        self.btn_browse = QPushButton("Browse folder...")
        self.btn_browse.clicked.connect(self._browse_folder)

        file_layout.addWidget(self.cb_save)
        file_layout.addWidget(self.cb_fast_mode)
        file_layout.addWidget(self.cb_overwrite)
        file_layout.addWidget(QLabel("Output folder:"))
        file_layout.addWidget(self.out_dir, 2)
        file_layout.addWidget(self.btn_browse)
        root.addWidget(file_group)

        # Control
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

        # Log
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
        로그 메시지를 타임스탬프와 함께 출력합니다.

        :param str msg: 출력할 로그 메시지
        """
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_view.appendPlainText(f"[{ts}] {msg}")
        c = self.log_view.textCursor()
        c.movePosition(c.MoveOperation.End)
        self.log_view.setTextCursor(c)

    def _refresh_devices(self):
        """FT4222 장치 목록을 새로고침하고 콤보박스를 업데이트합니다."""
        devices = list_ft4222_devices()
        self.spi_combo.clear()
        self.gpio_combo.clear()

        for d in devices:
            idx = d["index"]
            desc = d["desc"]
            serial = d["serial"]
            label = f"[{idx}] {desc} (SN: {serial})"
            user = {"index": idx, "desc": desc}
            self.spi_combo.addItem(label, user)
            self.gpio_combo.addItem(label, user)

        if len(devices) >= 2:
            self.spi_combo.setCurrentIndex(0)
            self.gpio_combo.setCurrentIndex(1)
        elif len(devices) == 1:
            self.spi_combo.setCurrentIndex(0)
            self.gpio_combo.setCurrentIndex(0)

        self._log(f"Found {len(devices)} FT4222 device(s)")
        for d in devices:
            self._log(f"  [{d['index']}] {d['desc']} (SN: {d['serial']})")

        if not devices:
            QMessageBox.warning(
                self, "Warning",
                "No FT4222 devices found.\n\n"
                "Check:\n"
                "1) UMFT4222EV connected\n"
                "2) LibFT4222 / D2XX installed\n"
                "3) Device shows in Device Manager"
            )

    def _browse_folder(self):
        """출력 폴더 선택 다이얼로그를 표시합니다."""
        path = QFileDialog.getExistingDirectory(self, "Select Output Folder", self.out_dir.text())
        if path:
            self.out_dir.setText(path)

    def _start_capture(self):
        """캡처 시작 버튼 핸들러: 설정을 수집하고 워커 스레드를 시작합니다."""
        if self.spi_combo.count() == 0:
            QMessageBox.warning(self, "Warning", "No FT4222 devices found!")
            return

        spi_ud = self.spi_combo.currentData()
        gpio_ud = self.gpio_combo.currentData()
        if not isinstance(spi_ud, dict) or not isinstance(gpio_ud, dict):
            QMessageBox.critical(self, "Error", "Device selection data invalid.")
            return

        out_dir = self.out_dir.text().strip()

        # Parse initex mode
        initex_text = self.initex_combo.currentText()
        initex_mode = None
        if initex_text != "Auto (1,2,0)":
            initex_mode = int(initex_text)

        settings = CaptureSettings(
            spi_dev_index=int(spi_ud["index"]),
            gpio_dev_index=int(gpio_ud["index"]),
            spi_desc=str(spi_ud["desc"]),
            gpio_desc=str(gpio_ud["desc"]),
            spi_mode=int(self.spi_mode_combo.currentText()),
            initex_mode=initex_mode,
            num_frames=self.spin_frames.value(),
            frame_size=self.spin_frame_size.value(),
            host_intr_port=self.intr_port_combo.currentIndex(),
            timeout_s=float(self.spin_timeout.value()),
            intr_stable_reads=self.spin_stable_reads.value(),
            intr_stable_sleep_us=self.spin_stable_sleep.value(),
            wait_intr_high=self.cb_wait_high.isChecked(),
            full_reset=self.cb_full_reset.isChecked(),
            flush_before_start=self.cb_flush_before.isChecked(),
            flush_each_frame=self.cb_flush_each.isChecked(),
            sync_frame_boundary=self.cb_sync_boundary.isChecked(),
            save_to_file=self.cb_save.isChecked(),
            out_dir=out_dir,
            overwrite_on_start=self.cb_overwrite.isChecked(),
            preview_every=self.spin_preview.value(),
            log_every=self.spin_log.value(),
            byteswap32=self.cb_byteswap.isChecked(),
            spi_read_timeout_s=float(self.spin_read_timeout.value()),
            fast_mode=self.cb_fast_mode.isChecked(),
        )

        if settings.save_to_file and not settings.out_dir:
            QMessageBox.warning(self, "Warning", "Output folder is empty. Please select a folder.")
            return

        self._log("===== Capture Start (SPI SLAVE MODE) =====")
        self._log(f"SPI dev         : desc='{settings.spi_desc}', index={settings.spi_dev_index}")
        self._log(f"GPIO dev        : desc='{settings.gpio_desc}', index={settings.gpio_dev_index}")
        self._log(f"SPI Mode        : {settings.spi_mode}")
        self._log(f"InitEx Mode     : {settings.initex_mode if settings.initex_mode is not None else 'Auto'}")
        self._log(f"Frame Size      : {settings.frame_size:,} bytes")
        self._log(f"HOST_INTR       : P{settings.host_intr_port}, timeout={settings.timeout_s}s")
        self._log(f"Wait INTR HIGH  : {'ON' if settings.wait_intr_high else 'OFF'}")
        self._log(f"Full Reset      : {'ON' if settings.full_reset else 'OFF'}")
        self._log(f"Frame Sync      : {'ON (skip 1 frame)' if settings.sync_frame_boundary else 'OFF'}")
        self._log(f"Byteswap32      : {'ON' if settings.byteswap32 else 'OFF'}")
        self._log(f"Fast Mode       : {'ON (buffer in memory)' if settings.fast_mode else 'OFF (write immediately)'}")
        self._log(f"Save            : {'ON' if settings.save_to_file else 'OFF'}")
        self._log(f"Output folder   : {settings.out_dir if settings.save_to_file else '(not saving)'}")

        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.progress.setValue(0)
        self.progress.setMaximum(settings.num_frames if settings.num_frames > 0 else 0)

        self.capture_thread = QThread()
        self.capture_worker = SpiCaptureWorker(settings)
        self.capture_worker.moveToThread(self.capture_thread)

        self.capture_thread.started.connect(self.capture_worker.start)
        self.capture_worker.status.connect(self._on_status)
        self.capture_worker.error.connect(self._on_error)
        self.capture_worker.progress.connect(self._on_progress)
        self.capture_worker.preview.connect(self._on_preview)
        self.capture_worker.finished.connect(self._on_finished)

        self.capture_thread.start()

    def _stop_capture(self):
        """캡처 중지 버튼 핸들러: 워커에 stop 요청을 전달합니다."""
        if self.capture_worker:
            self.capture_worker.stop()
        self._log("Stop requested...")

    def _on_status(self, msg: str):
        """상태 시그널 핸들러: 상태바와 로그에 출력합니다."""
        self.status_label.setText(msg[:100])
        self._log(msg)

    def _on_error(self, msg: str):
        """오류 시그널 핸들러: 로그에 기록하고 다이얼로그를 띄웁니다."""
        self._log(f"ERROR: {msg}")
        QMessageBox.critical(self, "Error", msg)

    def _on_progress(self, current: int, total: int):
        """진행률 시그널 핸들러: ProgressBar를 업데이트합니다."""
        if total > 0:
            self.progress.setMaximum(total)
            self.progress.setValue(current)
        else:
            self.progress.setValue(current % 100)

    def _on_preview(self, data: bytes):
        """프리뷰 시그널 핸들러: 프레임 앞부분을 hex로 로그에 출력합니다."""
        hex_str = " ".join(f"{b:02X}" for b in data)
        self._log(f"[PREVIEW] {hex_str}")

    def _on_finished(self):
        """완료 시그널 핸들러: 스레드를 정리하고 UI를 초기화합니다."""
        if self.capture_thread:
            self.capture_thread.quit()
            self.capture_thread.wait(2000)

        self.capture_thread = None
        self.capture_worker = None
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self._log("Capture finished")


def main():
    """애플리케이션 엔트리 포인트."""
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()