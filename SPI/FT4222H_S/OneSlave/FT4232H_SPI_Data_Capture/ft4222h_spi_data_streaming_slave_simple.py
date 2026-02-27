#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ft4222h_spi_data_streaming_slave_simple.py
========================================

FT4222H SPI SLAVE 캡처 도구 (CLI 버전, 프레임 기반 읽기)

이 스크립트는 FT4222H를 **SPI Slave**로 사용하여, AWRL6844(마스터)가 SPI로 전송하는
ADC 데이터를 **프레임(고정 길이)** 단위로 수신하고 파일로 저장합니다.

개요
----

- 마스터(AWRL6844)가 프레임당 고정 크기 데이터를 SPI로 전송합니다.
- 슬레이브(FT4222H/PC)는 HOST_INTR(GPIO) 신호를 이용해 프레임 경계를 동기화합니다.
- RX FIFO/USB 파이프라인 잔여 데이터를 flush하여 프레임 경계 틀어짐을 줄입니다.

마스터 사양 (AWRL6844)
----------------------

- SPI Master, 20MHz, Mode0
- 프레임당 32 KiB 전송 (예: 16 chirps × 256 samples × 4 Rx × 2 bytes)
- 프레임마다 HOST_INTR LOW/HIGH 토글

슬레이브 사양 (FT4222H, 본 스크립트)
------------------------------------

- HOST_INTR LOW 대기 → 정확히 프레임 크기만큼 읽기 → (옵션) HIGH 대기

프레임 패턴(검증용)
-------------------

- 패턴: ``XX XX XX XX 00 01 02 03 04 05 06 07 ...``
- 바이트 0-3: 프레임 카운터(4바이트, big-endian u32 등 가변)
- 바이트 4-: 순차 패턴(테스트/검증용) ``((i-4) & 0xFF)``

:Author:
:Date: 2026
"""

import os
import time
import struct
import argparse
import glob
import array
import sys
from typing import Optional, List, Tuple, Any

import ft4222
from ft4222.SPI import Cpha, Cpol
from ft4222.GPIO import Dir, Port

# ----------------------------------------------------------------------
# Constants & compatibility
# ----------------------------------------------------------------------
API_SAFE_MAX_READ = 65535
DEFAULT_CHUNK_SIZE = 64 * 1024          # 65536, MAXSPISIZEFTDI
DEFAULT_FRAME_SIZE = 32 * 1024          # 32768 (16 chirps * 256 samples * 4 Rx * 2 bytes)


def list_ft4222_devices() -> List[Tuple[int, Any]]:
    """
    시스템에 연결된 FT4222 장치 목록을 반환합니다.

    FTDI FT4222 Python 라이브러리를 사용하여 시스템에 연결된 모든 FT4222 장치를
    검색하고 장치 정보를 수집합니다.

    :returns: ``(index, device_info)`` 튜플 리스트
    :rtype: list[tuple[int, Any]]
    """
    out: List[Tuple[int, Any]] = []
    try:
        n = ft4222.createDeviceInfoList()
    except Exception:
        return out

    for i in range(n):
        try:
            info = ft4222.getDeviceInfoDetail(i, update=False)
        except TypeError:
            info = ft4222.getDeviceInfoDetail(i)
        except Exception as e:
            info = f"<error: {e}>"
        out.append((i, info))
    return out


def open_by_index_compat(index: int):
    """
    인덱스로 FT4222 장치를 여는 호환성 폴백 함수입니다.

    일부 환경에서는 ``ft4222.open(index)``가 지원되지 않을 수 있습니다.
    이 경우 장치의 location 정보를 조회한 뒤 ``openByLocation(location)``으로
    여는 방식으로 폴백합니다.

    :param int index: 열고자 하는 장치 인덱스
    :returns: 열린 FT4222 장치 핸들
    :raises RuntimeError: 장치 열기 실패(예: location 조회 실패, openByLocation 실패 등)
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

    우선 ``openByDescription(desc)``를 시도하고,
    실패하면 ``open_by_index_compat(index)``로 폴백합니다.

    :param str desc: 장치 description (예: ``"FT4222 A"``)
    :param index: 폴백용 장치 인덱스. ``None``이면 폴백하지 않고 예외를 그대로 올립니다.
    :type index: int | None
    :returns: 열린 FT4222 장치 핸들
    :raises Exception: description/index 모두 실패 시 (하위 함수 예외 포함)
    """
    try:
        return ft4222.openByDescription(desc)
    except TypeError:
        return ft4222.openByDescription(desc.encode("utf-8"))
    except Exception:
        if index is None:
            raise
        return open_by_index_compat(index)


def port_enum(p: int) -> Port:
    """
    정수 GPIO 포트 번호를 FT4222 ``Port`` 열거형으로 변환합니다.

    :param int p: GPIO 포트 번호 (0~3)
    :returns: ``Port.P0`` ~ ``Port.P3``
    :rtype: ft4222.GPIO.Port
    :raises ValueError: ``p``가 0~3 범위를 벗어난 경우
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


def spi_mode_to_cpol_cpha(mode: int):
    """
    SPI 모드(0~3)를 CPOL/CPHA 열거형 조합으로 변환합니다.

    - Mode 0: CPOL=0, CPHA=0 (Idle LOW, Leading edge)
    - Mode 1: CPOL=0, CPHA=1 (Idle LOW, Trailing edge)
    - Mode 2: CPOL=1, CPHA=0 (Idle HIGH, Leading edge)
    - Mode 3: CPOL=1, CPHA=1 (Idle HIGH, Trailing edge)

    :param int mode: SPI mode (0~3)
    :returns: ``(Cpol, Cpha)`` 튜플
    :rtype: tuple[ft4222.SPI.Cpol, ft4222.SPI.Cpha]
    :raises ValueError: ``mode``가 0~3 범위를 벗어난 경우
    """
    if mode == 0:
        return Cpol.IDLE_LOW, Cpha.CLK_LEADING
    if mode == 1:
        return Cpol.IDLE_LOW, Cpha.CLK_TRAILING
    if mode == 2:
        return Cpol.IDLE_HIGH, Cpha.CLK_LEADING
    if mode == 3:
        return Cpol.IDLE_HIGH, Cpha.CLK_TRAILING
    raise ValueError("SPI mode must be 0..3")


def byteswap32_if_little_endian(data: bytes) -> bytes:
    """
    호스트가 little-endian인 경우에만 32비트 워드 단위로 바이트 스왑합니다.

    SPI로 수신한 데이터가 워드 단위 big-endian 정렬인 경우,
    little-endian PC에서 해석하기 쉽게 만들기 위해 사용합니다.

    .. note::
       입력 데이터 길이가 4의 배수가 아니면 0x00으로 패딩하여 4바이트 정렬 후 처리합니다.

    :param bytes data: 변환할 입력 데이터
    :returns: 스왑된 데이터(호스트가 big-endian이면 원본 반환)
    :rtype: bytes
    """
    if sys.byteorder == 'little':
        if len(data) % 4 != 0:
            data = data + b"\x00" * (4 - (len(data) % 4))

        a = array.array('I')
        if a.itemsize != 4:
            b = bytearray(data)
            for i in range(0, len(b), 4):
                b[i:i+4] = b[i:i+4][::-1]
            return bytes(b)

        a.frombytes(data)
        a.byteswap()
        return a.tobytes()

    return data


def safe_clean_previous(out_dir: str, pattern: str = "adc_data_*.bin") -> int:
    """
    출력 디렉토리에서 이전 캡처 파일을 삭제합니다.

    ``out_dir``이 없으면 생성한 뒤, ``pattern``과 매칭되는 파일을 삭제합니다.

    :param str out_dir: 출력 디렉토리 경로
    :param str pattern: 삭제할 파일 패턴 (기본: ``"adc_data_*.bin"``)
    :returns: 삭제된 파일 개수
    :rtype: int
    """
    os.makedirs(out_dir, exist_ok=True)
    removed = 0
    for p in glob.glob(os.path.join(out_dir, pattern)):
        try:
            os.remove(p)
            removed += 1
        except Exception:
            pass
    return removed


# ----------------------------------------------------------------------
# GPIO helpers with stability options
# ----------------------------------------------------------------------
def gpio_init_input_all(dev_gpio) -> None:
    """
    FT4222 GPIO P0~P3를 모두 입력 모드로 초기화합니다.

    FT4222 Python API 버전에 따라 ``gpio_Init`` 시그니처가 다를 수 있어
    keyword/positional 호출을 모두 지원합니다.

    :param dev_gpio: FT4222 GPIO 디바이스 핸들
    """
    try:
        dev_gpio.gpio_Init(gpio0=Dir.INPUT, gpio1=Dir.INPUT,
                           gpio2=Dir.INPUT, gpio3=Dir.INPUT)
    except TypeError:
        dev_gpio.gpio_Init(Dir.INPUT, Dir.INPUT, Dir.INPUT, Dir.INPUT)


def gpio_wait_level(dev_gpio, port, level_low: bool, timeout_s: float,
                    stable_reads: int = 0, stable_sleep_us: int = 20) -> None:
    """
    GPIO 핀이 특정 레벨(LOW/HIGH)이 될 때까지 대기합니다.

    - 하드웨어 대기 기능(``gpio_Wait``)이 있으면 이를 우선 사용합니다.
    - 없으면 소프트웨어 폴링(``gpio_Read`` 반복)으로 대기합니다.
    - ``stable_reads``를 지정하면 목표 레벨이 연속으로 유지되는지 추가 검증합니다.

    :param dev_gpio: FT4222 GPIO 디바이스 핸들
    :param port: 대기할 GPIO 포트(enum 또는 해당 타입)
    :param bool level_low: ``True``면 LOW, ``False``면 HIGH를 목표로 대기
    :param float timeout_s: 타임아웃(초)
    :param int stable_reads: 연속 동일 레벨 확인 횟수(0이면 미사용)
    :param int stable_sleep_us: 안정화 확인 사이 sleep(마이크로초)
    :raises TimeoutError: 타임아웃 내에 목표 레벨에 도달하지 못한 경우
    """
    timeout_ms = int(timeout_s * 1000)
    target_level = level_low  # True = LOW, False = HIGH

    def is_level() -> bool:
        v = dev_gpio.gpio_Read(port)
        is_low = (v is False) or (v == 0)
        return is_low == target_level

    # HW wait
    if hasattr(dev_gpio, "gpio_Wait"):
        deadline = time.perf_counter() + timeout_s
        while True:
            # 1) gpio_Wait로 먼저 대기
            try:
                if hasattr(dev_gpio.gpio_Wait, '__call__'):
                    dev_gpio.gpio_Wait(port, target_level, timeout=timeout_ms, sleep=0)
                else:
                    dev_gpio.gpio_Wait(port, target_level, timeout_ms)
            except Exception:
                pass

            # 2) 안정화 검증
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
                    raise TimeoutError(
                        f"HOST_INTR {'LOW' if target_level else 'HIGH'} timeout (port={port})"
                    )
                timeout_ms = int(remaining * 1000)
                continue

            return

    # SW polling
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

        time.sleep(0.0001)  # 100us polling


def gpio_wait_low(dev_gpio, port: Port, timeout_s: float,
                  stable_reads: int = 0, stable_sleep_us: int = 10) -> None:
    """
    GPIO가 LOW가 될 때까지 대기합니다 (프레임 시작 신호).

    :param dev_gpio: FT4222 GPIO 디바이스 핸들
    :param ft4222.GPIO.Port port: 대기할 포트
    :param float timeout_s: 타임아웃(초)
    :param int stable_reads: 안정화 확인 횟수
    :param int stable_sleep_us: 안정화 확인 사이 sleep(마이크로초)
    :raises TimeoutError: 타임아웃 발생 시
    """
    gpio_wait_level(dev_gpio, port, True, timeout_s, stable_reads, stable_sleep_us)


def gpio_wait_high(dev_gpio, port: Port, timeout_s: float,
                   stable_reads: int = 0, stable_sleep_us: int = 10) -> None:
    """
    GPIO가 HIGH가 될 때까지 대기합니다 (프레임 종료/idle 신호).

    :param dev_gpio: FT4222 GPIO 디바이스 핸들
    :param ft4222.GPIO.Port port: 대기할 포트
    :param float timeout_s: 타임아웃(초)
    :param int stable_reads: 안정화 확인 횟수
    :param int stable_sleep_us: 안정화 확인 사이 sleep(마이크로초)
    :raises TimeoutError: 타임아웃 발생 시
    """
    gpio_wait_level(dev_gpio, port, False, timeout_s, stable_reads, stable_sleep_us)


# ----------------------------------------------------------------------
# SPI Slave low-level access (works with multiple API flavours)
# ----------------------------------------------------------------------
def _find_spi_obj(dev_spi):
    """
    FT4222 디바이스 핸들에서 SPI Slave 기능을 제공하는 객체를 찾습니다.

    FT4222 Python 래퍼/버전/바인딩에 따라 SPI 메서드가 붙어있는 객체가 다를 수 있습니다.
    이 함수는 대표적인 후보 속성명을 검색하여 SPI Slave 관련 메서드가 있는 객체를 반환합니다.

    :param dev_spi: FT4222 SPI 디바이스 핸들
    :returns: SPI Slave 메서드를 가진 객체(없으면 ``dev_spi`` 자체)
    """
    for name in ("spiSlave", "spislave", "SPI", "spi"):
        if hasattr(dev_spi, name):
            obj = getattr(dev_spi, name)
            if any(hasattr(obj, m) for m in (
                "spiSlave_InitEx", "spiSlave_Init",
                "spiSlave_Read", "spiSlave_GetRxStatus"
            )):
                return obj
    return dev_spi


def spi_slave_get_rx_available(spi) -> Optional[int]:
    """
    SPI Slave RX 버퍼에서 읽을 수 있는 바이트 수를 조회합니다.

    API 이름이 환경에 따라 다를 수 있어 여러 후보 메서드를 순차 시도합니다.

    :param spi: SPI Slave 객체
    :returns: 읽기 가능한 바이트 수 (조회 실패 시 ``None``)
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
    SPI Slave RX 버퍼에서 데이터를 한 번 읽습니다(호환성 포함).

    환경마다 read 함수 이름/시그니처/리턴 타입이 다를 수 있어
    여러 후보 메서드를 순차적으로 시도합니다.

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
                # 구형 시그니처 대응: fn(nbytes, 0)
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

    USB/드라이버 특성상 한 번의 read 호출로 항상 ``nbytes``가 반환되지 않을 수 있으므로,
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
    SPI Slave RX 버퍼를 읽어서 폐기합니다(플러시).

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

    USB 파이프라인/드라이버 버퍼링으로 인해 flush 직후에도 잔여 데이터가
    조금 늦게 도착하는 경우가 있어, 여러 번 반복하고 중간에 잠깐 대기합니다.

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

        # 연속 2회 이상 데이터 없으면 종료
        if flushed == 0 and i > 0:
            break

        if delay_ms > 0:
            time.sleep(delay_ms / 1000.0)

    return total_flushed


def spi_slave_wait_idle(spi, stable_checks: int = 3, check_interval_ms: float = 20) -> bool:
    """
    RX FIFO가 비어있는 상태가 일정 횟수 연속으로 유지되는지 확인합니다.

    :param spi: SPI Slave 객체
    :param int stable_checks: 연속으로 비어있어야 하는 횟수
    :param float check_interval_ms: 확인 간격(ms)
    :returns: FIFO가 안정적으로 비어있다고 판단되면 True
    :rtype: bool
    """
    zero_count = 0
    for _ in range(stable_checks * 2):
        avail = spi_slave_get_rx_available(spi)
        if avail is None or avail <= 0:
            zero_count += 1
            if zero_count >= stable_checks:
                return True
        else:
            zero_count = 0
            spi_slave_flush(spi, 100_000)

        time.sleep(check_interval_ms / 1000.0)

    return zero_count >= stable_checks


def find_frame_sync_offset(data: bytes, debug: bool = False) -> int:
    """
    프레임 헤더 패턴을 기반으로 동기 오프셋을 찾습니다(관대한 패턴).

    관대한 패턴(처음 8바이트 중 7바이트만 체크):

    - ``00 00 01 XX 00 01 02 03`` 형태를 first 2048 bytes 내에서 탐색
    - 여기서 ``XX``는 가변값(예: 카운터 일부)로 간주

    :param bytes data: 검사할 데이터(최소 12바이트 필요)
    :param bool debug: True이면 디버그 로그 출력
    :returns: 패턴 시작 오프셋 (0이면 정렬됨, >0이면 그만큼 어긋남, -1이면 미발견)
    :rtype: int
    """
    if len(data) < 12:
        return -1

    for i in range(min(len(data) - 12, 2048)):
        if (data[i] == 0x00 and data[i+1] == 0x00 and data[i+2] == 0x01 and
            data[i+4] == 0x00 and data[i+5] == 0x01 and data[i+6] == 0x02 and data[i+7] == 0x03):
            if debug:
                print(f"[SYNC DEBUG] Found pattern at offset {i}")
            return i

    if debug:
        print(f"[SYNC DEBUG] Pattern not found in first {min(len(data), 2048)} bytes")
        print(f"[SYNC DEBUG] First 32 bytes: {' '.join(f'{b:02X}' for b in data[:32])}")

    return -1


def check_frame_header_simple(data: bytes) -> bool:
    """
    프레임 헤더 패턴을 간단히 확인합니다.

    확인 패턴(8바이트 중 7바이트):

    - ``00 00 01 XX 00 01 02 03``

    :param bytes data: 확인할 프레임 데이터(최소 8바이트 필요)
    :returns: 패턴 일치 여부
    :rtype: bool
    """
    if len(data) < 8:
        return False
    return (data[0] == 0x00 and data[1] == 0x00 and data[2] == 0x01 and
            data[4] == 0x00 and data[5] == 0x01 and data[6] == 0x02 and data[7] == 0x03)


def spi_slave_init_stable(dev_spi, cpol, cpha, initex_override: Optional[int] = None):
    """
    SPI Slave 모드를 초기화합니다(여러 API 버전 호환).

    초기화 순서:
    1) 가능하면 SPI reset 계열 호출
    2) ``spiSlave_InitEx`` 또는 ``spiSlave_Init``로 초기화
    3) 가능하면 CPOL/CPHA 모드 설정

    :param dev_spi: FT4222 SPI 디바이스 핸들
    :param cpol: CPOL 설정(enum)
    :param cpha: CPHA 설정(enum)
    :param initex_override: ``spiSlave_InitEx(value)`` 강제 값. None이면 (1→2→0) 순으로 시도
    :type initex_override: int | None
    :returns: 초기화된 SPI Slave 객체
    :raises RuntimeError: 초기화 API를 찾지 못했거나 모든 초기화 시도가 실패한 경우
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


# ----------------------------------------------------------------------
# Frame verification
# ----------------------------------------------------------------------
def verify_frame(frame: bytes, expected_cnt: Optional[int], sample_checks: int = 256) -> dict:
    """
    프레임 데이터의 간이 무결성 검증을 수행합니다.

    검증 항목:
    - 프레임 카운터(바이트 0-3, big-endian u32)
    - 순차 패턴(바이트 4-15 == 00..0B)
    - 프레임 일부 샘플링 검사(간격 기반)

    :param bytes frame: 검증할 프레임 데이터
    :param expected_cnt: 예상 프레임 카운터(None이면 카운터 비교 생략)
    :type expected_cnt: int | None
    :param int sample_checks: 샘플링 검사 횟수
    :returns: 검증 결과 dict
    :rtype: dict
    """
    res = {
        "seq_ok": False,
        "cnt": None,
        "cnt_ok": None,
        "sample_mismatch": 0,
    }

    if len(frame) < 16:
        return res

    cnt = struct.unpack(">I", frame[0:4])[0]
    res["cnt"] = cnt

    if expected_cnt is not None:
        res["cnt_ok"] = (cnt == expected_cnt)

    res["seq_ok"] = (frame[4:16] == bytes(range(0x00, 0x0C)))

    n = len(frame)
    if n > 20 and sample_checks > 0:
        step = max((n - 4) // sample_checks, 1)
        mism = 0
        pos = 4
        for _ in range(sample_checks):
            if pos >= n:
                break
            exp = (pos - 4) & 0xFF
            if frame[pos] != exp:
                mism += 1
            pos += step
        res["sample_mismatch"] = mism

    return res


# ----------------------------------------------------------------------
# Main capture routine (single-block or split, based on args)
# ----------------------------------------------------------------------
def capture(args) -> None:
    """
    메인 캡처 루틴: SPI Slave로 프레임 데이터를 수신/저장합니다.

    처리 흐름(요약):

    1) 장치 오픈(SPI/GPIO)
    2) (옵션) full reset(close/reopen)으로 상태 초기화
    3) SPI Slave 초기화 및 모드 설정
    4) 시작 전 HOST_INTR idle(HIGH) 확인(옵션)
    5) RX FIFO/USB 잔여 데이터 flush(옵션)
    6) 프레임 경계 동기화(옵션)
    7) 반복 수신:
       - HOST_INTR LOW 대기
       - 프레임 크기만큼 정확히 읽기
       - (옵션) HOST_INTR HIGH 대기
       - (옵션) byteswap32 적용
       - 저장(또는 fast-mode 버퍼링)
    8) 통계/요약 출력 후 정리

    :param args: argparse로 파싱된 인수 객체
    """
    if args.list_devices:
        devs = list_ft4222_devices()
        if not devs:
            print("No FT4222 devices found.")
        else:
            print("Detected FT4222 devices:")
            for i, info in devs:
                print(f"  [{i}] {info}")
        return

    out_dir = args.out_dir
    os.makedirs(out_dir, exist_ok=True)

    if args.clean:
        removed = safe_clean_previous(out_dir, "adc_data_*.bin")
        print(f"[CLEAN] Removed {removed} file(s) in: {out_dir}")

    # 1) 장치 열기
    dev_spi = open_by_desc_or_index(args.spi_desc, args.spi_index)
    dev_gpio = open_by_desc_or_index(args.gpio_desc, args.gpio_index)

    spi = None
    try:
        # 2) full reset (close/reopen)
        if args.full_reset:
            print("[RESET] Full device close/reopen...")
            try:
                dev_spi.close()
            except Exception:
                pass
            try:
                dev_gpio.close()
            except Exception:
                pass

            time.sleep(1.0)  # USB 리셋/버퍼 정리 대기

            dev_spi = open_by_desc_or_index(args.spi_desc, args.spi_index)
            dev_gpio = open_by_desc_or_index(args.gpio_desc, args.gpio_index)
            time.sleep(0.2)
            print("[RESET] Devices reopened.")

        # 3) 클럭 + GPIO 초기화
        if hasattr(dev_spi, "setClock") and hasattr(ft4222, "SysClock"):
            try:
                dev_spi.setClock(ft4222.SysClock.CLK_80)
                print("[CLK] setClock(SysClock.CLK_80) OK")
            except Exception as e:
                print(f"[CLK] setClock failed (ignored): {e}")

        gpio_init_input_all(dev_gpio)
        intr_port = port_enum(args.host_intr_port)

        # 4) SPI slave init
        cpol, cpha = spi_mode_to_cpol_cpha(args.spi_mode)
        spi = spi_slave_init_stable(dev_spi, cpol, cpha, initex_override=args.initex)

        # 5) 시작 전 HOST_INTR idle(HIGH) 확인
        if args.sync_idle_high:
            try:
                gpio_wait_high(dev_gpio, intr_port, args.intr_timeout_s)
                print("[SYNC] HOST_INTR is HIGH (idle) – OK")
            except TimeoutError:
                sys.exit(
                    "[ERROR] HOST_INTR is not HIGH at start. "
                    "Please ensure master is idle (CS=HIGH) before running.\n"
                    "Reset the master device or unplug/replug FT4222."
                )
        else:
            print("[WARNING] --no-sync-idle-high: startup synchronization disabled. "
                  "Data corruption is likely if master CS is already LOW.")

        # 6) 안정화 + flush
        time.sleep(0.01)

        if args.flush_before_start:
            flushed = spi_slave_flush_aggressive(spi, attempts=5, delay_ms=30)
            if flushed > 0:
                print(f"[SPI] Aggressive flush: {flushed} bytes discarded")

            if spi_slave_wait_idle(spi, stable_checks=3, check_interval_ms=20):
                print("[SPI] RX FIFO confirmed idle")
            else:
                print("[WARNING] RX FIFO may still have residual data")
                extra = spi_slave_flush_aggressive(spi, attempts=3, delay_ms=50)
                if extra > 0:
                    print(f"[SPI] Extra flush: {extra} bytes discarded")

        # 6-1) 프레임 경계 동기화
        if args.sync_frame_boundary:
            print("[SYNC] Synchronizing to frame boundary...")

            current_level = dev_gpio.gpio_Read(intr_port)
            is_low = (current_level is False) or (current_level == 0)
            print(f"[SYNC] Current HOST_INTR: {'LOW (frame active)' if is_low else 'HIGH (idle)'}")

            if is_low:
                print("[SYNC] Waiting for current frame to end (HIGH)...")
                try:
                    gpio_wait_high(dev_gpio, intr_port, args.intr_timeout_s)
                    print("[SYNC] Frame ended (HIGH)")
                except TimeoutError:
                    print("[WARNING] Timeout waiting for HIGH")

            # HIGH 상태에서 USB 파이프라인 잔여 데이터 도착 고려
            time.sleep(0.03)
            flush_total = 0
            for _ in range(10):
                flushed = spi_slave_flush(spi, 200_000)
                flush_total += flushed
                if flushed == 0:
                    break
                time.sleep(0.02)

            if flush_total > 0:
                print(f"[SYNC] Flushed {flush_total} bytes (previous frame data)")

            # 한 프레임 통째로 버리기(크기 기반 동기화)
            frame_size = args.frame_size
            print(f"[SYNC] Skipping one full frame ({frame_size} bytes)...")

            try:
                gpio_wait_low(dev_gpio, intr_port, args.intr_timeout_s)

                skip_data = spi_slave_read_exact(spi, frame_size, args.spi_read_timeout_s)
                print(f"[SYNC] Read and discarded {len(skip_data)} bytes")

                if args.byteswap32:
                    skip_swapped = byteswap32_if_little_endian(skip_data[:32])
                else:
                    skip_swapped = skip_data[:32]
                print(f"[SYNC] Skipped frame start: {' '.join(f'{b:02X}' for b in skip_swapped[:16])}")

                gpio_wait_high(dev_gpio, intr_port, args.intr_timeout_s)
                print("[SYNC] Frame boundary confirmed (HIGH)")

            except TimeoutError as e:
                print(f"[WARNING] Frame skip timeout: {e}")
                spi_slave_flush(spi, 500_000)

            time.sleep(0.02)
            final_flushed = spi_slave_flush(spi, 100_000)
            if final_flushed > 0:
                print(f"[SYNC] Final cleanup: {final_flushed} bytes")

            print("[SYNC] Ready - aligned at frame boundary")

        # 7) 설정 출력
        print("\n" + "=" * 60)
        print("    FT4222H SPI SLAVE Capture (64KB Frame Mode)")
        print("=" * 60)
        print("Mode           : SPI SLAVE")
        print("                 AWRL6844 = Master (drives SCLK/CS, sends frames)")
        print("                 FT4222H  = Slave  (receives one block per frame)")
        print("-" * 60)
        print(f"SPI desc       : {args.spi_desc} (fallback index={args.spi_index})")
        print(f"GPIO desc      : {args.gpio_desc} (fallback index={args.gpio_index})")
        print(f"SPI mode       : {args.spi_mode} (CPOL/CPHA)")
        print(f"HOST_INTR      : P{args.host_intr_port} (LOW = data ready)")
        print(f"Wait INTR HIGH : {'YES' if args.wait_intr_high else 'NO'} (per frame)")
        print(f"Frame size     : {args.frame_size:,} bytes")
        print(f"Chunk size     : {args.chunk_size:,} bytes "
              f"{'(≥ frame size → no splitting)' if args.chunk_size >= args.frame_size else '(split into chunks)'}")
        print(f"Byteswap32     : {'ON' if args.byteswap32 else 'OFF'} "
              f"(host={sys.byteorder}-endian)")
        print(f"Fast mode      : {'ON (buffer in memory)' if args.fast_mode else 'OFF'}")
        print(f"Full reset     : {'ON' if args.full_reset else 'OFF'}")
        print(f"Frame boundary : {'ON (skip 1 frame for sync)' if args.sync_frame_boundary else 'OFF'}")
        print(f"Auto sync      : {'ON (frame header detection)' if args.auto_sync else 'OFF'}")
        print(f"OUT dir        : {out_dir}")
        print("=" * 60)
        print("\nWaiting for AWRL6844 (Master) to send data...\n")

        # 8) 메인 캡처 루프
        total_bytes = 0
        start_time = time.perf_counter()
        frame_timeout_cnt = 0
        chunk_timeout_cnt = 0
        expected_cnt = None

        frame_buffer = [] if args.fast_mode else None

        last_stat_time = start_time
        last_stat_bytes = 0
        last_stat_frames = 0

        frame_idx = 0
        max_frames = args.frames if args.frames > 0 else 10**12
        sync_established = False
        sync_retries = 0
        first_frame_debug_done = False

        while frame_idx < max_frames:
            if args.flush_each_frame:
                try:
                    gpio_wait_high(dev_gpio, intr_port, args.intr_timeout_s)
                except TimeoutError:
                    pass
                spi_slave_flush(spi)

            remaining = args.frame_size
            frame_chunks: List[bytes] = []
            frame_ok = True

            while remaining > 0:
                # 프레임 시작(LOW) 대기
                try:
                    gpio_wait_low(dev_gpio, intr_port, args.intr_timeout_s,
                                  stable_reads=args.intr_stable_reads,
                                  stable_sleep_us=args.intr_stable_sleep_us)
                except TimeoutError:
                    print(f"[FRAME {frame_idx+1}] Timeout waiting INTR LOW")
                    frame_timeout_cnt += 1
                    frame_ok = False
                    break

                to_read = min(remaining, args.chunk_size)
                try:
                    chunk = spi_slave_read_exact(spi, to_read, args.spi_read_timeout_s)
                except TimeoutError as e:
                    print(f"[FRAME {frame_idx+1}] Read timeout: {e}")
                    chunk_timeout_cnt += 1
                    frame_ok = False
                    break

                frame_chunks.append(chunk)
                remaining -= len(chunk)

                # 프레임/청크 종료 후 HIGH 대기
                if args.wait_intr_high:
                    try:
                        gpio_wait_high(dev_gpio, intr_port, args.intr_timeout_s,
                                       stable_reads=args.intr_stable_reads,
                                       stable_sleep_us=args.intr_stable_sleep_us)
                    except TimeoutError:
                        print(f"[FRAME {frame_idx+1}] Warning: INTR HIGH timeout")

                if args.post_chunk_delay_us > 0:
                    time.sleep(args.post_chunk_delay_us / 1_000_000.0)

            if not frame_ok:
                if args.stop_on_error:
                    print("[STOP] stop_on_error triggered")
                    break
                continue

            raw_frame = b"".join(frame_chunks)

            # 첫 프레임 디버그 출력(autosync OFF일 때)
            if (not first_frame_debug_done) and args.debug_first_frame and (not args.auto_sync):
                first_frame_debug_done = True
                test_frame = byteswap32_if_little_endian(raw_frame) if args.byteswap32 else raw_frame
                print(f"[DEBUG] First frame RAW  (32B): {' '.join(f'{b:02X}' for b in raw_frame[:32])}")
                print(f"[DEBUG] First frame SWAP (32B): {' '.join(f'{b:02X}' for b in test_frame[:32])}")
                if check_frame_header_simple(test_frame):
                    print("[DEBUG] Frame header pattern: OK")
                else:
                    print("[DEBUG] Frame header pattern: MISMATCH (data may be misaligned)")

            # 첫 프레임 동기화(auto_sync)
            if (not sync_established) and args.auto_sync:
                test_frame = byteswap32_if_little_endian(raw_frame) if args.byteswap32 else raw_frame

                print(f"[SYNC DEBUG] RAW  first 32: {' '.join(f'{b:02X}' for b in raw_frame[:32])}")
                print(f"[SYNC DEBUG] SWAP first 32: {' '.join(f'{b:02X}' for b in test_frame[:32])}")

                if check_frame_header_simple(test_frame):
                    print("[SYNC] Frame header OK (simple check)")
                    sync_established = True
                else:
                    sync_offset = find_frame_sync_offset(test_frame, debug=True)

                    if sync_offset == 0:
                        print("[SYNC] Frame header aligned correctly")
                        sync_established = True
                    elif sync_offset > 0:
                        sync_retries += 1
                        print(f"[SYNC] Frame misaligned by {sync_offset} bytes, discarding... "
                              f"(retry {sync_retries}/{args.max_sync_retries})")

                        if sync_retries >= args.max_sync_retries:
                            print("[ERROR] Max sync retries reached. Continuing without sync verification.")
                            print("        Data may be misaligned. Use --no-auto-sync to skip sync check.")
                            sync_established = True
                        else:
                            spi_slave_flush_aggressive(spi, attempts=2, delay_ms=20)
                            try:
                                gpio_wait_high(dev_gpio, intr_port, args.intr_timeout_s)
                            except TimeoutError:
                                pass
                            spi_slave_flush(spi, 200_000)
                            continue
                    else:
                        sync_retries += 1
                        print(f"[SYNC WARNING] Frame header pattern not found "
                              f"(retry {sync_retries}/{args.max_sync_retries})")

                        if sync_retries >= args.max_sync_retries:
                            print("[WARNING] Sync pattern not found. Accepting frames as-is.")
                            print("          Check if master data format matches expected pattern.")
                            sync_established = True
                        else:
                            spi_slave_flush_aggressive(spi, attempts=2, delay_ms=20)
                            try:
                                gpio_wait_high(dev_gpio, intr_port, args.intr_timeout_s)
                            except TimeoutError:
                                pass
                            spi_slave_flush(spi, 200_000)
                            continue

            frame_idx += 1
            total_bytes += len(raw_frame)

            frame = byteswap32_if_little_endian(raw_frame) if args.byteswap32 else raw_frame

            if args.fast_mode:
                assert frame_buffer is not None
                frame_buffer.append(frame)
            else:
                out_path = os.path.join(out_dir, f"adc_data_{frame_idx:06d}.bin")
                with open(out_path, "wb") as f:
                    f.write(frame)

            # 통계 출력
            if args.log_every > 0 and (frame_idx % args.log_every) == 0:
                now = time.perf_counter()
                elapsed_interval = now - last_stat_time
                interval_bytes = total_bytes - last_stat_bytes
                interval_frames = frame_idx - last_stat_frames

                interval_mbps = (interval_bytes / (1024 * 1024)) / elapsed_interval if elapsed_interval > 0 else 0.0
                interval_fps = interval_frames / elapsed_interval if elapsed_interval > 0 else 0.0

                total_elapsed = now - start_time
                total_mbps = (total_bytes / (1024 * 1024)) / total_elapsed if total_elapsed > 0 else 0.0
                total_fps = frame_idx / total_elapsed if total_elapsed > 0 else 0.0

                print(f"[SLAVE] Frame {frame_idx}: "
                      f"Interval({interval_mbps:.2f} MB/s, {interval_fps:.1f} fps) | "
                      f"Total({total_mbps:.2f} MB/s, {total_fps:.1f} fps, {total_bytes/(1024*1024):.1f} MB) | "
                      f"TO_frm={frame_timeout_cnt} TO_chk={chunk_timeout_cnt}")

                last_stat_time = now
                last_stat_bytes = total_bytes
                last_stat_frames = frame_idx

            # 프리뷰 출력 + 간단 검증
            if args.preview_every > 0 and (frame_idx % args.preview_every) == 0:
                pr_raw = raw_frame[:32]
                pr_sw = frame[:32]

                if args.preview_raw:
                    print("[SLAVE][PREVIEW][RAW ] " + " ".join(f"{b:02X}" for b in pr_raw))
                print("[SLAVE][PREVIEW][SWAP] " + " ".join(f"{b:02X}" for b in pr_sw))

                if args.verify_preview:
                    v = verify_frame(frame, expected_cnt, sample_checks=64)
                    cnt = v["cnt"]
                    if expected_cnt is None and cnt is not None:
                        expected_cnt = cnt
                    elif expected_cnt is not None:
                        expected_cnt = (expected_cnt + 1) & 0xFFFFFFFF

                    if (not v["seq_ok"]) or (v["sample_mismatch"] > 0):
                        print(f"[WARN] Pattern mismatch: seq_ok={v['seq_ok']} "
                              f"sample_mis={v['sample_mismatch']} cnt={cnt}")
                        if not args.preview_raw:
                            print("[SLAVE][PREVIEW][RAW ] " + " ".join(f"{b:02X}" for b in pr_raw))
                        if args.stop_on_error:
                            print("[STOP] stop_on_error triggered.")
                            break

        # 10) Fast mode 일괄 저장
        if args.fast_mode and frame_buffer:
            print(f"\n[SLAVE] Capture finished. Saving {len(frame_buffer)} frames...")
            save_start = time.perf_counter()
            for i, fr in enumerate(frame_buffer, start=1):
                out_path = os.path.join(out_dir, f"adc_data_{i:06d}.bin")
                with open(out_path, "wb") as f:
                    f.write(fr)
                if i % 20 == 0:
                    print(f"\r[SLAVE] Saved {i}/{len(frame_buffer)}...", end="", flush=True)
            print()
            print(f"[SLAVE] Save elapsed: {time.perf_counter() - save_start:.2f}s")

        elapsed = time.perf_counter() - start_time
        fps = (frame_idx / elapsed) if elapsed > 0 else 0.0
        mbps = (total_bytes / (1024 * 1024)) / elapsed if elapsed > 0 else 0.0

        print("\n" + "=" * 60)
        print("           CAPTURE SUMMARY (SPI SLAVE MODE)")
        print("=" * 60)
        print(f"Frames captured : {frame_idx}")
        print(f"Total bytes     : {total_bytes:,} ({total_bytes/(1024*1024):.2f} MB)")
        print(f"Capture time    : {elapsed:.2f} s")
        print(f"Throughput      : {mbps:.2f} MB/s")
        print(f"Frame rate      : {fps:.2f} fps")
        print(f"Frame timeouts  : {frame_timeout_cnt}")
        print(f"Chunk timeouts  : {chunk_timeout_cnt}")
        print(f"Sync retries    : {sync_retries}")
        print(f"Output dir      : {out_dir}")
        print("=" * 60)

    finally:
        # 11) 정리
        try:
            dev_spi.close()
        except Exception:
            pass
        try:
            dev_gpio.close()
        except Exception:
            pass


def build_argparser() -> argparse.ArgumentParser: 
    """
    명령줄 인수 파서를 생성합니다.

    장치 선택, SPI 모드, GPIO 동기 옵션, flush/sync 옵션, 저장 옵션 등을 포함합니다.

    :returns: 설정된 ArgumentParser
    :rtype: argparse.ArgumentParser
    """
    p = argparse.ArgumentParser(
        description="FT4222H SPI SLAVE capture – frame mode (AWRL6844 master)"
    )

    p.add_argument("--list-devices", action="store_true")

    p.add_argument("--spi-desc", default="FT4222 A")
    p.add_argument("--gpio-desc", default="FT4222 B")
    p.add_argument("--spi-index", type=int, default=0)
    p.add_argument("--gpio-index", type=int, default=1)

    p.add_argument("--spi-mode", type=int, default=0, choices=[0, 1, 2, 3])

    p.add_argument("--initex", type=int, default=None,
                   help="Force spiSlave_InitEx(value). If not set, tries 1→2→0")

    p.add_argument("--host-intr-port", type=int, default=3, choices=[0, 1, 2, 3])
    p.add_argument("--intr-timeout-s", type=float, default=5.0)
    p.add_argument("--intr-stable-reads", type=int, default=0)
    p.add_argument("--intr-stable-sleep-us", type=int, default=10)

    p.add_argument("--wait-intr-high", action="store_true", default=True,
                   help="Wait for HOST_INTR HIGH after each frame/chunk (recommended)")
    p.add_argument("--no-wait-intr-high", dest="wait_intr_high", action="store_false")

    p.add_argument("--frame-size", type=int, default=DEFAULT_FRAME_SIZE,
                   help="Total bytes per frame (default: 65536 for 32 chirps)")
    p.add_argument("--chunk-size", type=int, default=DEFAULT_CHUNK_SIZE,
                   help="Max bytes per SPI transaction (default: 65536)")
    p.add_argument("--frames", type=int, default=100,
                   help="Number of frames to capture (0 = infinite)")

    p.add_argument("--spi-read-timeout-s", type=float, default=10.0,
                   help="Timeout for reading a frame/chunk")

    p.add_argument("--post-chunk-delay-us", type=int, default=0,
                   help="Delay after each chunk before waiting for next LOW")

    p.add_argument("--byteswap32", action="store_true", default=False,
                   help="Swap 32-bit words (for little-endian hosts)")
    p.add_argument("--no-byteswap32", dest="byteswap32", action="store_false")

    p.add_argument("--fast-mode", action="store_true", default=True,
                   help="Keep frames in memory, write all at the end")
    p.add_argument("--no-fast-mode", dest="fast_mode", action="store_false")

    p.add_argument("--out-dir", default="capture_out")
    p.add_argument("--clean", action="store_true", default=True)
    p.add_argument("--no-clean", dest="clean", action="store_false")

    p.add_argument("--log-every", type=int, default=10)
    p.add_argument("--preview-every", type=int, default=1)
    p.add_argument("--preview-raw", action="store_true")
    p.add_argument("--verify-preview", action="store_true")
    p.add_argument("--stop-on-error", action="store_true")

    p.add_argument("--sync-idle-high", action="store_true", default=True)
    p.add_argument("--no-sync-idle-high", dest="sync_idle_high", action="store_false")

    p.add_argument("--flush-before-start", action="store_true", default=True)
    p.add_argument("--no-flush-before-start", dest="flush_before_start", action="store_false")

    p.add_argument("--flush-each-frame", action="store_true", default=False)

    p.add_argument("--sync-frame-boundary", action="store_true", default=True,
                   help="Wait for frame boundary (HIGH->LOW) before starting capture")
    p.add_argument("--no-sync-frame-boundary", dest="sync_frame_boundary", action="store_false")

    p.add_argument("--full-reset", action="store_true", default=True,
                   help="Close and reopen devices before init (clean FIFO state)")
    p.add_argument("--no-full-reset", dest="full_reset", action="store_false")

    p.add_argument("--auto-sync", action="store_true", default=False,
                   help="Auto-detect frame header and retry if misaligned")
    p.add_argument("--no-auto-sync", dest="auto_sync", action="store_false")
    p.add_argument("--max-sync-retries", type=int, default=10,
                   help="Max retries for frame sync before giving up")
    p.add_argument("--debug-first-frame", action="store_true", default=True,
                   help="Print first frame raw data for debugging")
    p.add_argument("--no-debug-first-frame", dest="debug_first_frame", action="store_false")

    return p


if __name__ == "__main__":
    parser = build_argparser()
    args = parser.parse_args()
    capture(args)