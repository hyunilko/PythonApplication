# File: ft4222h_spi_data_streaming_slave_function.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ft4222h_spi_data_streaming_slave_function.py
===========================================

FT4222H SPI SLAVE MODE 기능 모듈

이 모듈은 FT4222H를 SPI Slave로 사용하여 AWRL6844 레이더 센서(SPI Master)로부터
ADC 데이터를 수신하는 공통 기능을 제공합니다. GUI 또는 CLI에서 참조 가능하도록
클래스와 함수로 구성되었습니다.

주요 클래스:
- SpiCapture: 캡처 로직을 담당하는 클래스. 콜백을 통해 상태 보고.

단독 실행 시 CLI 모드로 동작합니다 (ft4222h_spi_data_streaming_slave_simple.py 기반).

Dependencies:
- ft4222
- (optional for CLI) argparse

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
from typing import Optional, Any, Dict, List, Callable, Tuple

import ft4222
from ft4222.SPI import Cpha, Cpol
from ft4222.GPIO import Dir, Port

# ---------------- Default sizes ----------------
DEFAULT_FRAME_SIZE = 64 * 1024   # 32KB default (configurable)
API_SAFE_MAX_READ = 65535
DEFAULT_CHUNK_SIZE = 64 * 1024   # 65536 for simple CLI

# ======================== FT4222 helpers ========================

def list_ft4222_devices() -> List[Dict[str, Any]]:
    """
    시스템에 연결된 FT4222 장치 목록을 반환합니다.
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
    """
    return os.path.join(out_dir, f"adc_data_{frame_idx_1based:06d}.bin")


# ======================== GPIO helpers ========================

def gpio_init_input_all(dev_gpio):
    """
    GPIO P0~P3를 모두 입력 모드로 초기화합니다.
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
    """
    timeout_ms = int(timeout_s * 1000)
    target_level = level_low

    def is_level():
        v = dev_gpio.gpio_Read(port)
        is_low = (v is False) or (v == 0)
        return is_low == target_level

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
            time.sleep(0.0001)


def gpio_wait_low(dev_gpio, port: Port, timeout_s: float,
                  stable_reads: int = 0, stable_sleep_us: int = 10):
    """
    GPIO가 LOW 상태가 될 때까지 대기합니다(프레임 시작 신호).
    """
    return gpio_wait_level(dev_gpio, port, True, timeout_s,
                           stable_reads, stable_sleep_us)


def gpio_wait_high(dev_gpio, port: Port, timeout_s: float,
                   stable_reads: int = 0, stable_sleep_us: int = 10):
    """
    GPIO가 HIGH 상태가 될 때까지 대기합니다(프레임 종료/idle 신호).
    """
    return gpio_wait_level(dev_gpio, port, False, timeout_s,
                           stable_reads, stable_sleep_us)


# ======================== SPI Slave helpers ========================

def _find_spi_obj(dev_spi):
    """
    FT4222 디바이스에서 SPI Slave 기능을 제공하는 객체를 찾습니다(호환성).
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


def spi_slave_read_exact(spi, nbytes: int, timeout_s: float,
                         chunk_size: int = 16384) -> bytes:
    """
    지정된 바이트 수만큼 정확히 읽습니다(타임아웃 내).
    """
    out = bytearray()
    t0 = time.perf_counter()
    stall_count = 0

    while len(out) < nbytes:
        if (time.perf_counter() - t0) > timeout_s:
            raise TimeoutError(f"read_exact: got {len(out)}/{nbytes} bytes")

        remain = nbytes - len(out)
        want = min(remain, chunk_size, API_SAFE_MAX_READ)
        b = spi_slave_read_once(spi, want)

        if not b:
            stall_count += 1
            if stall_count > 100:
                time.sleep(0.001)
            else:
                time.sleep(0.0001)
            continue

        stall_count = 0
        out.extend(b)

    return bytes(out)


def spi_slave_flush(spi, max_bytes: int = 2_000_000) -> int:
    """
    SPI Slave RX 버퍼의 잔여 데이터를 읽어서 폐기합니다(플러시).
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
    APP_PDU 프레임 헤더 패턴을 간단히 확인합니다.
    """
    if len(data) < 4:
        return False
    return (data[0] == 0x12 and data[1] == 0x34 and
            data[2] == 0x56 and data[3] == 0x78)


def parse_app_pdu_header(data: bytes) -> dict:
    """
    APP_PDU 헤더를 파싱합니다.
    """
    import struct
    if len(data) < 16:
        return {}

    return {
        "frame_id": struct.unpack(">I", data[0:4])[0],
        "frame_count": struct.unpack(">I", data[4:8])[0],
        "chunk_length": struct.unpack(">H", data[8:10])[0],
        "payload_crc": struct.unpack(">H", data[10:12])[0],
        "payload_length": struct.unpack(">I", data[12:16])[0],
    }


def find_frame_sync_offset(data: bytes, debug: bool = False) -> int:
    """
    APP_PDU 프레임 헤더 패턴을 기반으로 동기 오프셋을 찾습니다.
    """
    if len(data) < 16:
        return -1

    for i in range(min(len(data) - 16, 4096)):
        if (data[i] == 0x12 and data[i+1] == 0x34 and
            data[i+2] == 0x56 and data[i+3] == 0x78):
            if debug:
                print(f"[SYNC DEBUG] Found APP_PDU header at offset {i}")
            return i

    if debug:
        print(f"[SYNC DEBUG] APP_PDU pattern (12 34 56 78) not found in first {min(len(data), 4096)} bytes")

    return -1


def spi_slave_init_stable(dev_spi, cpol, cpha, initex_override: Optional[int] = None):
    """
    SPI Slave 모드를 초기화합니다(다양한 API 버전 호환).
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


def verify_frame(frame: bytes, expected_cnt: Optional[int], sample_checks: int = 256) -> dict:
    """
    APP_PDU 프레임 데이터의 간이 무결성 검증을 수행합니다.

    APP_PDU 헤더 구조 (16 bytes):
    - Bytes 0-3: Frame ID = 0x12345678 (고정 매직 넘버)
    - Bytes 4-7: Frame count (big-endian)
    - Bytes 8-9: Max chunk length (big-endian)
    - Bytes 10-11: Payload CRC (big-endian)
    - Bytes 12-15: Payload total length (big-endian)
    - Bytes 16+: Payload

    :param bytes frame: 검증할 프레임 데이터
    :param expected_cnt: 예상 프레임 카운터(None이면 카운터 비교 생략)
    :type expected_cnt: int | None
    :param int sample_checks: 샘플링 검사 횟수
    :returns: 검증 결과 dict
    :rtype: dict
    """
    res = {
        "frame_id_ok": False,
        "cnt": None,
        "cnt_ok": False,
        "payload_length": None,
        "samples_ok": 0,
        "samples_checked": 0,
        "samples_failed": 0,
    }
    if len(frame) < 16:
        return res
    import struct
    frame_id = struct.unpack(">I", frame[0:4])[0]
    res["frame_id_ok"] = frame_id == 0x12345678
    res["cnt"] = struct.unpack(">I", frame[4:8])[0]
    if expected_cnt is not None:
        res["cnt_ok"] = res["cnt"] == expected_cnt
    res["payload_length"] = struct.unpack(">I", frame[12:16])[0]
    # Sample pattern check (from byte 16)
    payload = frame[16:]
    res["samples_checked"] = min(sample_checks, len(payload))
    for i in range(res["samples_checked"]):
        expected = i % 256
        if payload[i] == expected:
            res["samples_ok"] += 1
        else:
            res["samples_failed"] += 1
    return res


# ======================== Capture Settings and Worker ========================

@dataclass
class CaptureSettings:
    spi_dev_index: int
    gpio_dev_index: int
    spi_desc: str
    gpio_desc: str
    spi_mode: int
    initex_mode: Optional[int]
    num_frames: int
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
    fast_mode: bool
    variable_frame: bool
    read_chunk_size: int
    preview_payload_only: bool = False
    preview_raw: bool = False
    verify_preview: bool = False
    stop_on_error: bool = False
    preview_detailed: bool = False


class SpiCapture:
    """
    SPI Slave 캡처 클래스 (콜백 기반).
    
    콜백 함수:
    - status_callback(str)
    - error_callback(str)
    - progress_callback(int, int)
    - preview_callback(bytes)
    - finished_callback()
    """
    def __init__(self, settings: CaptureSettings,
                 status_callback: Callable[[str], None] = print,
                 error_callback: Callable[[str], None] = print,
                 progress_callback: Callable[[int, int], None] = None,
                 preview_callback: Callable[[bytes], None] = None,
                 finished_callback: Callable[[], None] = None):
        self.settings = settings
        self.status_callback = status_callback
        self.error_callback = error_callback
        self.progress_callback = progress_callback or (lambda c, t: None)
        self.preview_callback = preview_callback or (lambda b: None)
        self.finished_callback = finished_callback or (lambda: None)
        self._running = False
        self._dev_spi = None
        self._dev_gpio = None
        self._spi = None

    def start(self):
        self._running = True
        self.run()

    def stop(self):
        self._running = False

    def run(self):
        try:
            # 장치 열기
            self._dev_spi = open_by_desc_or_index(self.settings.spi_desc, self.settings.spi_dev_index)
            self._dev_gpio = open_by_desc_or_index(self.settings.gpio_desc, self.settings.gpio_dev_index) if self.settings.gpio_desc != self.settings.spi_desc else self._dev_spi

            # 초기화
            gpio_init_input_all(self._dev_gpio)
            cpol, cpha = spi_mode_to_cpol_cpha(self.settings.spi_mode)
            self._spi = spi_slave_init_stable(self._dev_spi, cpol, cpha, self.settings.initex_mode)

            # 플러시 등
            if self.settings.flush_before_start:
                spi_slave_flush_aggressive(self._spi)

            # 캡처 루프 (gui.py와 simple.py의 로직 합침)
            frame_count = 0
            total_bytes = 0
            start_t = time.perf_counter()
            self.frame_timeout_cnt = 0
            self.chunk_timeout_cnt = 0
            last_stat_time = start_t
            last_stat_bytes = 0
            last_stat_frames = 0
            frame_buffer = [] if self.settings.fast_mode else None
            carry_over = b""
            aligned = False
            expected_cnt = None

            total_for_progress = self.settings.num_frames if self.settings.num_frames > 0 else 0

            intr_port = port_enum(self.settings.host_intr_port)

            while self._running and (self.settings.num_frames == 0 or frame_count < self.settings.num_frames):
                try:
                    gpio_wait_low(self._dev_gpio, intr_port, self.settings.timeout_s,
                                  self.settings.intr_stable_reads, self.settings.intr_stable_sleep_us)
                except TimeoutError:
                    self.status_callback(f"[FRAME {frame_count+1}] Timeout waiting INTR LOW")
                    self.frame_timeout_cnt += 1
                    continue

                try:
                    if self.settings.variable_frame:
                        if len(carry_over) >= 16:
                            header_data = carry_over[:16]
                            carry_over = carry_over[16:]
                        else:
                            need_header = 16 - len(carry_over)
                            new_header = spi_slave_read_exact(self._spi, need_header, self.settings.spi_read_timeout_s,
                                                              self.settings.read_chunk_size)
                            header_data = carry_over + new_header
                            carry_over = b""

                        if not check_frame_header_simple(header_data):
                            self.status_callback(f"[FRAME {frame_count+1}] Header mismatch")
                            offset = find_frame_sync_offset(header_data + spi_slave_read_once(self._spi, 4096))
                            if offset > 0:
                                carry_over = header_data[offset:]
                            else:
                                spi_slave_flush(self._spi)
                                carry_over = b""
                            continue

                        hdr = parse_app_pdu_header(header_data)
                        payload_len = hdr["payload_length"]
                        if payload_len == 0 or payload_len > 65536:
                            self.status_callback(f"[FRAME {frame_count+1}] Invalid payload_length={payload_len}")
                            spi_slave_flush(self._spi)
                            carry_over = b""
                            continue

                        if len(carry_over) >= payload_len:
                            payload_data = carry_over[:payload_len]
                            carry_over = carry_over[payload_len:]
                        else:
                            need_payload = payload_len - len(carry_over)
                            new_payload = spi_slave_read_exact(self._spi, need_payload, self.settings.spi_read_timeout_s,
                                                               self.settings.read_chunk_size)
                            payload_data = carry_over + new_payload
                            carry_over = b""

                        raw_frame = header_data + payload_data
                    else:
                        need_bytes = self.settings.frame_size - len(carry_over)
                        new_data = spi_slave_read_exact(self._spi, need_bytes, self.settings.spi_read_timeout_s,
                                                        self.settings.read_chunk_size)
                        raw_frame = carry_over + new_data
                        carry_over = b""

                except TimeoutError as e:
                    self.status_callback(f"[FRAME {frame_count+1}] Read timeout: {e}")
                    self.chunk_timeout_cnt += 1
                    spi_slave_flush(self._spi)
                    carry_over = b""
                    aligned = False
                    continue

                if self.settings.wait_intr_high:
                    try:
                        gpio_wait_high(self._dev_gpio, intr_port, self.settings.timeout_s,
                                       self.settings.intr_stable_reads, self.settings.intr_stable_sleep_us)
                    except TimeoutError:
                        self.status_callback(f"[FRAME {frame_count+1}] Timeout waiting INTR HIGH")
                        self.frame_timeout_cnt += 1

                test_frame = byteswap32_fast(raw_frame) if self.settings.byteswap32 else raw_frame
                if not aligned:
                    offset = find_frame_sync_offset(test_frame)
                    if offset == 0:
                        aligned = True
                    elif offset > 0:
                        carry_over = raw_frame[offset:] + spi_slave_read_exact(self._spi, offset, self.settings.spi_read_timeout_s)
                        if len(carry_over) >= self.settings.frame_size:
                            raw_frame = carry_over[:self.settings.frame_size]
                            carry_over = carry_over[self.settings.frame_size:]
                            aligned = True
                            test_frame = byteswap32_fast(raw_frame) if self.settings.byteswap32 else raw_frame
                        else:
                            continue
                    else:
                        self.status_callback(f"[SYNC] Frame {frame_count+1}: Header not found, skipping...")
                        spi_slave_flush(self._spi)
                        continue

                frame = test_frame

                frame_count += 1
                total_bytes += len(frame)

                if self.settings.save_to_file:
                    if self.settings.fast_mode:
                        frame_buffer.append(frame)
                    else:
                        out_path = make_frame_path(self.settings.out_dir, frame_count)
                        with open(out_path, "wb") as f:
                            f.write(frame)

                self.progress_callback(frame_count, total_for_progress)

                if self.settings.preview_every > 0 and (frame_count % self.settings.preview_every) == 0:
                    self.preview_callback(frame[:64])

                    if self.settings.preview_detailed:
                        # 프리뷰 출력 + 간단 검증
                        # Payload starts at byte 16 (after 16-byte header)
                        if self.settings.preview_payload_only:
                            pr_raw = raw_frame[16:48]  # payload bytes 0-31
                            pr_sw = frame[16:48]
                            preview_label = "PAYLOAD"
                        else:
                            pr_raw = raw_frame[:32]  # includes header
                            pr_sw = frame[:32]
                            preview_label = "SWAP"
                        if self.settings.preview_raw:
                            self.status_callback(f"[SLAVE][PREVIEW][RAW ] " + " ".join(f"{b:02X}" for b in pr_raw))
                        self.status_callback(f"[SLAVE][PREVIEW][{preview_label}] " + " ".join(f"{b:02X}" for b in pr_sw))
                        if self.settings.verify_preview:
                            v = verify_frame(frame, expected_cnt)
                            cnt = v["cnt"]
                            if expected_cnt is None and cnt is not None:
                                expected_cnt = cnt
                            elif expected_cnt is not None:
                                expected_cnt = (expected_cnt + 1) & 0xFFFFFFFF
                            if not v["frame_id_ok"]:
                                self.status_callback(f"[WARN] Frame ID mismatch! cnt={cnt} "
                                              f"payload_len={v['payload_length']}")
                                if not self.settings.preview_raw:
                                    self.status_callback("[SLAVE][PREVIEW][RAW ] " + " ".join(f"{b:02X}" for b in pr_raw))
                                if self.settings.stop_on_error:
                                    self.status_callback("[STOP] stop_on_error triggered.")
                                    break

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

                    self.status_callback(
                        f"[SLAVE] Frame {frame_count}: "
                        f"Interval({interval_mbps:.2f} MB/s, {interval_fps:.1f} fps) | "
                        f"Total({total_mbps:.2f} MB/s, {total_fps:.1f} fps, {total_bytes/(1024*1024):.1f} MB) | "
                        f"TO_frm={self.frame_timeout_cnt} TO_chk={self.chunk_timeout_cnt}"
                    )

                    last_stat_time = now
                    last_stat_bytes = total_bytes
                    last_stat_frames = frame_count

            if self.settings.fast_mode and self.settings.save_to_file and frame_buffer:
                self.status_callback(f"\n[SAVE] Writing {len(frame_buffer)} frames to disk...")
                for i, frm in enumerate(frame_buffer, start=1):
                    out_path = make_frame_path(self.settings.out_dir, i)
                    with open(out_path, "wb") as f:
                        f.write(frm)
                frame_buffer.clear()

            elapsed = time.perf_counter() - start_t
            fps = (frame_count / elapsed) if elapsed > 0 else 0.0
            mbps = (total_bytes / (1024 * 1024)) / elapsed if elapsed > 0 else 0.0

            self.status_callback("\n" + "=" * 60)
            self.status_callback("           CAPTURE SUMMARY (SPI SLAVE MODE)")
            self.status_callback("=" * 60)
            self.status_callback(f"Frames captured : {frame_count}")
            self.status_callback(f"Total bytes     : {total_bytes:,} ({total_bytes/(1024*1024):.2f} MB)")
            self.status_callback(f"Capture time    : {elapsed:.2f} s")
            self.status_callback(f"Throughput      : {mbps:.2f} MB/s")
            self.status_callback(f"Frame rate      : {fps:.2f} fps")
            self.status_callback(f"Frame timeouts  : {self.frame_timeout_cnt}")
            self.status_callback(f"Chunk timeouts  : {self.chunk_timeout_cnt}")
            self.status_callback(f"Output dir      : {self.settings.out_dir}")
            self.status_callback("=" * 60)

        except Exception as e:
            import traceback
            self.error_callback(f"Capture error: {e}\n{traceback.format_exc()}")

        finally:
            self._cleanup()
            self.finished_callback()

    def _cleanup(self):
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
        self.status_callback("Devices closed")


# ======================== CLI Mode (단독 실행 시) ========================

def build_argparser() -> 'argparse.ArgumentParser':
    import argparse
    p = argparse.ArgumentParser(
        description="FT4222H SPI SLAVE capture – frame mode (AWRL6844 master)"
    )

    # Similar to simple.py's parser
    p.add_argument("--list-devices", action="store_true")
    p.add_argument("--spi-desc", default="FT4222 A")
    p.add_argument("--gpio-desc", default="FT4222 B")
    p.add_argument("--spi-index", type=int, default=0)
    p.add_argument("--gpio-index", type=int, default=1)
    p.add_argument("--spi-mode", type=int, default=0, choices=[0, 1, 2, 3])
    p.add_argument("--initex", type=int, default=None)
    p.add_argument("--host-intr-port", type=int, default=3, choices=[0, 1, 2, 3])
    p.add_argument("--intr-timeout-s", type=float, default=5.0)
    p.add_argument("--intr-stable-reads", type=int, default=0)
    p.add_argument("--intr-stable-sleep-us", type=int, default=10)
    p.add_argument("--wait-intr-high", action="store_true", default=True)
    p.add_argument("--frame-size", type=int, default=DEFAULT_FRAME_SIZE)
    p.add_argument("--frames", type=int, default=20)
    p.add_argument("--spi-read-timeout-s", type=float, default=10.0)
    p.add_argument("--read-chunk-size", type=int, default=16384)
    p.add_argument("--byteswap32", action="store_true", default=False)
    p.add_argument("--fast-mode", action="store_true", default=True)
    p.add_argument("--out-dir", default="capture_out")
    p.add_argument("--overwrite-on-start", action="store_true", default=True)
    p.add_argument("--preview-every", type=int, default=1)
    p.add_argument("--log-every", type=int, default=10)
    p.add_argument("--flush-before-start", action="store_true", default=True)
    p.add_argument("--flush-each-frame", action="store_true", default=False)
    p.add_argument("--sync-frame-boundary", action="store_true", default=True)
    p.add_argument("--full-reset", action="store_true", default=True)
    p.add_argument("--variable-frame", action="store_true", default=True)
    p.add_argument("--save-to-file", action="store_true", default=True)
    p.add_argument("--preview-payload-only", action="store_true", default=False)
    p.add_argument("--preview-raw", action="store_true")
    p.add_argument("--verify-preview", action="store_true")
    p.add_argument("--stop-on-error", action="store_true")
    p.add_argument("--preview-detailed", action="store_true", default=True)

    return p

if __name__ == "__main__":
    print("[SLAVE][PREVIEW][SWAP] 12 34 56 78 00 00 EF 1E FF FE AB CD 00 00 00 48 DE 20 00 00 00 01 01 1E D8 19 00 00 01 07 01 1E")
    parser = build_argparser()
    args = parser.parse_args()

    if args.list_devices:
        devices = list_ft4222_devices()
        for d in devices:
            print(f"[{d['index']}] {d['desc']} (SN: {d['serial']})")
        sys.exit(0)

    settings = CaptureSettings(
        spi_dev_index=args.spi_index,
        gpio_dev_index=args.gpio_index,
        spi_desc=args.spi_desc,
        gpio_desc=args.gpio_desc,
        spi_mode=args.spi_mode,
        initex_mode=args.initex,
        num_frames=args.frames,
        frame_size=args.frame_size,
        host_intr_port=args.host_intr_port,
        timeout_s=args.intr_timeout_s,
        intr_stable_reads=args.intr_stable_reads,
        intr_stable_sleep_us=args.intr_stable_sleep_us,
        wait_intr_high=args.wait_intr_high,
        full_reset=args.full_reset,
        flush_before_start=args.flush_before_start,
        flush_each_frame=args.flush_each_frame,
        sync_frame_boundary=args.sync_frame_boundary,
        save_to_file=args.save_to_file,
        out_dir=args.out_dir,
        overwrite_on_start=args.overwrite_on_start,
        preview_every=args.preview_every,
        log_every=args.log_every,
        byteswap32=args.byteswap32,
        spi_read_timeout_s=args.spi_read_timeout_s,
        fast_mode=args.fast_mode,
        variable_frame=args.variable_frame,
        read_chunk_size=args.read_chunk_size,
        preview_payload_only=args.preview_payload_only,
        preview_raw=args.preview_raw,
        verify_preview=args.verify_preview,
        stop_on_error=args.stop_on_error,
        preview_detailed=args.preview_detailed,
    )

    if settings.overwrite_on_start:
        delete_existing_frame_files(settings.out_dir)

    capture = SpiCapture(settings)
    capture.start()