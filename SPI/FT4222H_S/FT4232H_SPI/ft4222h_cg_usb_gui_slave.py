#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ft4222h_cg_usb_gui_slave.py  (FT4222H SPI SLAVE MODE)

Firmware:
- adcDataPerFrame = 32768 bytes (32KB default, configurable)
- Single frame read per HOST_INTR LOW signal

Host (FT4222H as SPI Slave):
- AWRL6844 is SPI Master (drives SCLK/CS)
- Wait HOST_INTR LOW, read frame data, wait HIGH
- Apply byteswap32 to reconstruct original byte stream

Dependency:
- pip install ft4222 PyQt6
- Windows: FTDI LibFT4222 / D2XX driver installed
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
    Returns list of dict:
      { index:int, desc:str, serial:str }
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
    """Fallback for environments where ft4222.open() does not accept index."""
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
    """Try openByDescription(desc), fallback open(index)."""
    try:
        return ft4222.openByDescription(desc)
    except TypeError:
        return ft4222.openByDescription(desc.encode("utf-8"))
    except Exception:
        if index is None:
            raise
        return open_by_index_compat(index)


def byteswap32_fast(data: bytes) -> bytes:
    """Fast 32-bit byteswap using array('I').byteswap()."""
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
    """Delete adc_data_*.bin in out_dir; return removed count."""
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
    return os.path.join(out_dir, f"adc_data_{frame_idx_1based:06d}.bin")


# ======================== GPIO helpers ========================

def gpio_init_input_all(dev_gpio):
    try:
        dev_gpio.gpio_Init(gpio0=Dir.INPUT, gpio1=Dir.INPUT,
                           gpio2=Dir.INPUT, gpio3=Dir.INPUT)
    except TypeError:
        dev_gpio.gpio_Init(Dir.INPUT, Dir.INPUT, Dir.INPUT, Dir.INPUT)


def gpio_wait_level(dev_gpio, port, level_low, timeout_s,
                    stable_reads=0, stable_sleep_us=20):
    """
    GPIO 핀이 특정 레벨(LOW/HIGH)이 될 때까지 대기.
    - stable_reads: 연속으로 같은 레벨을 읽어야 성공으로 간주하는 횟수
    - stable_sleep_us: 안정화 읽기 사이의 지연 시간 (마이크로초)
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
    return gpio_wait_level(dev_gpio, port, True, timeout_s,
                           stable_reads, stable_sleep_us)


def gpio_wait_high(dev_gpio, port: Port, timeout_s: float,
                   stable_reads: int = 0, stable_sleep_us: int = 10):
    return gpio_wait_level(dev_gpio, port, False, timeout_s,
                           stable_reads, stable_sleep_us)


# ======================== SPI Slave helpers ========================

def _find_spi_obj(dev_spi):
    for name in ("spiSlave", "spislave", "SPI", "spi"):
        if hasattr(dev_spi, name):
            obj = getattr(dev_spi, name)
            if any(hasattr(obj, m) for m in ("spiSlave_InitEx", "spiSlave_Init",
                                              "spiSlave_Read", "spiSlave_GetRxStatus")):
                return obj
    return dev_spi


def spi_slave_get_rx_available(spi) -> Optional[int]:
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


def spi_slave_init_stable(dev_spi, cpol, cpha, initex_override: Optional[int] = None):
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

    save_to_file: bool
    out_dir: str
    overwrite_on_start: bool

    preview_every: int
    log_every: int
    byteswap32: bool
    spi_read_timeout_s: float


class SpiCaptureWorker(QObject):
    status = pyqtSignal(str)
    error = pyqtSignal(str)
    progress = pyqtSignal(int, int)
    preview = pyqtSignal(bytes)
    finished = pyqtSignal()

    def __init__(self, settings: CaptureSettings):
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
                time.sleep(0.5)

                self._dev_spi = open_by_desc_or_index(self.settings.spi_desc, self.settings.spi_dev_index)
                self._dev_gpio = open_by_desc_or_index(self.settings.gpio_desc, self.settings.gpio_dev_index)
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

            # Stabilization delay and flush
            time.sleep(0.001)
            if self.settings.flush_before_start:
                flushed = spi_slave_flush(self._spi)
                if flushed > 0:
                    self.status.emit(f"[SPI] RX flush before start: {flushed} bytes discarded")

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

                frame_count += 1
                total_bytes += len(frame)

                # Save to file
                if self.settings.save_to_file:
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
        self._running = False

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
        self.status.emit("Devices closed")


# ======================== GUI ========================

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AWRL6844 SPI Data Capture Tool (FT4222H SPI SLAVE)")
        self.resize(1100, 900)

        self.capture_thread: Optional[QThread] = None
        self.capture_worker: Optional[SpiCaptureWorker] = None

        self._build_ui()
        self._refresh_devices()

    def _build_ui(self):
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

        self.cb_byteswap = QCheckBox("Byteswap32")
        self.cb_byteswap.setChecked(True)

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

        self.cb_overwrite = QCheckBox("Overwrite on start")
        self.cb_overwrite.setChecked(True)

        self.out_dir = QLineEdit()
        self.out_dir.setText(os.path.abspath("capture_out"))

        self.btn_browse = QPushButton("Browse folder...")
        self.btn_browse.clicked.connect(self._browse_folder)

        file_layout.addWidget(self.cb_save)
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
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_view.appendPlainText(f"[{ts}] {msg}")
        c = self.log_view.textCursor()
        c.movePosition(c.MoveOperation.End)
        self.log_view.setTextCursor(c)

    def _refresh_devices(self):
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
        path = QFileDialog.getExistingDirectory(self, "Select Output Folder", self.out_dir.text())
        if path:
            self.out_dir.setText(path)

    def _start_capture(self):
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
            save_to_file=self.cb_save.isChecked(),
            out_dir=out_dir,
            overwrite_on_start=self.cb_overwrite.isChecked(),
            preview_every=self.spin_preview.value(),
            log_every=self.spin_log.value(),
            byteswap32=self.cb_byteswap.isChecked(),
            spi_read_timeout_s=float(self.spin_read_timeout.value()),
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
        self._log(f"Byteswap32      : {'ON' if settings.byteswap32 else 'OFF'}")
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
        if self.capture_worker:
            self.capture_worker.stop()
        self._log("Stop requested...")

    def _on_status(self, msg: str):
        self.status_label.setText(msg[:100])
        self._log(msg)

    def _on_error(self, msg: str):
        self._log(f"ERROR: {msg}")
        QMessageBox.critical(self, "Error", msg)

    def _on_progress(self, current: int, total: int):
        if total > 0:
            self.progress.setMaximum(total)
            self.progress.setValue(current)
        else:
            self.progress.setValue(current % 100)

    def _on_preview(self, data: bytes):
        hex_str = " ".join(f"{b:02X}" for b in data)
        self._log(f"[PREVIEW] {hex_str}")

    def _on_finished(self):
        if self.capture_thread:
            self.capture_thread.quit()
            self.capture_thread.wait(2000)

        self.capture_thread = None
        self.capture_worker = None
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self._log("Capture finished")


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
