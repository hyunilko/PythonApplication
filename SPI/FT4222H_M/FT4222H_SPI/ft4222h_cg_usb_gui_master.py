#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ft4222h_cg_usb_gui.py  (FT4222H / FW header = u32_to_be(cnt))

Firmware:
- adcDataPerFrame = 131072 bytes
- 2 chunks x 65536
- Frame header:
    byte[0..3] = u32_to_be(cnt)
    byte[4..]  = (i-4) & 0xFF sequence

Host (FT4222H):
- Wait HOST_INTR READY before each chunk, then SPI read exactly 65536 bytes
- Optional post-intr delay (us)
- Apply byteswap32 to reconstruct original byte stream

Requested change:
- GUI out_path => select OUTPUT FOLDER
- Auto save per-frame files:
    adc_data_000001.bin, adc_data_000002.bin, ...

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
from ft4222.SPIMaster import Mode, Clock, SlaveSelect
from ft4222.GPIO import Dir, Port

from PyQt6.QtCore import QThread, pyqtSignal, QObject, pyqtSlot
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QPlainTextEdit, QLineEdit,
    QCheckBox, QMessageBox, QGroupBox, QFileDialog, QSpinBox,
    QProgressBar
)

# ---------------- Firmware-fixed sizes ----------------
FW_FRAME_SIZE = 131072
CHUNK_SIZE = 65536
CHUNKS_PER_FRAME = FW_FRAME_SIZE // CHUNK_SIZE

# NOTE: translated to English.
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


def open_by_desc_or_index(desc: str, index: Optional[int]):
    """Try openByDescription(desc), fallback open(index)."""
    try:
        return ft4222.openByDescription(desc)
    except Exception:
        if index is None:
            raise
        return ft4222.open(index)


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


def decode_fw_frame_no(u32_be: int) -> int:
    return int(u32_be)


def chunk1_signature(byteswapped: bool) -> bytes:
    return b"\xFC\xFD\xFE\xFF" if byteswapped else b"\xFF\xFE\xFD\xFC"


def looks_like_frame_start(chunk: bytes, byteswapped: bool) -> bool:
    """
    Frame start (chunk0) heuristic:
    - chunk1은 시작 4바이트가 FC FD FE FF(byteswap ON 기준)로 보이므로 배제
    - chunk0는 [4..15]가 항상 00 01 02 ... 0B
    """
    if len(chunk) < 16:
        return False
    if chunk[:4] == chunk1_signature(byteswapped):
        return False
    if chunk[4:16] != bytes(range(0x00, 0x0C)):
        return False
    return True


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


def ss_enum(ss: int) -> SlaveSelect:
    if ss == 0:
        return SlaveSelect.SS0
    if ss == 1:
        return SlaveSelect.SS1
    if ss == 2:
        return SlaveSelect.SS2
    if ss == 3:
        return SlaveSelect.SS3
    raise ValueError("SS must be 0..3")


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


def set_sysclock_for_target(dev_spi, target_hz: int) -> int:
    """
    Set FT4222H system clock to best match target SPI frequency.
    Returns the actual system clock in Hz.

    For 30MHz SPI: need 60MHz sysclock (60/2=30)
    For 24MHz SPI: need 48MHz sysclock (48/2=24) or 24MHz (24/1, but no DIV_1)
    """
    if not hasattr(ft4222, "SysClock"):
        return 60_000_000  # assume default

    # Choose sysclock that can produce target_hz with available dividers
    # Available dividers: 2, 4, 8, 16, 32, 64, 128, 256, 512
    sysclock_options = [
        (ft4222.SysClock.CLK_80, 80_000_000),
        (ft4222.SysClock.CLK_60, 60_000_000),
        (ft4222.SysClock.CLK_48, 48_000_000),
        (ft4222.SysClock.CLK_24, 24_000_000),
    ]
    dividers = [2, 4, 8, 16, 32, 64, 128, 256, 512]

    best_sysclk_enum = ft4222.SysClock.CLK_60
    best_sysclk_hz = 60_000_000
    best_diff = float('inf')

    for sysclk_enum, sysclk_hz in sysclock_options:
        for div in dividers:
            actual_spi = sysclk_hz // div
            if actual_spi <= target_hz:
                diff = target_hz - actual_spi
                if diff < best_diff:
                    best_diff = diff
                    best_sysclk_enum = sysclk_enum
                    best_sysclk_hz = sysclk_hz
                break

    try:
        dev_spi.setClock(best_sysclk_enum)
    except Exception:
        pass

    return best_sysclk_hz


def pick_clock_div_for_target(dev_spi, target_hz: int) -> tuple:
    """
    FT4222: SCK = SysClock / DIV
    Sets system clock first, then picks best divider.
    Returns (Clock enum, actual SPI frequency).
    """
    sys_hz = set_sysclock_for_target(dev_spi, target_hz)

    table = [
        (Clock.DIV_2,   sys_hz // 2),
        (Clock.DIV_4,   sys_hz // 4),
        (Clock.DIV_8,   sys_hz // 8),
        (Clock.DIV_16,  sys_hz // 16),
        (Clock.DIV_32,  sys_hz // 32),
        (Clock.DIV_64,  sys_hz // 64),
        (Clock.DIV_128, sys_hz // 128),
        (Clock.DIV_256, sys_hz // 256),
        (Clock.DIV_512, sys_hz // 512),
    ]

    best = table[-1]
    for div_enum, hz in table:
        if hz <= target_hz:
            best = (div_enum, hz)
            break

    return best


def pick_spi_single_mode():
    candidates = ["SPI_IO_SINGLE", "SINGLE", "SINGLE_IO", "IO_SINGLE", "SPI_SINGLE", "SINGLE_MODE"]
    for name in candidates:
        if hasattr(Mode, name):
            return getattr(Mode, name)
    for name in dir(Mode):
        if "SINGLE" in name.upper():
            return getattr(Mode, name)
    raise AttributeError(f"Cannot find SINGLE mode in ft4222.SPIMaster.Mode. Available: {dir(Mode)}")


def spi_master_init_compat(dev_spi, clk_div, cpol, cpha, ss):
    io_mode = pick_spi_single_mode()
    try:
        dev_spi.spiMaster_Init(io_mode, clk_div, cpol, cpha, ss)
    except TypeError:
        dev_spi.spiMaster_Init(clk_div, cpol, cpha, ss)


def spi_single_read_compat(dev_spi, length: int, is_end: bool) -> bytes:
    try:
        return dev_spi.spiMaster_SingleRead(length, is_end)
    except TypeError:
        try:
            return dev_spi.spiMaster_SingleRead(length, isEndTransaction=is_end)
        except TypeError:
            b = dev_spi.spiMaster_SingleRead(length)
            if is_end and hasattr(dev_spi, "spiMaster_EndTransaction"):
                try:
                    dev_spi.spiMaster_EndTransaction()
                except Exception:
                    pass
            return b


def spi_read_keep_cs(dev_spi, nbytes: int) -> bytes:
    """CS 유지한 채로 정확히 nbytes 읽기 (65536 -> 65535 + 1)"""
    out = bytearray()
    remain = nbytes
    while remain > 0:
        part = API_SAFE_MAX_READ if remain > API_SAFE_MAX_READ else remain
        remain -= part
        is_end = (remain == 0)
        b = spi_single_read_compat(dev_spi, part, is_end)
        if len(b) != part:
            raise RuntimeError(f"spiMaster_SingleRead returned {len(b)} bytes, expected {part}")
        out.extend(b)
    return bytes(out)


def wait_intr_ready(dev_gpio, port: Port, active_low: bool, timeout_s: float, poll_sleep_us: int) -> bool:
    """
    active_low=True 이면 LOW(False)일 때 ready.
    Returns True if ready detected, False if timeout.
    """
    timeout_ms = int(timeout_s * 1000)
    want = False if active_low else True

    if hasattr(dev_gpio, "gpio_Wait"):
        try:
            dev_gpio.gpio_Wait(port, want, timeout=timeout_ms, sleep=0)
            return True
        except TypeError:
            try:
                dev_gpio.gpio_Wait(port, want, timeout_ms)
                return True
            except Exception:
                pass

    t0 = time.perf_counter()
    sleep_s = max(poll_sleep_us, 0) / 1_000_000.0
    while True:
        v = dev_gpio.gpio_Read(port)
        if v == want:
            return True
        if (time.perf_counter() - t0) > timeout_s:
            return False
        if sleep_s > 0:
            time.sleep(sleep_s)


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


# ======================== Capture Worker ========================

@dataclass
class CaptureSettings:
    spi_dev_index: int
    gpio_dev_index: int
    spi_desc: str
    gpio_desc: str

    clock_hz: int
    spi_mode: int
    ss: int

    num_frames: int  # 0=infinite

    host_intr_port: int
    host_intr_active_low: bool
    timeout_s: float
    poll_sleep_us: int
    post_intr_delay_us: int

    save_to_file: bool
    out_dir: str                 # <- folder
    overwrite_on_start: bool     # NOTE: translated to English.

    preview_every: int
    log_every: int
    byteswap32: bool
    resync_on_start: bool


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

        # Stats
        self.timeouts = 0
        self.sig_mismatch = 0
        self.frame_gaps = 0
        self.resync_discards = 0

    @pyqtSlot()
    def start(self):
        self._running = True
        last_frame_no: Optional[int] = None

        try:
            self.status.emit(f"Firmware frame size: {FW_FRAME_SIZE:,} bytes ({CHUNKS_PER_FRAME} x 64KB)")
            self.status.emit(f"HOST_INTR: P{self.settings.host_intr_port} (active_low={self.settings.host_intr_active_low})")
            self.status.emit(f"SPI: {self.settings.clock_hz/1e6:.1f}MHz, mode={self.settings.spi_mode}, SS{self.settings.ss}")
            self.status.emit(f"byteswap32: {'ON' if self.settings.byteswap32 else 'OFF'}")
            self.status.emit(f"Resync on start: {'ON' if self.settings.resync_on_start else 'OFF'}")

            if self.settings.save_to_file:
                if not self.settings.out_dir:
                    raise RuntimeError("Output folder is empty. Please select an output folder.")
                os.makedirs(self.settings.out_dir, exist_ok=True)
                if self.settings.overwrite_on_start:
                    removed = delete_existing_frame_files(self.settings.out_dir, "adc_data_*.bin")
                    self.status.emit(f"Deleted {removed} existing file(s) in: {self.settings.out_dir}")
                self.status.emit(f"Saving per-frame files into: {self.settings.out_dir}")
                self.status.emit("Filename: adc_data_000001.bin, adc_data_000002.bin, ...")

            # Open devices
            self._dev_spi = open_by_desc_or_index(self.settings.spi_desc, self.settings.spi_dev_index)
            self._dev_gpio = open_by_desc_or_index(self.settings.gpio_desc, self.settings.gpio_dev_index)

            # Set system clock FIRST (before SPI init)
            sysclk_hz = 60_000_000
            sysclk_name = "60MHz"
            if hasattr(ft4222, "SysClock"):
                # For 30MHz SPI: need 60MHz sysclock
                # For 24MHz SPI: need 48MHz sysclock
                target = self.settings.clock_hz
                if target >= 30_000_000:
                    sysclk_enum = ft4222.SysClock.CLK_60
                    sysclk_hz = 60_000_000
                    sysclk_name = "60MHz"
                elif target >= 24_000_000:
                    sysclk_enum = ft4222.SysClock.CLK_48
                    sysclk_hz = 48_000_000
                    sysclk_name = "48MHz"
                elif target >= 20_000_000:
                    sysclk_enum = ft4222.SysClock.CLK_80
                    sysclk_hz = 80_000_000
                    sysclk_name = "80MHz"
                else:
                    sysclk_enum = ft4222.SysClock.CLK_60
                    sysclk_hz = 60_000_000
                    sysclk_name = "60MHz"

                try:
                    self._dev_spi.setClock(sysclk_enum)
                    self.status.emit(f"System clock set to {sysclk_name}")
                except Exception as e:
                    self.status.emit(f"Warning: setClock failed: {e}")

            # GPIO init
            self._dev_gpio.gpio_Init(gpio0=Dir.INPUT, gpio1=Dir.INPUT, gpio2=Dir.INPUT, gpio3=Dir.INPUT)
            intr_port = port_enum(self.settings.host_intr_port)

            # SPI init - calculate divider based on sysclk
            cpol, cpha = spi_mode_to_cpol_cpha(self.settings.spi_mode)

            # Find best divider for target SPI clock
            dividers = [
                (Clock.DIV_2,   sysclk_hz // 2),
                (Clock.DIV_4,   sysclk_hz // 4),
                (Clock.DIV_8,   sysclk_hz // 8),
                (Clock.DIV_16,  sysclk_hz // 16),
                (Clock.DIV_32,  sysclk_hz // 32),
                (Clock.DIV_64,  sysclk_hz // 64),
                (Clock.DIV_128, sysclk_hz // 128),
                (Clock.DIV_256, sysclk_hz // 256),
                (Clock.DIV_512, sysclk_hz // 512),
            ]
            clk_div = dividers[-1][0]
            actual_spi_hz = dividers[-1][1]
            for div_enum, hz in dividers:
                if hz <= self.settings.clock_hz:
                    clk_div = div_enum
                    actual_spi_hz = hz
                    break

            spi_master_init_compat(self._dev_spi, clk_div, cpol, cpha, ss_enum(self.settings.ss))

            if actual_spi_hz != self.settings.clock_hz:
                self.status.emit(f"SPI Clock: requested {self.settings.clock_hz/1e6:.1f} MHz -> actual {actual_spi_hz/1e6:.1f} MHz")
            else:
                self.status.emit(f"SPI Clock: {actual_spi_hz/1e6:.1f} MHz")

            self.status.emit("Waiting for sensor streaming...")

            frame_count = 0
            total_bytes = 0
            start_t = time.perf_counter()

            target_frames = self.settings.num_frames if self.settings.num_frames > 0 else float("inf")
            total_for_progress = int(target_frames) if target_frames != float("inf") else 0

            # --- resync on start ---
            pending_chunk0 = None
            if self.settings.resync_on_start:
                self.status.emit("Syncing to frame boundary (looking for chunk0 header)...")
                while self._running:
                    ok = wait_intr_ready(self._dev_gpio, intr_port, self.settings.host_intr_active_low,
                                         self.settings.timeout_s, self.settings.poll_sleep_us)
                    if not ok:
                        self.timeouts += 1
                        continue

                    if self.settings.post_intr_delay_us > 0:
                        time.sleep(self.settings.post_intr_delay_us / 1_000_000.0)

                    raw = spi_read_keep_cs(self._dev_spi, CHUNK_SIZE)
                    if self.settings.byteswap32:
                        raw = byteswap32_fast(raw)

                    if looks_like_frame_start(raw, self.settings.byteswap32):
                        pending_chunk0 = raw
                        break

                    self.resync_discards += 1
                    if self.resync_discards % 5 == 0:
                        b0 = " ".join(f"{x:02X}" for x in raw[:8])
                        self.status.emit(f"Resync: discarded {self.resync_discards} chunks (head={b0})")

                if pending_chunk0 is None and self._running:
                    raise RuntimeError("Resync failed: no frame boundary detected")

            # --- capture loop ---
            while self._running and frame_count < target_frames:
                # chunk0
                if pending_chunk0 is not None:
                    chunk0 = pending_chunk0
                    pending_chunk0 = None
                else:
                    ok = wait_intr_ready(self._dev_gpio, intr_port, self.settings.host_intr_active_low,
                                         self.settings.timeout_s, self.settings.poll_sleep_us)
                    if not ok:
                        self.timeouts += 1
                        continue

                    if self.settings.post_intr_delay_us > 0:
                        time.sleep(self.settings.post_intr_delay_us / 1_000_000.0)

                    chunk0 = spi_read_keep_cs(self._dev_spi, CHUNK_SIZE)
                    if self.settings.byteswap32:
                        chunk0 = byteswap32_fast(chunk0)

                if not self.settings.resync_on_start and not looks_like_frame_start(chunk0, self.settings.byteswap32):
                    b0 = " ".join(f"{x:02X}" for x in chunk0[:8])
                    self.status.emit(f"Warning: chunk0 does not look like frame start (head={b0})")

                raw_u32 = int.from_bytes(chunk0[0:4], "big")
                frame_no = decode_fw_frame_no(raw_u32)
                first64 = chunk0[:64]

                # chunk1
                ok = wait_intr_ready(self._dev_gpio, intr_port, self.settings.host_intr_active_low,
                                     self.settings.timeout_s, self.settings.poll_sleep_us)
                if not ok:
                    self.timeouts += 1
                    continue

                if self.settings.post_intr_delay_us > 0:
                    time.sleep(self.settings.post_intr_delay_us / 1_000_000.0)

                chunk1 = spi_read_keep_cs(self._dev_spi, CHUNK_SIZE)
                if self.settings.byteswap32:
                    chunk1 = byteswap32_fast(chunk1)

                exp = chunk1_signature(self.settings.byteswap32)
                if chunk1[:4] != exp:
                    self.sig_mismatch += 1
                    self.status.emit(
                        f"Warning: chunk1 signature mismatch "
                        f"(count={self.sig_mismatch}) head={' '.join(f'{x:02X}' for x in chunk1[:8])}"
                    )

                # assemble frame
                frame = chunk0 + chunk1

                # FrameNo continuity check
                if last_frame_no is not None:
                    expected = (last_frame_no + 1) & 0xFFFFFFFF
                    if frame_no != expected:
                        self.frame_gaps += 1
                        self.status.emit(f"Warning: FrameNo jump: last={last_frame_no} -> now={frame_no} (expected {expected})")
                last_frame_no = frame_no

                # save per-frame file
                frame_count += 1
                if self.settings.save_to_file:
                    out_path = make_frame_path(self.settings.out_dir, frame_count)
                    with open(out_path, "wb") as f:
                        f.write(frame)

                total_bytes += FW_FRAME_SIZE

                self.progress.emit(frame_count, total_for_progress)

                if self.settings.preview_every > 0 and (frame_count % self.settings.preview_every) == 0:
                    self.preview.emit(first64)

                if self.settings.log_every > 0 and (frame_count % self.settings.log_every) == 0:
                    elapsed = time.perf_counter() - start_t
                    mbps = (total_bytes / (1024 * 1024)) / elapsed if elapsed > 0 else 0.0
                    self.status.emit(
                        f"Frame {frame_count}: saved adc_data_{frame_count:06d}.bin | "
                        f"{mbps:.2f} MB/s | FrameNo={frame_no} | "
                        f"timeout={self.timeouts}, sig_mis={self.sig_mismatch}, gaps={self.frame_gaps}"
                    )

            elapsed = time.perf_counter() - start_t
            mbps = (total_bytes / (1024 * 1024)) / elapsed if elapsed > 0 else 0.0
            self.status.emit(
                f"Capture complete: frames={frame_count}, bytes={total_bytes}, speed={mbps:.2f} MB/s | "
                f"timeout={self.timeouts}, sig_mis={self.sig_mismatch}, gaps={self.frame_gaps}, resync_discard={self.resync_discards}"
            )

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
            if self._dev_spi and hasattr(self._dev_spi, "spiMaster_EndTransaction"):
                try:
                    self._dev_spi.spiMaster_EndTransaction()
                except Exception:
                    pass
        except Exception:
            pass
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
        self.status.emit("Devices closed")


# ======================== GUI ========================

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AWRL6844 SPI Data Capture Tool (FT4222H / per-frame files)")
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

        # Firmware info
        fw_group = QGroupBox("Firmware Frame Info")
        fw_layout = QHBoxLayout(fw_group)
        lbl = QLabel(f"Fixed Frame Size: {FW_FRAME_SIZE:,} bytes  (= {CHUNKS_PER_FRAME} x 64KB)")
        lbl.setStyleSheet("font-weight: bold;")
        fw_layout.addWidget(lbl)
        fw_layout.addStretch(1)
        root.addWidget(fw_group)

        # HOST_INTR
        intr_group = QGroupBox("HOST_INTR GPIO Configuration (FT4222H: Port-based)")
        intr_layout = QHBoxLayout(intr_group)

        self.intr_port_combo = QComboBox()
        self.intr_port_combo.addItems(["P0", "P1", "P2", "P3"])
        self.intr_port_combo.setCurrentIndex(3)  # NOTE: translated to English.

        self.cb_intr_active_low = QCheckBox("Active LOW (recommended)")
        self.cb_intr_active_low.setChecked(True)

        self.spin_timeout = QSpinBox()
        self.spin_timeout.setRange(1, 60)
        self.spin_timeout.setValue(5)

        self.spin_post_intr_us = QSpinBox()
        self.spin_post_intr_us.setRange(0, 5000)
        self.spin_post_intr_us.setValue(100)  # 100us default for 30MHz stability
        self.spin_post_intr_us.setToolTip("Delay after HOST_INTR before SPI read.\nIncrease if 30MHz is unstable (try 200-500us).")

        intr_layout.addWidget(QLabel("HOST_INTR Port:"))
        intr_layout.addWidget(self.intr_port_combo)
        intr_layout.addWidget(self.cb_intr_active_low)
        intr_layout.addWidget(QLabel("Wait timeout (s):"))
        intr_layout.addWidget(self.spin_timeout)
        intr_layout.addWidget(QLabel("Post-INTR delay (us):"))
        intr_layout.addWidget(self.spin_post_intr_us)
        intr_layout.addStretch(1)
        root.addWidget(intr_group)

        # Options
        opt_group = QGroupBox("Capture Options")
        opt_layout = QHBoxLayout(opt_group)

        self.clock_combo = QComboBox()
        self.clock_combo.addItems(["30000000", "24000000", "15000000", "12000000", "6000000", "1000000"])
        self.clock_combo.setCurrentText("24000000")  # FT4222H max stable: 24MHz (30MHz unstable)
        self.clock_combo.setToolTip("FT4222H: max stable 24MHz\nFT4232H: 30MHz with 3-Phase Clock")

        self.spi_mode_combo = QComboBox()
        self.spi_mode_combo.addItems(["0", "1", "2", "3"])
        self.spi_mode_combo.setCurrentText("0")

        self.ss_combo = QComboBox()
        self.ss_combo.addItems(["0", "1", "2", "3"])
        self.ss_combo.setCurrentText("0")

        self.spin_frames = QSpinBox()
        self.spin_frames.setRange(0, 1000000)
        self.spin_frames.setValue(100)
        self.spin_frames.setSpecialValueText("Infinite")

        self.spin_poll_us = QSpinBox()
        self.spin_poll_us.setRange(0, 5000)
        self.spin_poll_us.setValue(50)

        self.spin_preview = QSpinBox()
        self.spin_preview.setRange(0, 1000)
        self.spin_preview.setValue(1)

        self.spin_log = QSpinBox()
        self.spin_log.setRange(1, 1000)
        self.spin_log.setValue(1)

        self.cb_byteswap = QCheckBox("byteswap32 (ON recommended)")
        self.cb_byteswap.setChecked(True)

        self.cb_resync = QCheckBox("Resync on start (recommended)")
        self.cb_resync.setChecked(True)

        opt_layout.addWidget(QLabel("SPI Clock (Hz):"))
        opt_layout.addWidget(self.clock_combo)
        opt_layout.addWidget(QLabel("SPI Mode:"))
        opt_layout.addWidget(self.spi_mode_combo)
        opt_layout.addWidget(QLabel("SS:"))
        opt_layout.addWidget(self.ss_combo)
        opt_layout.addWidget(QLabel("Frames (0=inf):"))
        opt_layout.addWidget(self.spin_frames)
        opt_layout.addWidget(QLabel("GPIO poll (us):"))
        opt_layout.addWidget(self.spin_poll_us)
        opt_layout.addWidget(QLabel("Preview every N (0=off):"))
        opt_layout.addWidget(self.spin_preview)
        opt_layout.addWidget(QLabel("Log every N:"))
        opt_layout.addWidget(self.spin_log)
        opt_layout.addWidget(self.cb_byteswap)
        opt_layout.addWidget(self.cb_resync)
        opt_layout.addStretch(1)
        root.addWidget(opt_group)

        # Output folder + overwrite
        file_group = QGroupBox("Output Folder (per-frame files)")
        file_layout = QHBoxLayout(file_group)

        self.cb_save = QCheckBox("Save to folder")
        self.cb_save.setChecked(True)

        self.cb_overwrite = QCheckBox("Overwrite on start (delete adc_data_*.bin)")
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

        settings = CaptureSettings(
            spi_dev_index=int(spi_ud["index"]),
            gpio_dev_index=int(gpio_ud["index"]),
            spi_desc=str(spi_ud["desc"]),
            gpio_desc=str(gpio_ud["desc"]),
            clock_hz=int(self.clock_combo.currentText()),
            spi_mode=int(self.spi_mode_combo.currentText()),
            ss=int(self.ss_combo.currentText()),
            num_frames=self.spin_frames.value(),
            host_intr_port=self.intr_port_combo.currentIndex(),
            host_intr_active_low=self.cb_intr_active_low.isChecked(),
            timeout_s=float(self.spin_timeout.value()),
            poll_sleep_us=self.spin_poll_us.value(),
            post_intr_delay_us=self.spin_post_intr_us.value(),
            save_to_file=self.cb_save.isChecked(),
            out_dir=out_dir,
            overwrite_on_start=self.cb_overwrite.isChecked(),
            preview_every=self.spin_preview.value(),
            log_every=self.spin_log.value(),
            byteswap32=self.cb_byteswap.isChecked(),
            resync_on_start=self.cb_resync.isChecked(),
        )

        if settings.save_to_file and not settings.out_dir:
            QMessageBox.warning(self, "Warning", "Output folder is empty. Please select a folder.")
            return

        self._log("===== Capture Start =====")
        self._log(f"SPI dev         : desc='{settings.spi_desc}', index={settings.spi_dev_index}")
        self._log(f"GPIO dev        : desc='{settings.gpio_desc}', index={settings.gpio_dev_index}")
        self._log(f"SPI Clock       : {settings.clock_hz/1e6:.1f} MHz")
        self._log(f"SPI Mode/SS     : mode={settings.spi_mode}, SS{settings.ss}")
        self._log(f"HOST_INTR       : P{settings.host_intr_port}, active_low={settings.host_intr_active_low}, timeout={settings.timeout_s}s")
        self._log(f"Post-INTR delay : {settings.post_intr_delay_us} us")
        self._log(f"Frame Size      : {FW_FRAME_SIZE:,} bytes ({CHUNKS_PER_FRAME} chunks)")
        self._log(f"Byteswap32      : {'ON' if settings.byteswap32 else 'OFF'}")
        self._log(f"Resync          : {'ON' if settings.resync_on_start else 'OFF'}")
        self._log(f"Save            : {'ON' if settings.save_to_file else 'OFF'}")
        self._log(f"Overwrite       : {'ON' if settings.overwrite_on_start else 'OFF'}")
        self._log(f"Output folder   : {settings.out_dir if settings.save_to_file else '(not saving)'}")
        self._log("File naming     : adc_data_000001.bin, adc_data_000002.bin, ...")

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
        self.status_label.setText(msg)
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
