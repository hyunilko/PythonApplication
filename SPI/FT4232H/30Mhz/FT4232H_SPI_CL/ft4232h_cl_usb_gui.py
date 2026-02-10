#!/usr/bin/env python3
"""
file: ft4232h_cl_usb_gui.py
AWRL6844 SPI Data Capture Tool (FT4232H / ftd2xx / MPSSE)

Firmware (main_full_mss.c) behavior assumed:
- Each frame = adcDataPerFrame bytes (default 131072)
- Sent in chunks of 65536 bytes (MAXSPISIZEFTDI)
- For each chunk:
    HOST_INTR goes LOW  -> FW calls MCSPI_transfer(dataSize=32, count=chunk/4) -> HOST_INTR goes HIGH
- Frame header:
    byte[0..3] = u32_to_be(cnt)   (big-endian frame counter)
    byte[4.. ] = (i-4) & 0xFF     (test pattern)

Host behavior:
- Wait HOST_INTR LOW before EACH chunk, then SPI read exactly that chunk size
- Because FW transfers uint32_t* with dataSize=32 and MSB-first on SPI, byte order is reversed per 32-bit word
  -> byteswap32 on host reconstructs original byte stream (00 00 01 00 00 01 02 03 ...)

Fixes included:
1) SPI read uses spi_read_exact() to avoid partial-read misalignment
2) Resync-on-start (default ON): discards chunks until chunk0 is found, so PREVIEW always starts with frame counter
3) HOST_INTR uses user-selected mask ALWAYS (no device-type hardcode)
"""

import sys
import time
import os
import glob
import array
from datetime import datetime
from dataclasses import dataclass

try:
    import ftd2xx as ftd
except ImportError:
    print("ERROR: ftd2xx library not found. Install with: pip install ftd2xx")
    sys.exit(1)

from PyQt6.QtCore import QThread, pyqtSignal, QObject, pyqtSlot
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QPlainTextEdit, QLineEdit,
    QCheckBox, QMessageBox, QGroupBox, QFileDialog, QSpinBox,
    QProgressBar
)

# ======================== FTDI MPSSE Configuration ========================
# AN108: Command Processor for MPSSE and MCU Host Bus Emulation Modes

FTDI_CFG_60MHZ_SYSCLK = b"\x8A"         # Disable Clk Divide by 5 -> 60MHz
FTDI_CFG_NO_ADAPT_CLK = b"\x97"         # Turn Off Adaptive clocking
FTDI_CFG_3PHAS_CLK    = b"\x8C"         # Enable 3-Phase Data Clocking (more stable at high speed)
FTDI_CFG_NO_3PHAS_CLK = b"\x8D"         # Disable 3-Phase Data Clocking
FTDI_CFG_NO_LOOPBACK  = b"\x85"         # Disable loopback
FTDI_CFG_SPI_4PIN_CFG = b"\x80\x08\x0B" # Value=0x08, Dir=0x0B (SCK/MOSI/CS outputs as typical)

# MPSSE Commands
FTDI_CMD_CS_LOW     = b"\x80\x00\x0B"
FTDI_CMD_CS_HIGH    = b"\x80\x08\x0B"
FTDI_CMD_READ_BYTES = b"\x20"           # Clock Data Bytes In on +ve edge MSB first
FTDI_CMD_READ_GPIO  = b"\x81"           # Read Data bits LowByte

FTDI_MAX_CHUNK = 65536  # MAXSPISIZEFTDI


# ======================== Low-level helpers ========================

def set_clk(handle, hz: int) -> int:
    """Set SPI clock frequency from 60MHz system clock.

    FT4232H MPSSE divides 60MHz system clock:
        actual_clk = 60MHz / (2 * (div + 1))

    Only certain frequencies are achievable:
        div=0: 30MHz, div=1: 15MHz, div=2: 10MHz, div=4: 6MHz, etc.

    Returns the actual clock frequency set.
    """
    if hz > 30_000_000:
        raise ValueError("Max SCK rate is 30MHz")

    # Calculate divisor - round UP to get clock <= requested
    # div = ceil(60MHz / (2 * hz)) - 1
    div = max(0, (60_000_000 + 2 * hz - 1) // (2 * hz) - 1)

    # Calculate actual clock
    actual_hz = 60_000_000 // (2 * (div + 1))

    cmd = bytes((0x86, div & 0xFF, (div >> 8) & 0xFF))
    handle.write(cmd)

    return actual_hz


def set_device(handle, clk_speed: int = 15_000_000, latency_timer: int = 1, rw_timeout_ms: int = 5000,
                use_3phase_clk: bool = False) -> int:
    """Initialize FTDI device for MPSSE SPI + GPIO reads.

    Args:
        use_3phase_clk: Enable 3-phase clocking for more stable data sampling at high speeds.
                        Recommended for 30MHz operation.

    Returns the actual SPI clock frequency set.
    """
    handle.resetDevice()

    # Purge stale RX bytes
    try:
        rx_bytes, tx_bytes, event_status = handle.getStatus()
        if rx_bytes and rx_bytes > 0:
            handle.read(rx_bytes)
    except Exception:
        pass

    handle.setUSBParameters(65535, 65535)
    handle.setChars(False, 0, False, 0)
    handle.setTimeouts(rw_timeout_ms, rw_timeout_ms)
    handle.setLatencyTimer(latency_timer)

    handle.setBitMode(0, 0)  # Reset MPSSE
    handle.setBitMode(0, 2)  # Enable MPSSE
    time.sleep(0.050)

    handle.write(FTDI_CFG_60MHZ_SYSCLK)
    handle.write(FTDI_CFG_NO_ADAPT_CLK)

    # 3-phase clocking improves stability at high speeds (30MHz)
    if use_3phase_clk:
        handle.write(FTDI_CFG_3PHAS_CLK)
    else:
        handle.write(FTDI_CFG_NO_3PHAS_CLK)

    handle.write(FTDI_CFG_SPI_4PIN_CFG)
    actual_clk = set_clk(handle, clk_speed)
    time.sleep(0.010)
    handle.write(FTDI_CFG_NO_LOOPBACK)
    time.sleep(0.010)

    return actual_clk


def read_gpio(handle) -> int:
    """Read GPIO pins state (returns 8-bit value)."""
    handle.write(FTDI_CMD_READ_GPIO)
    res = handle.read(1)
    return int.from_bytes(res, "big")


def spi_read_exact(handle, length: int) -> bytes:
    """
    Read EXACTLY 'length' bytes via SPI.
    IMPORTANT: ftd2xx.read(length) can return fewer bytes; we must loop until full.
    """
    if length < 1 or length > 0x10000:
        raise ValueError("Length must be between 1 and 65536")

    # Length encoding: 0x0000 = 1 byte, 0x0001 = 2 bytes, ...
    len_bytes = int(length - 1).to_bytes(2, "little")
    cmd = FTDI_CMD_CS_LOW + FTDI_CMD_READ_BYTES + len_bytes + FTDI_CMD_CS_HIGH
    handle.write(cmd)

    out = bytearray()
    while len(out) < length:
        chunk = handle.read(length - len(out))
        if not chunk:
            raise TimeoutError("SPI read timeout/empty")
        out.extend(chunk)
    return bytes(out)


def byteswap32_fast(data: bytes) -> bytes:
    """Fast byteswap per 32-bit word using array('I').byteswap()."""
    if len(data) % 4 != 0:
        data = data + b"\x00" * (4 - (len(data) % 4))

    a = array.array("I")
    if a.itemsize != 4:
        # Fallback (rare)
        b = bytearray(data)
        for i in range(0, len(b), 4):
            b[i:i+4] = b[i:i+4][::-1]
        return bytes(b)

    a.frombytes(data)
    a.byteswap()
    return a.tobytes()


def intr_active_low(gpio_val: int, mask: int) -> bool:
    """HOST_INTR is considered 'ready' when (GPIO & mask) == 0."""
    return (gpio_val & mask) == 0


def wait_intr_low(dev_gpio, mask: int, timeout_s: float = 5.0, poll_sleep_us: int = 10, settle_us: int = 50) -> int:
    """Wait until HOST_INTR becomes active-low (GPIO&mask==0).

    At higher SPI speeds (30MHz+), the FW may not have data fully ready immediately
    when HOST_INTR goes LOW. settle_us adds a small delay after detecting LOW
    to ensure FW has prepared the data buffer.
    """
    t0 = time.perf_counter()
    last = 0
    sleep_s = max(poll_sleep_us, 0) / 1_000_000.0
    settle_s = max(settle_us, 0) / 1_000_000.0

    while True:
        last = read_gpio(dev_gpio)
        if intr_active_low(last, mask):
            # Add settle time for high-speed SPI (24MHz+) to ensure FW data is ready
            if settle_s > 0:
                time.sleep(settle_s)
            return last
        if (time.perf_counter() - t0) > timeout_s:
            raise TimeoutError(f"HOST_INTR LOW timeout (GPIO=0x{last:02X}, mask=0x{mask:02X})")
        if sleep_s > 0:
            time.sleep(sleep_s)


def list_ftdi_devices() -> list[tuple[int, str, str]]:
    """List available FTDI devices with (index, description, serial)."""
    devices = []
    try:
        n = None
        try:
            n = ftd.createDeviceInfoList()
        except Exception:
            devs = ftd.listDevices()
            if devs is None:
                n = 0
            elif isinstance(devs, (list, tuple)):
                n = len(devs)
            else:
                # sometimes an int-like
                try:
                    n = int(devs)
                except Exception:
                    n = 0

        for i in range(n or 0):
            try:
                dev = ftd.open(i)
                info = dev.getDeviceInfo()
                desc = info.get("description", b"Unknown").decode(errors="ignore")
                serial = info.get("serial", b"").decode(errors="ignore")
                devices.append((i, desc, serial))
                dev.close()
            except Exception:
                pass
    except Exception:
        pass
    return devices


# ======================== Frame boundary heuristics (test pattern) ========================

CHUNK1_HEAD = b"\xFC\xFD\xFE\xFF"  # for your test pattern: byte[i]=(i-4)&0xFF, chunk1 starts at i=65536

def looks_like_chunk0(chunk_byteswapped: bytes) -> bool:
    """
    For current test pattern:
    - chunk1 starts with FC FD FE FF (constant)
    - chunk0 starts with frame counter (u32_to_be(cnt)) and is NOT equal to CHUNK1_HEAD
    This is enough to fix the preview issue.
    """
    if len(chunk_byteswapped) < 4:
        return False
    return chunk_byteswapped[:4] != CHUNK1_HEAD


def looks_like_chunk1(chunk_byteswapped: bytes) -> bool:
    if len(chunk_byteswapped) < 4:
        return False
    return chunk_byteswapped[:4] == CHUNK1_HEAD


# ======================== Capture Worker ========================

@dataclass
class CaptureSettings:
    spi_dev_index: int
    gpio_dev_index: int
    clock_hz: int
    num_frames: int

    adc_samples: int
    chirps_per_burst: int
    bursts_per_frame: int
    rx_antennas: int

    host_intr_mask: int
    poll_sleep_us: int
    settle_us: int  # Settle time after HOST_INTR LOW detection (for 24MHz+)

    byteswap32: bool
    resync_on_start: bool
    use_3phase_clk: bool  # Enable 3-phase clocking for high-speed stability

    save_to_file: bool
    out_path: str

    flush_every: int
    preview_every: int
    log_every: int

    device_type: str  # just for display ("AOP"/"FCCSP")


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

    @pyqtSlot()
    def start(self):
        self._running = True

        try:
            frame_size = (self.settings.chirps_per_burst *
                          self.settings.bursts_per_frame *
                          self.settings.adc_samples *
                          self.settings.rx_antennas * 2)

            chunks_per_frame = (frame_size + FTDI_MAX_CHUNK - 1) // FTDI_MAX_CHUNK
            if chunks_per_frame < 1:
                chunks_per_frame = 1

            self.status.emit(f"Frame Size      : {frame_size:,} bytes")
            self.status.emit(f"Chunks/Frame    : {chunks_per_frame}")
            self.status.emit(f"HOST_INTR mask  : 0x{self.settings.host_intr_mask:02X}  (ready when GPIO&mask==0)")
            self.status.emit(f"Byteswap32      : {'ON' if self.settings.byteswap32 else 'OFF'}")
            self.status.emit(f"Resync          : {'ON' if self.settings.resync_on_start else 'OFF'}")
            self.status.emit(f"Settle time     : {self.settings.settle_us} us")

            # Open FTDI devices
            self._dev_spi = ftd.open(self.settings.spi_dev_index)
            self.status.emit(f"SPI device opened: index {self.settings.spi_dev_index}")
            actual_clk = set_device(self._dev_spi, self.settings.clock_hz, latency_timer=1, rw_timeout_ms=5000,
                                    use_3phase_clk=self.settings.use_3phase_clk)
            clk_msg = f"SPI configured: {actual_clk/1e6:.1f} MHz"
            if actual_clk != self.settings.clock_hz:
                clk_msg = f"SPI configured: requested {self.settings.clock_hz/1e6:.1f} MHz -> actual {actual_clk/1e6:.1f} MHz"
            if self.settings.use_3phase_clk:
                clk_msg += " (3-phase)"
            self.status.emit(clk_msg)

            # Separate GPIO handle (AOP typical), but user controls indices anyway
            if self.settings.gpio_dev_index != self.settings.spi_dev_index:
                self._dev_gpio = ftd.open(self.settings.gpio_dev_index)
                # GPIO reads still use MPSSE; clock irrelevant, keep low
                set_device(self._dev_gpio, 1_000_000, latency_timer=1, rw_timeout_ms=5000)
                self.status.emit(f"GPIO device opened: index {self.settings.gpio_dev_index} (separate)")
            else:
                self._dev_gpio = self._dev_spi
                self.status.emit("GPIO device: same as SPI device")

            # Output folder (1 file per frame)
            out_folder = None
            if self.settings.save_to_file and self.settings.out_path:
                out_folder = self.settings.out_path
                os.makedirs(out_folder, exist_ok=True)

                # Remove existing .bin files
                old_files = glob.glob(os.path.join(out_folder, "*.bin"))
                if old_files:
                    removed = 0
                    failed = 0
                    for f in old_files:
                        try:
                            os.remove(f)
                            removed += 1
                        except Exception as e:
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

            target_frames = self.settings.num_frames if self.settings.num_frames > 0 else float("inf")
            total_for_progress = int(target_frames) if target_frames != float("inf") else 0

            frame_count = 0
            total_bytes = 0
            start_t = time.perf_counter()

            # ---------- RESYNC ON START ----------
            pending_chunks: list[bytes] = []
            if self.settings.resync_on_start:
                self.status.emit("Syncing to frame boundary (looking for chunk0 header)...")
                discards = 0

                # We validate by also reading the next chunk (if frame has >=2 chunks)
                # to reduce the chance we start at chunk1.
                while self._running:
                    # make stop responsive: poll with short timeouts
                    try:
                        wait_intr_low(self._dev_gpio, self.settings.host_intr_mask, timeout_s=0.25, poll_sleep_us=self.settings.poll_sleep_us, settle_us=self.settings.settle_us)
                    except TimeoutError:
                        continue

                    raw0 = spi_read_exact(self._dev_spi, min(FTDI_MAX_CHUNK, frame_size))
                    if self.settings.byteswap32:
                        raw0 = byteswap32_fast(raw0)

                    if not looks_like_chunk0(raw0):
                        discards += 1
                        if discards % 5 == 0:
                            head8 = " ".join(f"{b:02X}" for b in raw0[:8])
                            self.status.emit(f"Resync: discarded {discards} chunks (head={head8})")
                        continue

                    # If only 1 chunk per frame, accept immediately
                    if chunks_per_frame == 1:
                        pending_chunks = [raw0]
                        break

                    # For 2+ chunks, read the next chunk and confirm it's chunk1 head (test pattern)
                    try:
                        wait_intr_low(self._dev_gpio, self.settings.host_intr_mask, timeout_s=0.25, poll_sleep_us=self.settings.poll_sleep_us, settle_us=self.settings.settle_us)
                    except TimeoutError:
                        # couldn't get next chunk now; discard and continue
                        discards += 1
                        continue

                    raw1 = spi_read_exact(self._dev_spi, min(FTDI_MAX_CHUNK, frame_size - len(raw0)))
                    if self.settings.byteswap32:
                        raw1 = byteswap32_fast(raw1)

                    if looks_like_chunk1(raw1):
                        pending_chunks = [raw0, raw1]
                        break

                    # Not chunk1 -> we likely didn't align; discard both and keep searching
                    discards += 1
                    if discards % 5 == 0:
                        head8 = " ".join(f"{b:02X}" for b in raw0[:8])
                        self.status.emit(f"Resync: discarded {discards} chunks (head={head8})")

                if not pending_chunks and self._running:
                    raise RuntimeError("Resync failed: no frame boundary detected")

            # ---------- CAPTURE LOOP ----------
            while self._running and frame_count < target_frames:
                # Read full frame by chunks (and preview always from chunk0)
                frame_data = bytearray()
                first64 = None

                # Consume any pending chunks from resync (only for the very first frame)
                chunks_to_use = pending_chunks
                pending_chunks = []

                # Determine per-frame remaining sizes
                remain = frame_size

                # --- chunk 0 ---
                if chunks_to_use:
                    chunk0 = chunks_to_use.pop(0)
                else:
                    # wait for next chunk ready
                    while self._running:
                        try:
                            wait_intr_low(self._dev_gpio, self.settings.host_intr_mask, timeout_s=0.25, poll_sleep_us=self.settings.poll_sleep_us, settle_us=self.settings.settle_us)
                            break
                        except TimeoutError:
                            continue
                    if not self._running:
                        break

                    chunk0 = spi_read_exact(self._dev_spi, min(FTDI_MAX_CHUNK, remain))
                    if self.settings.byteswap32:
                        chunk0 = byteswap32_fast(chunk0)

                remain -= len(chunk0)
                first64 = chunk0[:64]
                frame_data.extend(chunk0)

                # --- remaining chunks ---
                while self._running and remain > 0:
                    if chunks_to_use:
                        chunk = chunks_to_use.pop(0)
                    else:
                        while self._running:
                            try:
                                wait_intr_low(self._dev_gpio, self.settings.host_intr_mask, timeout_s=0.25, poll_sleep_us=self.settings.poll_sleep_us, settle_us=self.settings.settle_us)
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
                    break

                # Save frame to individual file
                if out_folder:
                    frame_file = os.path.join(out_folder, f"frame_{frame_count:04d}.bin")
                    with open(frame_file, "wb") as f:
                        f.write(frame_data)

                frame_count += 1
                total_bytes += len(frame_data)

                # Preview control
                if self.settings.preview_every > 0 and (frame_count % self.settings.preview_every) == 0 and first64 is not None:
                    self.preview.emit(first64)

                # Progress
                self.progress.emit(frame_count, total_for_progress)

                # Log control
                if self.settings.log_every > 0 and (frame_count % self.settings.log_every) == 0:
                    elapsed = time.perf_counter() - start_t
                    mbps = (total_bytes / (1024 * 1024)) / elapsed if elapsed > 0 else 0.0
                    self.status.emit(
                        f"Frame {frame_count}: {frame_size} bytes | Total: {total_bytes/1024:.1f} KB | {mbps:.2f} MB/s"
                    )

            elapsed = time.perf_counter() - start_t
            mbps = (total_bytes / (1024 * 1024)) / elapsed if elapsed > 0 else 0.0
            self.status.emit(f"Capture complete: {frame_count} frames, {total_bytes} bytes, {mbps:.2f} MB/s")

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
        self.status.emit("Devices closed")


# ======================== GUI ========================

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AWRL6844 SPI Data Capture Tool (FT4232H / u32_to_be(cnt) / resync+exact-read)")
        self.resize(1050, 820)

        self.capture_thread: QThread | None = None
        self.capture_worker: SpiCaptureWorker | None = None

        self._build_ui()
        self._refresh_devices()

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # ---- Device Selection ----
        dev_group = QGroupBox("FTDI Device Selection")
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
        self.spin_frames.setValue(100)
        self.spin_frames.setSpecialValueText("Infinite")

        self.clock_combo = QComboBox()
        # FT4232H MPSSE: actual_clk = 60MHz / (2 * (div + 1))
        # Only these exact frequencies are achievable:
        self.clock_combo.addItems([
            "30000000",  # div=0: 30MHz (max)
            "24000000",  # div=1: 15MHz            
            "15000000",  # div=1: 24MHz
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
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_view.appendPlainText(f"[{ts}] {msg}")
        c = self.log_view.textCursor()
        c.movePosition(c.MoveOperation.End)
        self.log_view.setTextCursor(c)

    def _refresh_devices(self):
        devices = list_ftdi_devices()
        self.spi_combo.clear()
        self.gpio_combo.clear()

        for idx, desc, serial in devices:
            label = f"[{idx}] {desc} (SN: {serial})"
            self.spi_combo.addItem(label, idx)
            self.gpio_combo.addItem(label, idx)

        # default: SPI=0, GPIO=1 if available
        if len(devices) >= 2:
            self.spi_combo.setCurrentIndex(0)
            self.gpio_combo.setCurrentIndex(1)
        elif len(devices) == 1:
            self.spi_combo.setCurrentIndex(0)
            self.gpio_combo.setCurrentIndex(0)

        self._log(f"Found {len(devices)} FTDI device(s)")
        for idx, desc, serial in devices:
            self._log(f"  [{idx}] {desc} (SN: {serial})")

        if not devices:
            QMessageBox.warning(
                self, "Warning",
                "No FTDI devices found.\n\n"
                "Check:\n"
                "1) FT4232H cable connected\n"
                "2) FTDI D2XX drivers installed\n"
                "3) Device shows in Device Manager"
            )

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
        if self.spi_combo.count() == 0:
            QMessageBox.warning(self, "Warning", "No FTDI devices found!")
            return

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

        # UI state
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)

        if settings.num_frames > 0:
            self.progress.setMaximum(settings.num_frames)
            self.progress.setValue(0)
        else:
            self.progress.setMaximum(0)  # indeterminate
            self.progress.setValue(0)

        # Thread + worker
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
            # indeterminate mode; show something moving
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
