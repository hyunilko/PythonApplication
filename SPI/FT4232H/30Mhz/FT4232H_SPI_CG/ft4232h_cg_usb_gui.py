#!/usr/bin/env python3
"""
file: ft4232h_cg_usb_gui.py  (FW header = u32_to_be(cnt))  [SAVE PER FRAME + CNT IN FILENAME]

Firmware:
- adcDataPerFrame = 131072 bytes
- 2 chunks x 65536 (MAXSPISIZEFTDI)
- Frame header:
    byte[0..3] = u32_to_be(cnt)
    byte[4..]  = (i-4) & 0xFF sequence

Host:
- Wait HOST_INTR LOW before each chunk, then SPI read exactly 65536 bytes
- Apply byteswap32 to reconstruct original byte stream as in adcbuffer

Stability/Robust:
- spi_read_exact(): loop until FULL length received (ftd2xx read can be partial)
- SEND_IMMEDIATE (0x87) after GPIO/SPI read commands
- settle_us after HOST_INTR LOW
- set_clk(): ceil divisor so actual <= requested
- optional 3-phase clock

Save:
- Save per frame: frame_0000_cnt_7C7D7E7F.bin
- Default folder: Z:\\Texas_Instruments\\AWRL6844\\Python_App\\capture_out
- On start: remove existing *.bin in folder (always)
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

# ---------------- Firmware-fixed sizes ----------------
FW_FRAME_SIZE = 131072
CHUNK_SIZE = 65536
CHUNKS_PER_FRAME = FW_FRAME_SIZE // CHUNK_SIZE

# ======================== FTDI MPSSE Configuration ========================
FTDI_CFG_60MHZ_SYSCLK = b'\x8A'
FTDI_CFG_NO_ADAPT_CLK = b'\x97'
FTDI_CFG_3PHAS_CLK    = b'\x8C'
FTDI_CFG_NO_3PHAS_CLK = b'\x8D'
FTDI_CFG_NO_LOOPBACK  = b'\x85'
FTDI_CFG_SPI_4PIN_CFG = b'\x80\x08\x0B'

FTDI_CMD_CS_LOW       = b'\x80\x00\x0B'
FTDI_CMD_CS_HIGH      = b'\x80\x08\x0B'
FTDI_CMD_READ_BYTES   = b'\x20'
FTDI_CMD_READ_GPIO    = b'\x81'
FTDI_CMD_SEND_IMM     = b'\x87'   # flush response immediately


# ======================== Low-level helpers ========================

def set_clk(handle, hz_req: int) -> int:
    """
    actual_clk = 60MHz / (2*(div+1))
    div = ceil(60MHz/(2*hz_req)) - 1  -> actual <= requested
    """
    if hz_req > 30_000_000:
        raise ValueError("Max SCK rate is 30MHz")
    if hz_req <= 0:
        raise ValueError("Invalid SCK rate")

    div = max(0, (60_000_000 + 2 * hz_req - 1) // (2 * hz_req) - 1)
    actual = 60_000_000 // (2 * (div + 1))
    cmd = bytes((0x86, div & 0xFF, (div >> 8) & 0xFF))
    handle.write(cmd)
    return actual


def set_device(handle, clk_speed_req: int = 15_000_000, latency_timer: int = 1,
               rw_timeout_ms: int = 5000, use_3phase_clk: bool = False) -> int:
    handle.resetDevice()

    # Purge stale RX data
    try:
        rx_bytes, _, _ = handle.getStatus()
        if rx_bytes > 0:
            handle.read(rx_bytes)
    except Exception:
        pass

    handle.setUSBParameters(65535, 65535)
    handle.setChars(False, 0, False, 0)
    handle.setTimeouts(rw_timeout_ms, rw_timeout_ms)
    handle.setLatencyTimer(latency_timer)
    handle.setBitMode(0, 0)
    handle.setBitMode(0, 2)
    time.sleep(0.050)

    handle.write(FTDI_CFG_60MHZ_SYSCLK)
    handle.write(FTDI_CFG_NO_ADAPT_CLK)

    if use_3phase_clk:
        handle.write(FTDI_CFG_3PHAS_CLK)
    else:
        handle.write(FTDI_CFG_NO_3PHAS_CLK)

    handle.write(FTDI_CFG_SPI_4PIN_CFG)

    actual = set_clk(handle, clk_speed_req)
    time.sleep(0.010)

    handle.write(FTDI_CFG_NO_LOOPBACK)
    time.sleep(0.010)

    return actual


def read_gpio(handle) -> int:
    handle.write(FTDI_CMD_READ_GPIO + FTDI_CMD_SEND_IMM)
    b = handle.read(1)
    if not b:
        raise TimeoutError("GPIO read timeout (empty)")
    return int.from_bytes(b, "big")


def spi_read_exact(handle, length: int) -> bytes:
    if length < 1 or length > 0x10000:
        raise ValueError("Length must be 1..65536")

    len_bytes = int(length - 1).to_bytes(2, "little")

    # SEND_IMMEDIATE to flush response
    cmd = FTDI_CMD_CS_LOW + FTDI_CMD_READ_BYTES + len_bytes + FTDI_CMD_CS_HIGH + FTDI_CMD_SEND_IMM
    handle.write(cmd)

    out = bytearray()
    while len(out) < length:
        chunk = handle.read(length - len(out))
        if not chunk:
            raise TimeoutError("SPI read returned empty (timeout)")
        out.extend(chunk)
    return bytes(out)


def byteswap32_fast(data: bytes) -> bytes:
    if len(data) % 4 != 0:
        data = data + b"\x00" * (4 - (len(data) % 4))

    a = array.array("I")
    if a.itemsize != 4:
        b = bytearray(data)
        for i in range(0, len(b), 4):
            b[i:i+4] = b[i:i+4][::-1]
        return bytes(b)

    a.frombytes(data)
    a.byteswap()
    return a.tobytes()


def list_ftdi_devices() -> list[tuple[int, str, str]]:
    devices: list[tuple[int, str, str]] = []
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


def intr_active_low(gpio_val: int, mask: int) -> bool:
    return (gpio_val & mask) == 0


def wait_intr_low(dev_gpio, mask: int, timeout_s: float, poll_sleep_us: int, settle_us: int) -> int:
    t0 = time.perf_counter()
    last = 0
    sleep_s = max(poll_sleep_us, 0) / 1_000_000.0
    settle_s = max(settle_us, 0) / 1_000_000.0

    while True:
        last = read_gpio(dev_gpio)
        if intr_active_low(last, mask):
            if settle_s > 0:
                time.sleep(settle_s)
            return last
        if (time.perf_counter() - t0) > timeout_s:
            raise TimeoutError(f"HOST_INTR LOW timeout (GPIO=0x{last:02X}, mask=0x{mask:02X})")
        if sleep_s > 0:
            time.sleep(sleep_s)


def decode_fw_frame_no(u32_be: int) -> int:
    return int(u32_be)


def chunk1_signature(byteswapped: bool) -> bytes:
    return b"\xFC\xFD\xFE\xFF" if byteswapped else b"\xFF\xFE\xFD\xFC"


def looks_like_frame_start(chunk: bytes, byteswapped: bool) -> bool:
    if len(chunk) < 16:
        return False
    if chunk[:4] == chunk1_signature(byteswapped):
        return False
    return chunk[4:16] == bytes(range(0x00, 0x0C))


def clear_bin_files(folder: str) -> int:
    """Delete existing *.bin in folder. Returns removed count."""
    removed = 0
    for fpath in glob.glob(os.path.join(folder, "*.bin")):
        try:
            os.remove(fpath)
            removed += 1
        except Exception:
            pass
    return removed


# ======================== Capture Worker ========================

@dataclass
class CaptureSettings:
    spi_dev_index: int
    gpio_dev_index: int
    clock_hz: int
    use_3phase_clk: bool
    num_frames: int
    host_intr_mask: int

    save_to_folder: bool
    out_folder: str

    device_type: str
    poll_sleep_us: int
    settle_us: int

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

    @pyqtSlot()
    def start(self):
        self._running = True

        try:
            self.status.emit(f"Frame size: {FW_FRAME_SIZE:,} bytes ({CHUNKS_PER_FRAME} x 64KB)")
            self.status.emit(f"HOST_INTR mask: 0x{self.settings.host_intr_mask:02X} (ready when GPIO&mask==0)")
            self.status.emit(f"byteswap32: {'ON' if self.settings.byteswap32 else 'OFF'}")
            self.status.emit(f"Resync on start: {'ON' if self.settings.resync_on_start else 'OFF'}")
            self.status.emit(f"poll_sleep_us: {self.settings.poll_sleep_us} us, settle_us: {self.settings.settle_us} us")
            self.status.emit(f"3-phase clock: {'ON' if self.settings.use_3phase_clk else 'OFF'}")

            # Open SPI device
            self._dev_spi = ftd.open(self.settings.spi_dev_index)
            self.status.emit(f"SPI device opened: index {self.settings.spi_dev_index}")
            actual_clk = set_device(
                self._dev_spi,
                self.settings.clock_hz,
                latency_timer=1,
                rw_timeout_ms=5000,
                use_3phase_clk=self.settings.use_3phase_clk
            )
            if actual_clk != self.settings.clock_hz:
                self.status.emit(
                    f"SPI configured: requested {self.settings.clock_hz/1e6:.1f} MHz -> actual {actual_clk/1e6:.1f} MHz"
                )
            else:
                self.status.emit(f"SPI configured: {actual_clk/1e6:.1f} MHz")

            # Open GPIO device
            if self.settings.device_type == "AOP" and self.settings.gpio_dev_index != self.settings.spi_dev_index:
                self._dev_gpio = ftd.open(self.settings.gpio_dev_index)
                set_device(self._dev_gpio, 1_000_000, latency_timer=1, rw_timeout_ms=5000, use_3phase_clk=False)
                self.status.emit(f"GPIO device opened: index {self.settings.gpio_dev_index} (separate)")
            else:
                self._dev_gpio = self._dev_spi
                self.status.emit("GPIO device: same as SPI device")

            # Output folder (per frame)
            out_folder = None
            if self.settings.save_to_folder:
                out_folder = self.settings.out_folder.strip()
                if not out_folder:
                    raise ValueError("Output folder path is empty")
                os.makedirs(out_folder, exist_ok=True)

                # ALWAYS clear existing *.bin on start (as requested)
                removed = clear_bin_files(out_folder)
                self.status.emit(f"Cleared existing *.bin in output folder: removed {removed} file(s)")

                self.status.emit(f"Saving per frame to folder: {out_folder} (frame_XXXX_cnt_YYYYYYYY.bin)")
            else:
                self.status.emit("Output: (not saving)")

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
                discards = 0
                while self._running:
                    wait_intr_low(self._dev_gpio, self.settings.host_intr_mask, 5.0,
                                 self.settings.poll_sleep_us, self.settings.settle_us)
                    raw = spi_read_exact(self._dev_spi, CHUNK_SIZE)
                    if self.settings.byteswap32:
                        raw = byteswap32_fast(raw)

                    if looks_like_frame_start(raw, self.settings.byteswap32):
                        pending_chunk0 = raw
                        break

                    discards += 1
                    if discards % 5 == 0:
                        b0 = " ".join(f"{x:02X}" for x in raw[:8])
                        self.status.emit(f"Resync: discarded {discards} chunks (head={b0})")

                if pending_chunk0 is None and self._running:
                    raise RuntimeError("Resync failed: no frame boundary detected")

            # --- capture loop ---
            while self._running and frame_count < target_frames:
                # chunk0
                if pending_chunk0 is not None:
                    chunk0 = pending_chunk0
                    pending_chunk0 = None
                else:
                    wait_intr_low(self._dev_gpio, self.settings.host_intr_mask, 5.0,
                                 self.settings.poll_sleep_us, self.settings.settle_us)
                    chunk0 = spi_read_exact(self._dev_spi, CHUNK_SIZE)
                    if self.settings.byteswap32:
                        chunk0 = byteswap32_fast(chunk0)

                # if resync enabled, ensure chunk0 is really frame start
                if self.settings.resync_on_start and not looks_like_frame_start(chunk0, self.settings.byteswap32):
                    b0 = " ".join(f"{x:02X}" for x in chunk0[:8])
                    self.status.emit(f"Lost alignment, resyncing... (head={b0})")
                    discards = 0
                    while self._running:
                        wait_intr_low(self._dev_gpio, self.settings.host_intr_mask, 5.0,
                                     self.settings.poll_sleep_us, self.settings.settle_us)
                        raw = spi_read_exact(self._dev_spi, CHUNK_SIZE)
                        if self.settings.byteswap32:
                            raw = byteswap32_fast(raw)
                        if looks_like_frame_start(raw, self.settings.byteswap32):
                            pending_chunk0 = raw
                            break
                        discards += 1
                        if discards % 5 == 0:
                            b1 = " ".join(f"{x:02X}" for x in raw[:8])
                            self.status.emit(f"Resync: discarded {discards} chunks (head={b1})")
                    if pending_chunk0 is None:
                        break
                    continue

                # frame counter
                raw_u32 = int.from_bytes(chunk0[0:4], "big")
                frame_no = decode_fw_frame_no(raw_u32)
                first64 = chunk0[:64]

                # chunk1
                wait_intr_low(self._dev_gpio, self.settings.host_intr_mask, 5.0,
                             self.settings.poll_sleep_us, self.settings.settle_us)
                chunk1 = spi_read_exact(self._dev_spi, CHUNK_SIZE)
                if self.settings.byteswap32:
                    chunk1 = byteswap32_fast(chunk1)

                exp = chunk1_signature(self.settings.byteswap32)
                if chunk1[:4] != exp:
                    self.status.emit(
                        f"Warning: chunk1 head unexpected: "
                        f"{' '.join(f'{x:02X}' for x in chunk1[:8])} (exp {exp.hex().upper()})"
                    )

                # ---- SAVE PER FRAME with CNT ----
                if out_folder is not None:
                    # e.g. frame_0000_cnt_7C7D7E7F.bin
                    fname = f"frame_{frame_count:04d}_cnt_{frame_no:08X}.bin"
                    frame_file = os.path.join(out_folder, fname)
                    with open(frame_file, "wb") as f:
                        f.write(chunk0)
                        f.write(chunk1)

                frame_count += 1
                total_bytes += FW_FRAME_SIZE

                self.progress.emit(frame_count, total_for_progress)

                if self.settings.preview_every > 0 and (frame_count % self.settings.preview_every) == 0:
                    self.preview.emit(first64)

                if self.settings.log_every > 0 and (frame_count % self.settings.log_every) == 0:
                    elapsed = time.perf_counter() - start_t
                    mbps = (total_bytes / (1024 * 1024)) / elapsed if elapsed > 0 else 0.0
                    self.status.emit(
                        f"Frame {frame_count}: {FW_FRAME_SIZE} bytes | Total: {total_bytes/1024:.1f} KB | "
                        f"{mbps:.2f} MB/s | cnt=0x{frame_no:08X}"
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
        self.setWindowTitle("AWRL6844 SPI Data Capture Tool (Save per frame + cnt in filename)")
        self.resize(1080, 880)

        self.capture_thread: QThread | None = None
        self.capture_worker: SpiCaptureWorker | None = None

        self._build_ui()
        self._refresh_devices()

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # Device selection
        dev_group = QGroupBox("FTDI Device Selection")
        dev_layout = QHBoxLayout(dev_group)

        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self._refresh_devices)

        self.device_type_combo = QComboBox()
        self.device_type_combo.addItems(["AOP", "FCCSP"])
        self.device_type_combo.setCurrentText("AOP")

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

        # Firmware info
        fw_group = QGroupBox("Firmware Frame Info")
        fw_layout = QHBoxLayout(fw_group)
        lbl = QLabel(f"Fixed Frame Size: {FW_FRAME_SIZE:,} bytes  (= {CHUNKS_PER_FRAME} x 64KB)")
        lbl.setStyleSheet("font-weight: bold;")
        fw_layout.addWidget(lbl)
        fw_layout.addStretch(1)
        root.addWidget(fw_group)

        # HOST_INTR
        intr_group = QGroupBox("HOST_INTR GPIO Configuration")
        intr_layout = QHBoxLayout(intr_group)

        self.intr_bit_combo = QComboBox()
        self.intr_bit_combo.addItems([
            "Bit 4 (0x10)  (FCCSP typical)",
            "Bit 5 (0x20)  (AOP option)",
            "Bit 7 (0x80)  (AOP option)",
            "Bits 5+7 (0xA0) (legacy)"
        ])
        self.intr_bit_combo.setCurrentIndex(3)  # 0xA0 default

        intr_layout.addWidget(QLabel("HOST_INTR mask:"))
        intr_layout.addWidget(self.intr_bit_combo, 2)
        intr_layout.addStretch(1)
        root.addWidget(intr_group)

        # Options
        opt_group = QGroupBox("Capture Options")
        opt_layout = QHBoxLayout(opt_group)

        self.clock_combo = QComboBox()
        self.clock_combo.addItems(["30000000", "24000000", "15000000", "12000000", "10000000", "6000000", "1000000"])
        self.clock_combo.setCurrentText("24000000")

        self.cb_3phase = QCheckBox("3-Phase Clock (helps high-speed stability)")
        self.cb_3phase.setChecked(True)

        self.spin_frames = QSpinBox()
        self.spin_frames.setRange(0, 1_000_000)
        self.spin_frames.setValue(100)
        self.spin_frames.setSpecialValueText("Infinite")

        self.spin_poll_us = QSpinBox()
        self.spin_poll_us.setRange(0, 5000)
        self.spin_poll_us.setValue(10)

        self.spin_settle_us = QSpinBox()
        self.spin_settle_us.setRange(0, 5000)
        self.spin_settle_us.setValue(100)

        self.spin_preview = QSpinBox()
        self.spin_preview.setRange(0, 1000)
        self.spin_preview.setValue(1)

        self.spin_log = QSpinBox()
        self.spin_log.setRange(1, 1000)
        self.spin_log.setValue(1)

        self.cb_byteswap = QCheckBox("byteswap32 (ON recommended)")
        self.cb_byteswap.setChecked(True)

        self.cb_resync = QCheckBox("Resync on start (recommended for Stop/Start)")
        self.cb_resync.setChecked(True)

        opt_layout.addWidget(QLabel("SPI Clock req (Hz):"))
        opt_layout.addWidget(self.clock_combo)
        opt_layout.addWidget(self.cb_3phase)
        opt_layout.addWidget(QLabel("Frames (0=inf):"))
        opt_layout.addWidget(self.spin_frames)
        opt_layout.addWidget(QLabel("GPIO poll (us):"))
        opt_layout.addWidget(self.spin_poll_us)
        opt_layout.addWidget(QLabel("Settle (us):"))
        opt_layout.addWidget(self.spin_settle_us)
        opt_layout.addWidget(QLabel("Preview every N:"))
        opt_layout.addWidget(self.spin_preview)
        opt_layout.addWidget(QLabel("Log every N:"))
        opt_layout.addWidget(self.spin_log)
        opt_layout.addWidget(self.cb_byteswap)
        opt_layout.addWidget(self.cb_resync)
        opt_layout.addStretch(1)
        root.addWidget(opt_group)

        # Output folder (per frame)
        out_group = QGroupBox("Output Folder (1 file per frame: frame_0000_cnt_XXXXXXXX.bin)")
        out_layout = QHBoxLayout(out_group)

        self.cb_save = QCheckBox("Save frames")
        self.cb_save.setChecked(True)

        # Always delete existing files on start (requested)
        self.cb_clear_old = QCheckBox("Clear existing *.bin on start (forced)")
        self.cb_clear_old.setChecked(True)
        self.cb_clear_old.setEnabled(False)

        self.out_folder = QLineEdit()
        self.out_folder.setText(r"Z:\Texas_Instruments\AWRL6844\Python_App\capture_out")

        self.btn_browse = QPushButton("Browse...")
        self.btn_browse.clicked.connect(self._browse_folder)

        out_layout.addWidget(self.cb_save)
        out_layout.addWidget(self.cb_clear_old)
        out_layout.addWidget(self.out_folder, 2)
        out_layout.addWidget(self.btn_browse)
        root.addWidget(out_group)

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
        devices = list_ftdi_devices()
        self.spi_combo.clear()
        self.gpio_combo.clear()

        for idx, desc, serial in devices:
            label = f"[{idx}] {desc} (SN: {serial})"
            self.spi_combo.addItem(label, idx)
            self.gpio_combo.addItem(label, idx)

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
        path = QFileDialog.getExistingDirectory(self, "Select Output Folder", self.out_folder.text())
        if path:
            self.out_folder.setText(path)

    def _get_host_intr_mask(self) -> int:
        idx = self.intr_bit_combo.currentIndex()
        masks = [0x10, 0x20, 0x80, 0xA0]
        return masks[idx] if idx < len(masks) else 0xA0

    def _start_capture(self):
        if self.spi_combo.count() == 0:
            QMessageBox.warning(self, "Warning", "No FTDI devices found!")
            return

        settings = CaptureSettings(
            spi_dev_index=self.spi_combo.currentData(),
            gpio_dev_index=self.gpio_combo.currentData(),
            clock_hz=int(self.clock_combo.currentText()),
            use_3phase_clk=self.cb_3phase.isChecked(),
            num_frames=self.spin_frames.value(),
            host_intr_mask=self._get_host_intr_mask(),
            save_to_folder=self.cb_save.isChecked(),
            out_folder=self.out_folder.text(),
            device_type=self.device_type_combo.currentText(),
            poll_sleep_us=self.spin_poll_us.value(),
            settle_us=self.spin_settle_us.value(),
            preview_every=self.spin_preview.value(),
            log_every=self.spin_log.value(),
            byteswap32=self.cb_byteswap.isChecked(),
            resync_on_start=self.cb_resync.isChecked(),
        )

        self._log("===== Capture Start =====")
        self._log(f"Device Type     : {settings.device_type}")
        self._log(f"SPI index       : {settings.spi_dev_index}")
        self._log(f"GPIO index      : {settings.gpio_dev_index}{' (separate)' if settings.device_type=='AOP' and settings.gpio_dev_index!=settings.spi_dev_index else ''}")
        self._log(f"SPI Clock req   : {settings.clock_hz/1e6:.1f} MHz")
        self._log(f"3-Phase Clock   : {'ON' if settings.use_3phase_clk else 'OFF'}")
        self._log(f"Frame Size      : {FW_FRAME_SIZE:,} bytes")
        self._log(f"Chunks/Frame    : {CHUNKS_PER_FRAME}")
        self._log(f"HOST_INTR mask  : 0x{settings.host_intr_mask:02X}  (ready when GPIO&mask==0)")
        self._log(f"Byteswap32      : {'ON' if settings.byteswap32 else 'OFF'}")
        self._log(f"Resync          : {'ON' if settings.resync_on_start else 'OFF'}")
        self._log(f"poll_us/settle_us : {settings.poll_sleep_us} / {settings.settle_us}")
        self._log(f"Output folder   : {settings.out_folder if settings.save_to_folder else '(not saving)'}")
        self._log("NOTE: existing *.bin will be deleted on start (forced).")

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
    if  total > 0                                   : 
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
