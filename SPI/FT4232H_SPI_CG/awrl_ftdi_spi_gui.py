  #!/usr/bin/env python3
"""
awrl_ftdi_spi_gui.py

Matches firmware behavior:

- Firmware sends adcDataPerFrame = 131072 bytes per frame
- Transfer is done in 2 chunks of 65536 bytes (MAXSPISIZEFTDI)
- Each chunk:
    HOST_INTR LOW
    MCSPI_transfer(dataSize=32, txBuf=uint32_t*, count=tempSize/4)
    HOST_INTR HIGH

Host side (FT4232H / ftd2xx / MPSSE):
- Wait HOST_INTR LOW before each chunk, then SPI read exactly 65536 bytes
- Because firmware uses dataSize=32 + uint32_t* on little-endian MCU,
  bytes appear reversed within each 32-bit word on the wire.
  => Apply 32-bit byteswap on host to recover intended byte stream:
     00 00 00 69 00 01 02 03 04 05 ...
"""

import sys
import time
import os
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
CHUNK_SIZE = 65536  # MAXSPISIZEFTDI
CHUNKS_PER_FRAME = FW_FRAME_SIZE // CHUNK_SIZE

# ======================== FTDI MPSSE Configuration ========================
FTDI_CFG_60MHZ_SYSCLK = b'\x8A'
FTDI_CFG_NO_ADAPT_CLK = b'\x97'
FTDI_CFG_NO_3PHAS_CLK = b'\x8D'
FTDI_CFG_NO_LOOPBACK  = b'\x85'
FTDI_CFG_SPI_4PIN_CFG = b'\x80\x08\x0B'

FTDI_CMD_CS_LOW       = b'\x80\x00\x0B'
FTDI_CMD_CS_HIGH      = b'\x80\x08\x0B'
FTDI_CMD_READ_BYTES   = b'\x20'
FTDI_CMD_READ_GPIO    = b'\x81'


# ======================== Low-level helpers ========================

def set_clk(handle, hz: int):
    if hz > 30000000:
        raise ValueError("Max SCK rate is 30MHz")
    div = int((60000000 / (hz * 2)) - 1)
    cmd = bytes((0x86, div & 0xFF, (div >> 8) & 0xFF))
    handle.write(cmd)


def set_device(handle, clk_speed: int = 15000000, latency_timer: int = 1):
    handle.resetDevice()

    rx_bytes, tx_bytes, event_status = handle.getStatus()
    if rx_bytes > 0:
        handle.read(rx_bytes)

    handle.setUSBParameters(65535, 65535)
    handle.setChars(False, 0, False, 0)

    # 너무 길면 문제시 대기가 길어짐. SPI read는 어차피 우리가 exact loop로 모음.
    handle.setTimeouts(5000, 5000)

    handle.setLatencyTimer(latency_timer)
    handle.setBitMode(0, 0)
    handle.setBitMode(0, 2)
    time.sleep(0.050)

    handle.write(FTDI_CFG_60MHZ_SYSCLK)
    handle.write(FTDI_CFG_NO_ADAPT_CLK)
    handle.write(FTDI_CFG_NO_3PHAS_CLK)
    handle.write(FTDI_CFG_SPI_4PIN_CFG)
    set_clk(handle, clk_speed)
    time.sleep(0.010)
    handle.write(FTDI_CFG_NO_LOOPBACK)
    time.sleep(0.010)


def read_gpio(handle) -> int:
    handle.write(FTDI_CMD_READ_GPIO)
    return int.from_bytes(handle.read(1), "big")


def spi_read_exact(handle, length: int) -> bytes:
    if length < 1 or length > 0x10000:
        raise ValueError("Length must be 1..65536")

    len_bytes = int(length - 1).to_bytes(2, "little")
    cmd = FTDI_CMD_CS_LOW + FTDI_CMD_READ_BYTES + len_bytes + FTDI_CMD_CS_HIGH
    handle.write(cmd)

    out = bytearray()
    while len(out) < length:
        chunk = handle.read(length - len(out))
        if not chunk:
            raise TimeoutError("SPI read returned empty (timeout)")
        out.extend(chunk)
    return bytes(out)


def byteswap32_fast(data: bytes) -> bytes:
    """
    Very fast 32-bit word byteswap using array('I').byteswap() (C-implemented).
    Needed because firmware sends 32-bit words from little-endian memory with MSB-first shift.
    """
    if len(data) % 4 != 0:
        data = data + b"\x00" * (4 - (len(data) % 4))

    a = array.array("I")
    if a.itemsize != 4:
        # Fallback (rare on Windows): slow path
        b = bytearray(data)
        for i in range(0, len(b), 4):
            b[i:i+4] = b[i:i+4][::-1]
        return bytes(b)

    a.frombytes(data)
    a.byteswap()
    return a.tobytes()


def list_ftdi_devices() -> list[tuple[int, str, str]]:
    devices = []
    try:
        num = ftd.listDevices()
        if num is None:
            return devices
        for i in range(len(num)):
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
    # active when masked bits are 0
    return (gpio_val & mask) == 0


def wait_intr_low(dev_gpio, mask: int, timeout_s: float, poll_sleep_us: int) -> int:
    t0 = time.perf_counter()
    last = 0
    sleep_s = max(poll_sleep_us, 0) / 1_000_000.0
    while True:
        last = read_gpio(dev_gpio)
        if intr_active_low(last, mask):
            return last
        if (time.perf_counter() - t0) > timeout_s:
            raise TimeoutError(f"HOST_INTR LOW timeout (GPIO=0x{last:02X}, mask=0x{mask:02X})")
        if sleep_s > 0:
            time.sleep(sleep_s)


def decode_fw_frame_no(u32_be: int) -> int:
    """
    FW: cnt(decimal) -> str(decimal) -> strtol(str, 16) -> u32_be 전송
    PC: u32_be -> hex string -> int(hex_string, 10) 로 원래 cnt 복원
    예) u32_be=0x311(785) -> "311" -> 311
    """
    s = format(u32_be, 'X')          # "311"
    return int(s, 10)                # 311


# ======================== Capture Worker ========================

@dataclass
class CaptureSettings:
    spi_dev_index: int
    gpio_dev_index: int
    clock_hz: int
    num_frames: int
    host_intr_mask: int
    save_to_file: bool
    out_path: str
    device_type: str
    poll_sleep_us: int
    flush_every: int
    preview_every: int
    log_every: int
    byteswap32: bool


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
        fout = None

        try:
            self.status.emit(f"Firmware frame size forced: {FW_FRAME_SIZE:,} bytes ({CHUNKS_PER_FRAME} x 64KB)")
            self.status.emit(f"HOST_INTR mask: 0x{self.settings.host_intr_mask:02X}")
            self.status.emit(f"byteswap32: {'ON' if self.settings.byteswap32 else 'OFF'} (recommended ON for dataSize=32)")

            self._dev_spi = ftd.open(self.settings.spi_dev_index)
            self.status.emit(f"SPI device opened: index {self.settings.spi_dev_index}")
            set_device(self._dev_spi, self.settings.clock_hz, latency_timer=1)
            self.status.emit(f"SPI configured: {self.settings.clock_hz/1e6:.1f} MHz")

            # Optional separate GPIO handle
            if self.settings.device_type == "AOP" and self.settings.gpio_dev_index != self.settings.spi_dev_index:
                self._dev_gpio = ftd.open(self.settings.gpio_dev_index)
                set_device(self._dev_gpio, 1_000_000, latency_timer=1)
                self.status.emit(f"GPIO device opened: index {self.settings.gpio_dev_index}")
            else:
                self._dev_gpio = self._dev_spi

            # Quick sanity: if mask bits are ALWAYS 0 for a while -> warning (means "always active")
            zeros = 0
            for _ in range(30):
                v = read_gpio(self._dev_gpio)
                if (v & self.settings.host_intr_mask) == 0:
                    zeros += 1
                time.sleep(0.005)
            if zeros == 30:
                self.status.emit(
                    "Warning: (GPIO & mask) stayed 0 for 150ms -> HOST_INTR may look always-active.\n"
                    "If you see instability, try mask 0x20 or 0x80 (single pin) instead of 0xA0."
                )

            if self.settings.save_to_file and self.settings.out_path:
                dir_path = os.path.dirname(self.settings.out_path)
                if dir_path:
                    os.makedirs(dir_path, exist_ok=True)
                fout = open(self.settings.out_path, "wb", buffering=1024 * 1024)
                self.status.emit(f"Saving to: {self.settings.out_path}")

            self.status.emit("Waiting for data from radar sensor...")

            frame_count = 0
            total_bytes = 0
            start_t = time.perf_counter()

            target_frames = self.settings.num_frames if self.settings.num_frames > 0 else float("inf")
            total_for_progress = int(target_frames) if target_frames != float("inf") else 0

            while self._running and frame_count < target_frames:
                frame_no = None
                first64 = None

                # Stream per chunk (no need to hold full 128KB in RAM)
                for chunk_idx in range(CHUNKS_PER_FRAME):
                    wait_intr_low(
                        self._dev_gpio,
                        self.settings.host_intr_mask,
                        timeout_s=5.0,
                        poll_sleep_us=self.settings.poll_sleep_us,
                    )

                    raw = spi_read_exact(self._dev_spi, CHUNK_SIZE)

                    if self.settings.byteswap32:
                        raw = byteswap32_fast(raw)

                    # parse header & preview from very first chunk
                    if chunk_idx == 0:
                        # header is bytes[0..3] big-endian
                        raw_u32 = int.from_bytes(raw[0:4], "big")
                        frame_no = decode_fw_frame_no(raw_u32)
                        first64 = raw[:64]

                    if fout:
                        fout.write(raw)

                # end of one frame
                frame_count += 1
                total_bytes += FW_FRAME_SIZE

                if fout and self.settings.flush_every > 0 and (frame_count % self.settings.flush_every) == 0:
                    fout.flush()

                self.progress.emit(frame_count, total_for_progress)

                if self.settings.preview_every > 0 and (frame_count % self.settings.preview_every) == 0 and first64 is not None:
                    self.preview.emit(first64)

                if self.settings.log_every > 0 and (frame_count % self.settings.log_every) == 0:
                    elapsed = time.perf_counter() - start_t
                    mbps = (total_bytes / (1024 * 1024)) / elapsed if elapsed > 0 else 0.0
                    self.status.emit(
                        f"Frame {frame_count}: {FW_FRAME_SIZE} bytes | Total: {total_bytes/1024:.1f} KB | "
                        f"{mbps:.2f} MB/s | FrameNo={frame_no}"
                    )

            if fout:
                fout.flush()

            elapsed = time.perf_counter() - start_t
            mbps = (total_bytes / (1024 * 1024)) / elapsed if elapsed > 0 else 0.0
            self.status.emit(f"Capture complete: {frame_count} frames, {total_bytes} bytes, {mbps:.2f} MB/s")

        except Exception as e:
            import traceback
            self.error.emit(f"Capture error: {e}\n{traceback.format_exc()}")

        finally:
            try:
                if fout:
                    fout.close()
            except Exception:
                pass
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
        self.setWindowTitle("AWRL6844 SPI Data Capture Tool (FT4232H / ftd2xx / FW=131072B)")
        self.resize(980, 800)

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
        self.device_type_combo.currentTextChanged.connect(self._on_device_type_changed)

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
        # 펌웨어는 "1개 핀" 토글임. 보드/배선에 따라 bit5 또는 bit7일 가능성이 큼.
        # 0xA0은 legacy(두 핀)용으로 남김.
        self.intr_bit_combo.addItems([
            "Bit 4 (0x10)",
            "Bit 5 (0x20)  <-- 추천 (single pin)",
            "Bit 7 (0x80)  <-- 추천 (single pin)",
            "Bits 5+7 (0xA0) legacy"
        ])
        self.intr_bit_combo.setCurrentIndex(1)

        intr_layout.addWidget(QLabel("HOST_INTR mask:"))
        intr_layout.addWidget(self.intr_bit_combo, 2)
        intr_layout.addStretch(1)
        root.addWidget(intr_group)

        # Options
        opt_group = QGroupBox("Capture Options (speed)")
        opt_layout = QHBoxLayout(opt_group)

        self.clock_combo = QComboBox()
        self.clock_combo.addItems(["30000000", "24000000", "15000000", "12000000", "6000000", "1000000"])
        self.clock_combo.setCurrentText("15000000")

        self.spin_frames = QSpinBox()
        self.spin_frames.setRange(0, 1000000)
        self.spin_frames.setValue(100)
        self.spin_frames.setSpecialValueText("Infinite")

        self.spin_poll_us = QSpinBox()
        self.spin_poll_us.setRange(0, 5000)
        self.spin_poll_us.setValue(50)  # 10us도 가능하지만 USB/GPIO 폴링 부하 큼

        self.spin_flush = QSpinBox()
        self.spin_flush.setRange(0, 1000)
        self.spin_flush.setValue(0)  # 속도 최우선: 프레임마다 flush 하지 않음(끝에서 flush)

        self.spin_preview = QSpinBox()
        self.spin_preview.setRange(0, 1000)
        self.spin_preview.setValue(1)  # 비교용으로 1. 속도 올릴 땐 10~0 추천

        self.spin_log = QSpinBox()
        self.spin_log.setRange(1, 1000)
        self.spin_log.setValue(1)  # 속도 올릴 땐 5~10 추천

        self.cb_byteswap = QCheckBox("byteswap32 (MUST for dataSize=32)")
        self.cb_byteswap.setChecked(True)

        opt_layout.addWidget(QLabel("SPI Clock (Hz):"))
        opt_layout.addWidget(self.clock_combo)
        opt_layout.addWidget(QLabel("Frames (0=inf):"))
        opt_layout.addWidget(self.spin_frames)
        opt_layout.addWidget(QLabel("GPIO poll (us):"))
        opt_layout.addWidget(self.spin_poll_us)
        opt_layout.addWidget(QLabel("Flush every N (0=off):"))
        opt_layout.addWidget(self.spin_flush)
        opt_layout.addWidget(QLabel("Preview every N (0=off):"))
        opt_layout.addWidget(self.spin_preview)
        opt_layout.addWidget(QLabel("Log every N:"))
        opt_layout.addWidget(self.spin_log)
        opt_layout.addWidget(self.cb_byteswap)
        opt_layout.addStretch(1)
        root.addWidget(opt_group)

        # Output file
        file_group = QGroupBox("Output File")
        file_layout = QHBoxLayout(file_group)

        self.cb_save = QCheckBox("Save to file")
        self.cb_save.setChecked(True)

        self.out_path = QLineEdit()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.out_path.setText(f"adc_data_{timestamp}.bin")

        self.btn_browse = QPushButton("Browse...")
        self.btn_browse.clicked.connect(self._browse_file)

        file_layout.addWidget(self.cb_save)
        file_layout.addWidget(self.out_path, 2)
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

    def _on_device_type_changed(self, device_type: str):
        # 펌웨어는 single pin 토글이므로 AOP여도 bit5/bit7 중 하나가 보통 맞습니다.
        pass

    def _browse_file(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Save ADC Data", self.out_path.text(), "Binary (*.bin);;All (*.*)"
        )
        if path:
            self.out_path.setText(path)

    def _get_host_intr_mask(self) -> int:
        idx = self.intr_bit_combo.currentIndex()
        masks = [0x10, 0x20, 0x80, 0xA0]
        return masks[idx] if idx < len(masks) else 0x20

    def _start_capture(self):
        if self.spi_combo.count() == 0:
            QMessageBox.warning(self, "Warning", "No FTDI devices found!")
            return

        settings = CaptureSettings(
            spi_dev_index=self.spi_combo.currentData(),
            gpio_dev_index=self.gpio_combo.currentData(),
            clock_hz=int(self.clock_combo.currentText()),
            num_frames=self.spin_frames.value(),
            host_intr_mask=self._get_host_intr_mask(),
            save_to_file=self.cb_save.isChecked(),
            out_path=self.out_path.text(),
            device_type=self.device_type_combo.currentText(),
            poll_sleep_us=self.spin_poll_us.value(),
            flush_every=self.spin_flush.value(),
            preview_every=self.spin_preview.value(),
            log_every=self.spin_log.value(),
            byteswap32=self.cb_byteswap.isChecked(),
        )

        self._log(f"Starting capture: {settings.device_type} mode")
        self._log(f"Fixed Frame size: {FW_FRAME_SIZE} bytes ({CHUNKS_PER_FRAME} chunks)")
        self._log(f"HOST_INTR mask: 0x{settings.host_intr_mask:02X} | poll {settings.poll_sleep_us}us")
        self._log(f"SPI Clock: {settings.clock_hz/1e6:.1f} MHz | byteswap32={'ON' if settings.byteswap32 else 'OFF'}")
        self._log(f"FlushEvery={settings.flush_every} PreviewEvery={settings.preview_every} LogEvery={settings.log_every}")

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
        # 원하는 형태: 00 00 00 69 00 01 02 03 ...
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
