  #!/usr/bin/env python3
"""
file: ft4232h_cl_usb_gui.py
AWRL6844 SPI Data Capture Tool
Based on TI's spi_data_capture_tool.py, adapted for main_full_mss.c firmware

Uses ftd2xx library with direct MPSSE commands (not libMPSSE)
Fixed for proper HOST_INTR timing with SPI Slave mode
"""

import sys
import time
import os
from datetime import datetime
from dataclasses import dataclass

try:
    import ftd2xx as ftd
except ImportError:
    print("ERROR: ftd2xx library not found. Install with: pip install ftd2xx")
    sys.exit(1)

from PyQt6.QtCore import Qt, QThread, pyqtSignal, QObject, pyqtSlot
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QPlainTextEdit, QLineEdit,
    QCheckBox, QMessageBox, QGroupBox, QFileDialog, QSpinBox,
    QProgressBar
)

# ======================== FTDI MPSSE Configuration ========================
# AN108: Command Processor for MPSSE and MCU Host Bus Emulation Modes

FTDI_CFG_60MHZ_SYSCLK = b'\x8A'         # Disable Clk Divide by 5 -> 60MHz
FTDI_CFG_NO_ADAPT_CLK = b'\x97'         # Turn Off Adaptive clocking
FTDI_CFG_NO_3PHAS_CLK = b'\x8D'         # Disable 3 Phase Data Clocking
FTDI_CFG_NO_LOOPBACK  = b'\x85'         # Disconnect TDI to TDO Loopback
FTDI_CFG_SPI_4PIN_CFG = b'\x80\x08\x0B' # Set Data bits LowByte [Value=0x08, Dir=0x0B]

# MPSSE Commands
FTDI_CMD_CS_LOW       = b'\x80\x00\x0B' # CS = LOW
FTDI_CMD_CS_HIGH      = b'\x80\x08\x0B' # CS = HIGH
FTDI_CMD_READ_BYTES   = b'\x20'         # Clock Data Bytes In on +ve edge MSB first
FTDI_CMD_READ_GPIO    = b'\x81'         # Read Data bits LowByte

FTDI_MAX_SIZE = 65536  # Max transfer size per chunk


# ======================== FTDI Helper Functions ========================

def swap_bytes_32bit(data: bytes) -> bytes:
    """Swap bytes within each 32-bit word (Little Endian to Big Endian conversion)
    
    AWRL6844 sends data as 32-bit words, but byte order is reversed on SPI.
    Example: TX 00 00 01 02 -> RX 02 01 00 00
    This function corrects it back to: 00 00 01 02
    """
    if len(data) % 4 != 0:
        # Pad to multiple of 4 if needed
        data = data + bytes(4 - (len(data) % 4))
    
    result = bytearray(len(data))
    for i in range(0, len(data), 4):
        # Reverse bytes within each 4-byte word
        result[i] = data[i + 3]
        result[i + 1] = data[i + 2]
        result[i + 2] = data[i + 1]
        result[i + 3] = data[i]
    return bytes(result)


def set_clk(handle, hz: int):
    """Set SPI clock frequency from 60MHz system clock"""
    if hz > 30000000:
        raise ValueError('Max SCK rate is 30MHz')
    div = int((60000000 / (hz * 2)) - 1)
    cmd = bytes((0x86, div % 256, div // 256))
    handle.write(cmd)


def set_device(handle, clk_speed: int = 15000000, latency_timer: int = 1):
    """Initialize FTDI device for SPI operation"""
    handle.resetDevice()
    
    # Purge USB receive buffer
    rx_bytes, tx_bytes, event_status = handle.getStatus()
    if rx_bytes > 0:
        handle.read(rx_bytes)
    
    handle.setUSBParameters(65535, 65535)
    handle.setChars(False, 0, False, 0)
    handle.setTimeouts(200000, 200000)
    handle.setLatencyTimer(latency_timer)
    handle.setBitMode(0, 0)  # Reset MPSSE
    handle.setBitMode(0, 2)  # Enable MPSSE mode
    time.sleep(0.050)
    
    handle.write(FTDI_CFG_60MHZ_SYSCLK)
    handle.write(FTDI_CFG_NO_ADAPT_CLK)
    handle.write(FTDI_CFG_NO_3PHAS_CLK)
    handle.write(FTDI_CFG_SPI_4PIN_CFG)
    set_clk(handle, clk_speed)
    time.sleep(0.020)
    handle.write(FTDI_CFG_NO_LOOPBACK)
    time.sleep(0.030)


def read_gpio(handle) -> int:
    """Read GPIO pins state (returns 8-bit value)"""
    handle.write(FTDI_CMD_READ_GPIO)
    res = handle.read(1)
    return int.from_bytes(res, 'big')


def spi_read(handle, length: int) -> bytes:
    """Read bytes via SPI (with CS handling)"""
    if length > 0x10000 or length < 1:
        raise ValueError('Length must be between 1 and 65536')
    
    # Length encoding: 0x0000 = 1 byte, 0x0001 = 2 bytes, etc.
    len_bytes = int(length - 1).to_bytes(2, 'little')
    cmd = FTDI_CMD_CS_LOW + FTDI_CMD_READ_BYTES + len_bytes + FTDI_CMD_CS_HIGH
    handle.write(cmd)
    return handle.read(length)


def list_ftdi_devices() -> list[tuple[int, str, str]]:
    """List available FTDI devices"""
    devices = []
    try:
        num = ftd.listDevices()
        if num is None:
            return devices
        for i in range(len(num)):
            try:
                dev = ftd.open(i)
                info = dev.getDeviceInfo()
                desc = info.get('description', b'Unknown').decode(errors='ignore')
                serial = info.get('serial', b'').decode(errors='ignore')
                devices.append((i, desc, serial))
                dev.close()
            except Exception:
                pass
    except Exception:
        pass
    return devices


# ======================== SPI Worker Thread ========================

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
    host_intr_mask: int  # GPIO bit mask for HOST_INTR (e.g., 0x10 for bit4)
    save_to_file: bool
    out_path: str
    device_type: str  # "AOP" or "FCCSP"


class SpiCaptureWorker(QObject):
    status = pyqtSignal(str)
    error = pyqtSignal(str)
    progress = pyqtSignal(int, int)  # (current_frame, total_frames)
    frame_received = pyqtSignal(int, int)  # (frame_idx, bytes)
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
            # Calculate frame size based on firmware configuration
            # adcDataPerFrame = numOfChirpsInBurst * numOfBurstsInFrame * numOfAdcSamples * 4 * 2
            # where:
            #   4 = number of RX antennas (we use user-selected value)
            #   2 = bytes per ADC sample (16-bit = 2 bytes)
            frame_size = (self.settings.chirps_per_burst * 
                         self.settings.bursts_per_frame * 
                         self.settings.adc_samples * 
                         self.settings.rx_antennas * 2)
            
            # With default config: 64 * 1 * 256 * 4 * 2 = 131,072 bytes (128 KB)
            
            self.status.emit(f"Frame size: {frame_size} bytes ({frame_size/1024:.1f} KB)")
            
            # Open FTDI devices
            self._dev_spi = ftd.open(self.settings.spi_dev_index)
            self.status.emit(f"SPI device opened: index {self.settings.spi_dev_index}")
            
            set_device(self._dev_spi, self.settings.clock_hz, latency_timer=1)
            self.status.emit(f"SPI configured: {self.settings.clock_hz/1e6:.1f} MHz")
            
            # For AOP: Use separate GPIO device for HOST_INTR reading
            # For FCCSP: Same device can be used
            if self.settings.device_type == "AOP" and self.settings.gpio_dev_index != self.settings.spi_dev_index:
                self._dev_gpio = ftd.open(self.settings.gpio_dev_index)
                set_device(self._dev_gpio, 1000000, latency_timer=1)
                self.status.emit(f"GPIO device opened: index {self.settings.gpio_dev_index}")
            else:
                self._dev_gpio = self._dev_spi
            
            # Open output file
            if self.settings.save_to_file and self.settings.out_path:
                dir_path = os.path.dirname(self.settings.out_path)
                if dir_path:
                    os.makedirs(dir_path, exist_ok=True)
                fout = open(self.settings.out_path, 'wb')
                self.status.emit(f"Saving to: {self.settings.out_path}")
            
            self.status.emit("Waiting for data from radar sensor...")
            
            # Main capture loop (based on TI spi_data_capture_tool.py)
            frame_count = 0
            total_bytes = 0
            target_frames = self.settings.num_frames if self.settings.num_frames > 0 else float('inf')
            
            while self._running and frame_count < target_frames:
                frame_data = bytearray()
                size_remaining = frame_size
                
                # Process frame in chunks (max 64KB per transfer)
                while size_remaining > 0 and self._running:
                    chunk_size = min(size_remaining, FTDI_MAX_SIZE)
                    
                    # Wait for HOST_INTR LOW before each chunk
                    # This signals that firmware is ready to send
                    timeout_start = time.time()
                    intr_wait_logged = False
                    
                    while self._running:
                        gpio_val = read_gpio(self._dev_gpio)
                        
                        # Check HOST_INTR based on device type
                        if self.settings.device_type == "FCCSP":
                            intr_active = (gpio_val & 0x10) == 0
                        else:
                            intr_active = (gpio_val & 0xA0) == 0
                        
                        if intr_active:
                            break
                        
                        elapsed = time.time() - timeout_start
                        if elapsed > 5.0 and not intr_wait_logged:
                            self.status.emit(f"Waiting for HOST_INTR... (GPIO=0x{gpio_val:02X})")
                            intr_wait_logged = True
                            timeout_start = time.time()
                        
                        # Minimal delay - firmware is waiting for our clock
                        time.sleep(0.00001)  # 10us - reduced from 100us
                    
                    if not self._running:
                        break
                    
                    # Read SPI data immediately - firmware is in MCSPI_transfer() waiting
                    chunk = spi_read(self._dev_spi, chunk_size)
                    
                    # Apply 32-bit byte swap (AWRL6844 sends as 32-bit words with reversed byte order)
                    chunk = swap_bytes_32bit(chunk)
                    
                    frame_data.extend(chunk)
                    size_remaining -= len(chunk)
                
                if not self._running:
                    break
                
                if len(frame_data) == 0:
                    continue
                
                # Check if data is valid (not all zeros)
                if all(b == 0 for b in frame_data):
                    self.status.emit(f"Warning: Frame {frame_count} contains all zeros - skipping")
                    continue
                
                # Save to file
                if fout:
                    fout.write(frame_data)
                    fout.flush()
                
                total_bytes += len(frame_data)
                frame_count += 1
                
                # Emit signals
                self.frame_received.emit(frame_count, len(frame_data))
                self.progress.emit(frame_count, int(target_frames) if target_frames != float('inf') else 0)
                
                # Preview first 64 bytes
                self.preview.emit(bytes(frame_data[:64]))
                
                self.status.emit(f"Frame {frame_count}: {len(frame_data)} bytes (Total: {total_bytes/1024:.1f} KB)")
            
            self.status.emit(f"Capture complete: {frame_count} frames, {total_bytes} bytes")
            
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
        self.setWindowTitle("AWRL6844 SPI Data Capture Tool (TI Reference Based)")
        self.resize(950, 750)
        
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
        
        # ---- Radar Configuration ----
        radar_group = QGroupBox("Radar Configuration (must match firmware - main_full_mss.c)")
        radar_layout = QVBoxLayout(radar_group)
        
        row1 = QHBoxLayout()
        self.spin_adc = QSpinBox()
        self.spin_adc.setRange(64, 4096)
        self.spin_adc.setValue(256)  # mmwcfg.profileComCfg.numOfAdcSamples
        
        self.spin_chirps = QSpinBox()
        self.spin_chirps.setRange(1, 512)
        self.spin_chirps.setValue(64)  # mmwcfg.frameCfg.numOfChirpsInBurst
        
        self.spin_bursts = QSpinBox()
        self.spin_bursts.setRange(1, 64)
        self.spin_bursts.setValue(1)  # mmwcfg.frameCfg.numOfBurstsInFrame
        
        self.spin_rx = QSpinBox()
        self.spin_rx.setRange(1, 4)
        self.spin_rx.setValue(4)  # 4 RX antennas enabled
        
        row1.addWidget(QLabel("ADC Samples:"))
        row1.addWidget(self.spin_adc)
        row1.addWidget(QLabel("Chirps/Burst:"))
        row1.addWidget(self.spin_chirps)
        row1.addWidget(QLabel("Bursts/Frame:"))
        row1.addWidget(self.spin_bursts)
        row1.addWidget(QLabel("RX Antennas:"))
        row1.addWidget(self.spin_rx)
        radar_layout.addLayout(row1)
        
        row2 = QHBoxLayout()
        self.spin_frames = QSpinBox()
        self.spin_frames.setRange(0, 100000)
        self.spin_frames.setValue(100)  # Match mmwcfg.frameCfg.numOfFrames (0 = infinite)
        self.spin_frames.setSpecialValueText("Infinite")
        
        self.clock_combo = QComboBox()
        self.clock_combo.addItems(["15000000", "12000000", "6000000", "1000000"])
        
        self.lbl_frame_size = QLabel("Frame size: -- bytes")
        
        row2.addWidget(QLabel("Frames (0=infinite):"))
        row2.addWidget(self.spin_frames)
        row2.addWidget(QLabel("SPI Clock (Hz):"))
        row2.addWidget(self.clock_combo)
        row2.addStretch(1)
        row2.addWidget(self.lbl_frame_size)
        radar_layout.addLayout(row2)
        
        # Help text
        help_label = QLabel(
            "Firmware formula: adcDataPerFrame = chirps × bursts × adc_samples × rx × 2\n"
            "Default: 64 × 1 × 256 × 4 × 2 = 131,072 bytes (128 KB per frame)"
        )
        help_label.setStyleSheet("color: gray; font-size: 10px;")
        radar_layout.addWidget(help_label)
        
        # Connect spinboxes to update frame size label
        self.spin_adc.valueChanged.connect(self._update_frame_size)
        self.spin_chirps.valueChanged.connect(self._update_frame_size)
        self.spin_bursts.valueChanged.connect(self._update_frame_size)
        self.spin_rx.valueChanged.connect(self._update_frame_size)
        self._update_frame_size()
        
        root.addWidget(radar_group)
        
        # ---- HOST_INTR Configuration ----
        intr_group = QGroupBox("HOST_INTR GPIO Configuration")
        intr_layout = QHBoxLayout(intr_group)
        
        self.intr_bit_combo = QComboBox()
        self.intr_bit_combo.addItems([
            "Bit 4 (0x10) - FCCSP default",
            "Bit 5 (0x20)",
            "Bit 7 (0x80)",
            "Bits 5+7 (0xA0) - AOP default"
        ])
        self.intr_bit_combo.setCurrentIndex(3)  # AOP default
        
        intr_layout.addWidget(QLabel("HOST_INTR Pin:"))
        intr_layout.addWidget(self.intr_bit_combo, 2)
        intr_layout.addStretch(1)
        
        root.addWidget(intr_group)
        
        # ---- Output File ----
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
        
        # ---- Log Display ----
        log_header = QHBoxLayout()
        log_header.addWidget(QLabel("Log:"))
        log_header.addStretch(1)
        self.btn_clear_log = QPushButton("Clear")
        self.btn_clear_log.clicked.connect(lambda: self.log_view.clear())
        log_header.addWidget(self.btn_clear_log)
        root.addLayout(log_header)
        
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        root.addWidget(self.log_view, 1)
        
        # ---- Status Bar ----
        self.status_label = QLabel("Ready")
        self.statusBar().addWidget(self.status_label, 1)

    def _log(self, msg: str):
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        self.log_view.appendPlainText(f"[{timestamp}] {msg}")
        cursor = self.log_view.textCursor()
        cursor.movePosition(cursor.MoveOperation.End)
        self.log_view.setTextCursor(cursor)

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
            QMessageBox.warning(self, "Warning", 
                "No FTDI devices found.\n\n"
                "Check:\n"
                "1. FT4232H cable is connected\n"
                "2. FTDI D2XX drivers are installed\n"
                "3. Device shows in Device Manager")

    def _on_device_type_changed(self, device_type: str):
        """Update HOST_INTR selection based on device type"""
        if device_type == "FCCSP":
            self.intr_bit_combo.setCurrentIndex(0)  # Bit 4 (0x10)
        else:
            self.intr_bit_combo.setCurrentIndex(3)  # Bits 5+7 (0xA0)

    def _update_frame_size(self):
        # Firmware formula: adcDataPerFrame = chirps * bursts * adc_samples * 4 * 2
        # The '4' is hardcoded for 4 RX antennas, '2' is bytes per sample
        # We replace '4' with user-selected rx_antennas for flexibility
        # Result: chirps * bursts * adc_samples * rx * 2
        size = (self.spin_chirps.value() * 
                self.spin_bursts.value() * 
                self.spin_adc.value() * 
                self.spin_rx.value() * 2)
        self.lbl_frame_size.setText(f"Frame size: {size:,} bytes ({size/1024:.1f} KB)")

    def _get_frame_size(self) -> int:
        # Must match firmware: adcDataPerFrame = chirps * bursts * adc_samples * rx * 2
        return (self.spin_chirps.value() * 
                self.spin_bursts.value() * 
                self.spin_adc.value() * 
                self.spin_rx.value() * 2)

    def _browse_file(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Save ADC Data", self.out_path.text(), "Binary (*.bin);;All (*.*)")
        if path:
            self.out_path.setText(path)

    def _get_host_intr_mask(self) -> int:
        idx = self.intr_bit_combo.currentIndex()
        masks = [0x10, 0x20, 0x80, 0xA0]
        return masks[idx] if idx < len(masks) else 0x10

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
            save_to_file=self.cb_save.isChecked(),
            out_path=self.out_path.text(),
            device_type=self.device_type_combo.currentText()
        )
        
        self._log(f"Starting capture: {settings.device_type} mode")
        self._log(f"Frame size: {self._get_frame_size():,} bytes")
        self._log(f"Target frames: {settings.num_frames if settings.num_frames > 0 else 'Infinite'}")
        self._log(f"HOST_INTR mask: 0x{settings.host_intr_mask:02X}")
        
        # UI state
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.progress.setValue(0)
        if settings.num_frames > 0:
            self.progress.setMaximum(settings.num_frames)
        else:
            self.progress.setMaximum(0)  # Indeterminate
        
        # Create worker thread
        self.capture_thread = QThread()
        self.capture_worker = SpiCaptureWorker(settings)
        self.capture_worker.moveToThread(self.capture_thread)
        
        # Connect signals
        self.capture_thread.started.connect(self.capture_worker.start)
        self.capture_worker.status.connect(self._on_status)
        self.capture_worker.error.connect(self._on_error)
        self.capture_worker.progress.connect(self._on_progress)
        self.capture_worker.frame_received.connect(self._on_frame)
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

    def _on_frame(self, frame_idx: int, nbytes: int):
        pass  # Status already logged

    def _on_preview(self, data: bytes):
        hex_str = ' '.join(f'{b:02X}' for b in data)
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