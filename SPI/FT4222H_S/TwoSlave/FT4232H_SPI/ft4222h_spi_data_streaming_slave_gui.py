# File: ft4222h_spi_data_streaming_slave_gui.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ft4222h_spi_data_streaming_slave_gui.py
======================================

FT4222H SPI SLAVE MODE GUI 애플리케이션

AWRL6844 레이더 센서(=SPI Master)로부터 SPI를 통해 ADC 데이터를 수신하는
GUI 기반 캡처 도구입니다. 기능은 ft4222h_spi_data_streaming_slave_function.py의
SpiCapture 클래스를 사용하여 구현합니다.

Dependencies:
- ft4222h_spi_data_streaming_slave_function
- PyQt6

:author:
:date: 2026
"""

import sys
import os
from datetime import datetime

from PyQt6.QtCore import QThread, pyqtSignal, QObject, pyqtSlot
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QPlainTextEdit, QLineEdit,
    QCheckBox, QMessageBox, QGroupBox, QFileDialog, QSpinBox,
    QProgressBar
)

from ft4222h_spi_data_streaming_slave_function import (
    list_ft4222_devices, CaptureSettings, SpiCapture, delete_existing_frame_files
)

class CaptureWorker(QObject):
    status = pyqtSignal(str)
    error = pyqtSignal(str)
    progress = pyqtSignal(int, int)
    preview = pyqtSignal(bytes)
    finished = pyqtSignal()

    def __init__(self, settings):
        super().__init__()
        self.settings = settings
        self.capture = None

    @pyqtSlot()
    def start(self):
        def status_cb(msg):
            self.status.emit(msg)

        def error_cb(msg):
            self.error.emit(msg)

        def progress_cb(current, total):
            self.progress.emit(current, total)

        def preview_cb(data):
            self.preview.emit(data)

        def finished_cb():
            self.finished.emit()

        self.capture = SpiCapture(self.settings, status_cb, error_cb, progress_cb, preview_cb, finished_cb)
        self.capture.start()

    @pyqtSlot()
    def stop(self):
        if self.capture:
            self.capture.stop()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AWRL6844 SPI Data Capture Tool (FT4222H SPI SLAVE)")
        self.resize(1500, 900)

        self.capture_thread: Optional[QThread] = None
        self.capture_worker: Optional[CaptureWorker] = None

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

        self.spin_stable_sleep = QSpinBox()
        self.spin_stable_sleep.setRange(0, 1000)
        self.spin_stable_sleep.setValue(10)

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

        self.spin_frame_size = QSpinBox()
        self.spin_frame_size.setRange(1024, 1024*1024)
        self.spin_frame_size.setValue(65536)
        self.spin_frame_size.setSingleStep(1024)

        self.spin_frames = QSpinBox()
        self.spin_frames.setRange(0, 1000000)
        self.spin_frames.setValue(30)
        self.spin_frames.setSpecialValueText("Infinite")

        self.spin_read_timeout = QSpinBox()
        self.spin_read_timeout.setRange(1, 60)
        self.spin_read_timeout.setValue(10)

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

        self.cb_byteswap = QCheckBox("Byteswap32")
        self.cb_byteswap.setChecked(False)

        self.cb_variable_frame = QCheckBox("Variable frame")
        self.cb_variable_frame.setChecked(True)

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
        slave_layout.addWidget(self.cb_variable_frame)
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
            variable_frame=self.cb_variable_frame.isChecked(),
            read_chunk_size=16384,
        )

        if settings.save_to_file and not settings.out_dir:
            QMessageBox.warning(self, "Warning", "Output folder is empty. Please select a folder.")
            return

        if settings.overwrite_on_start:
            delete_existing_frame_files(settings.out_dir)

        self._log("===== Capture Start (SPI SLAVE MODE) =====")
        # ... (log other settings as in original)

        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.progress.setValue(0)
        self.progress.setMaximum(settings.num_frames if settings.num_frames > 0 else 0)

        self.capture_thread = QThread()
        self.capture_worker = CaptureWorker(settings)
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