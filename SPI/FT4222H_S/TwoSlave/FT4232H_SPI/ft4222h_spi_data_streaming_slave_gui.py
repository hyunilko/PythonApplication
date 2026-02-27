  # File: ft4222h_spi_data_streaming_slave_gui.py
  #!/usr/bin/env python3
  # -*- coding: utf-8 -*-
"""
ft4222h_spi_data_streaming_slave_gui.py
 ======================================

FT4222H SPI SLAVE MODE GUI 애플리케이션 (PyQt6)

이 파일은 "GUI(화면)"를 담당합니다.
실제 SPI 캡처(FT4222H Slave로 데이터 수신)는
ft4222h_spi_data_streaming_slave_function.py 의 SpiCapture 클래스가 수행합니다.

PyQt에서 꼭 알아두실 핵심 개념 (이 파일에서 계속 등장합니다)
------------------------------------------------
1) 이벤트 루프(Event Loop)
   - app.exec()가 실행되면 PyQt는 "이벤트 루프"에 들어갑니다.
   - 버튼 클릭/타이머/윈도우 갱신 같은 이벤트를 처리하며, 이 루프가 멈추면 GUI가 멈춘 것처럼 보입니다.

2) UI 스레드(Main Thread)와 작업 스레드(Worker Thread)
   - PyQt/Qt에서는 "GUI 위젯은 메인 스레드에서만" 안전하게 변경할 수 있습니다.
   - SPI 캡처처럼 오래 걸리고 블로킹(blocking)될 수 있는 작업을 메인 스레드에서 실행하면,
     이벤트 루프가 막혀서 창이 "응답 없음"이 됩니다.
   - 그래서 이 파일은 QThread + Worker(QObject) 구조로 캡처를 별도 스레드에서 실행합니다.

3) Signal / Slot(시그널/슬롯)
   - 스레드 간 통신(작업 스레드 → UI 스레드)에 가장 안전한 방법입니다.
   - Worker에서 status.emit(...) 같은 시그널을 발생시키면,
     MainWindow의 _on_status 같은 슬롯(일반 함수)에 연결되어 UI를 업데이트합니다.

Dependencies: 
- ft4222h_spi_data_streaming_slave_function
- PyQt6

: author: 
: date  : 2026
"""

import sys
import os
from datetime import datetime
from typing import Optional

from PyQt6.QtCore import QThread, pyqtSignal, QObject, pyqtSlot
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QPlainTextEdit, QLineEdit,
    QCheckBox, QMessageBox, QGroupBox, QFileDialog, QSpinBox,
    QProgressBar
)

  # GUI는 기능 모듈의 "공통 캡처 로직"을 그대로 사용합니다.
from ft4222h_spi_data_streaming_slave_function import (
    list_ft4222_devices, CaptureSettings, SpiCapture, delete_existing_frame_files
)


class CaptureWorker(QObject): 
    """
    [중요] 캡처 작업을 수행하는 Worker(QObject)

    - 이 객체는 "별도 스레드(QThread)"에서 실행됩니다.
    - SpiCapture는 내부에서 GPIO wait / SPI read 같은 블로킹 동작을 수행할 수 있으므로,
      GUI 스레드에서 직접 실행하면 창이 멈춥니다.
    - Worker는 SpiCapture의 콜백(callback)을 받아서 Qt 시그널(signal)로 변환해 UI로 전달합니다.

    사용 흐름(요약): 
      MainWindow에서 QThread 생성
        -> CaptureWorker(settings) 생성
        -> worker.moveToThread(thread)
        -> thread.started.connect(worker.start)
        -> worker의 시그널들을 MainWindow 슬롯에 connect
        -> thread.start()
    """

                                     # ---- Worker -> UI 로 전달할 시그널들 ----
    status   = pyqtSignal(str)       # 상태/로그 메시지
    error    = pyqtSignal(str)       # 오류 메시지(치명적일 때 QMessageBox로 표시)
    progress = pyqtSignal(int, int)  # 진행률(현재, 전체)
    preview  = pyqtSignal(bytes)     # 프리뷰용 바이트 데이터
    finished = pyqtSignal()          # 작업 종료 알림

    def __init__(self, settings: CaptureSettings): 
        super().__init__()
                     self.settings         = settings
        self.capture: Optional[SpiCapture] = None

    @pyqtSlot()
    def start(self): 
        """
        QThread가 시작될 때 호출되는 엔트리 포인트입니다.

        - 여기서는 SpiCapture를 생성하고 start()를 호출합니다.
        - SpiCapture는 "콜백 기반" 구조이므로, 콜백을 정의한 뒤 Worker 시그널로 연결합니다.
        """

          # SpiCapture 콜백 -> Worker 시그널로 브릿지(bridge)
        def status_cb(msg: str): 
            self.status.emit(msg)

        def error_cb(msg: str): 
            self.error.emit(msg)

        def progress_cb(current: int, total: int): 
            self.progress.emit(current, total)

        def preview_cb(data: bytes): 
            self.preview.emit(data)

        def finished_cb(): 
            self.finished.emit()

          # 실제 캡처 객체 생성 후 실행
        self.capture = SpiCapture(
            self.settings,
            status_callback   = status_cb,
            error_callback    = error_cb,
            progress_callback = progress_cb,
            preview_callback  = preview_cb,
            finished_callback = finished_cb,
        )
        self.capture.start()

    @pyqtSlot()
    def stop(self): 
        """
        캡처 중단 요청.

        주의: 
        - stop()은 즉시 강제 종료가 아니라, SpiCapture 내부 루프가 다음 체크 지점에서
          종료하도록 "플래그"를 내려주는 방식입니다.
        - 따라서 현재 read/gpio_wait 중이면 timeout 이후에 멈출 수 있습니다.
        """
        if self.capture: 
            self.capture.stop()


class MainWindow(QMainWindow): 
    """
    메인 GUI 윈도우.

    - 위젯들을 만들고(Layout 포함), 사용자 입력을 받아 CaptureSettings를 구성합니다.
    - 캡처 시작 시 QThread + CaptureWorker를 띄우고, 시그널을 받아 UI를 갱신합니다.
    """

    def __init__(self): 
        super().__init__()
        self.setWindowTitle("AWRL6844 SPI Data Capture Tool (FT4222H SPI SLAVE)")
        self.resize(1500, 900)

          # 캡처 스레드/워커(캡처 중일 때만 생성)
        self.capture_thread: Optional[QThread]       = None
        self.capture_worker: Optional[CaptureWorker] = None

        self._build_ui()
        self._refresh_devices()

      # ---------------- UI 생성 ----------------
    def _build_ui(self): 
        """
        화면에 표시할 위젯들을 생성하고 레이아웃을 배치합니다.

        PyQt에서 레이아웃 구조: 
          중앙 위젯(central widget)
            -> 최상위 레이아웃(root: QVBoxLayout)
              -> 여러 그룹 박스(QGroupBox)와 버튼/입력 위젯들을 추가
        """
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

          # ===================== Device selection =====================
          # FT4222 장치를 새로 스캔하고, SPI용/ GPIO용 장치를 콤보박스로 선택하도록 합니다.
        dev_group  = QGroupBox("FT4222H Device Selection")
        dev_layout = QHBoxLayout(dev_group)

        self.btn_refresh = QPushButton("Refresh")
          # clicked 시그널 -> _refresh_devices 슬롯(함수) 연결
        self.btn_refresh.clicked.connect(self._refresh_devices)

        self.spi_combo  = QComboBox()
        self.gpio_combo = QComboBox()

        dev_layout.addWidget(self.btn_refresh)
        dev_layout.addWidget(QLabel("SPI Device:"))
        dev_layout.addWidget(self.spi_combo, 2)
        dev_layout.addWidget(QLabel("GPIO Device:"))
        dev_layout.addWidget(self.gpio_combo, 2)
        root.addWidget(dev_group)

          # ===================== Mode info =====================
        mode_group  = QGroupBox("SPI Slave Mode Info")
        mode_layout = QHBoxLayout(mode_group)
        lbl         = QLabel("FT4222H = SPI SLAVE  |  AWRL6844 = SPI MASTER (drives SCLK/CS)")
        lbl.setStyleSheet("font-weight: bold; color: #0066cc;")
        mode_layout.addWidget(lbl)
        mode_layout.addStretch(1)
        root.addWidget(mode_group)

          # ===================== HOST_INTR 설정 =====================
          # 레이더(마스터) 쪽에서 프레임 준비/완료 신호를 GPIO로 알려주는 경우를 가정합니다.
        intr_group  = QGroupBox("HOST_INTR GPIO Configuration")
        intr_layout = QHBoxLayout(intr_group)

        self.intr_port_combo = QComboBox()
        self.intr_port_combo.addItems(["P0", "P1", "P2", "P3"])
        self.intr_port_combo.setCurrentIndex(3)  # 기본 P3

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

          # ===================== Capture Options =====================
        opt_group  = QGroupBox("Capture Options")
        opt_layout = QHBoxLayout(opt_group)

        self.spi_mode_combo = QComboBox()
        self.spi_mode_combo.addItems(["0", "1", "2", "3"])
        self.spi_mode_combo.setCurrentText("0")

        self.initex_combo = QComboBox()
        self.initex_combo.addItems(["Auto (1,2,0)", "0", "1", "2"])
        self.initex_combo.setCurrentIndex(0)

        self.spin_frame_size = QSpinBox()
        self.spin_frame_size.setRange(1024, 1024 * 1024)
        self.spin_frame_size.setValue(65536)
        self.spin_frame_size.setSingleStep(1024)

        self.spin_frames = QSpinBox()
        self.spin_frames.setRange(0, 1000000)
        self.spin_frames.setValue(30)
        self.spin_frames.setSpecialValueText("Infinite")  # 값이 0이면 무한

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

          # ===================== Slave-specific options =====================
        slave_group  = QGroupBox("Slave Mode Options")
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

          # ===================== Output folder =====================
        file_group  = QGroupBox("Output Folder (per-frame files)")
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

          # ===================== Control =====================
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

          # ===================== Log =====================
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

      # ---------------- 유틸: 로그 출력 ----------------
    def _log(self, msg: str) -> None: 
        """
        로그 창(QPlainTextEdit)에 메시지를 추가하고, 스크롤을 맨 아래로 유지합니다.

        - appendPlainText()               : 텍스트를 한 줄 추가합니다.
        - textCursor() + movePosition(End): 커서를 끝으로 이동시켜 자동 스크롤처럼 동작합니다.
        """
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]  # HH:MM:SS.mmm
        self.log_view.appendPlainText(f"[{ts}] {msg}")

          # 커서를 끝으로 보내면, 새 로그가 추가될 때 항상 최신 줄이 보입니다.
        c = self.log_view.textCursor()
        c.movePosition(c.MoveOperation.End)
        self.log_view.setTextCursor(c)

      # ---------------- 디바이스 스캔/선택 ----------------
    def _refresh_devices(self): 
        """연결된 FT4222 디바이스 목록을 다시 스캔하여 콤보박스를 갱신합니다."""
        devices = list_ft4222_devices()
        self.spi_combo.clear()
        self.gpio_combo.clear()

          # QComboBox.addItem(label, userData) 패턴:
          # - label: 사용자에게 보이는 문자열
          # - userData: 내부적으로 저장할 데이터(dict) -> currentData()로 꺼냄
        for d in devices: 
            idx    = d["index"]
            desc   = d["desc"]
            serial = d["serial"]
            label  = f"[{idx}] {desc} (SN: {serial})"
            user   = {"index": idx, "desc": desc}
            self.spi_combo.addItem(label, user)
            self.gpio_combo.addItem(label, user)

          # 디바이스가 2개 이상이면 A/B를 분리해서 기본 선택
        if len(devices) > = 2:
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
        """출력 폴더를 선택하는 다이얼로그를 띄웁니다."""
        path = QFileDialog.getExistingDirectory(self, "Select Output Folder", self.out_dir.text())
        if path: 
            self.out_dir.setText(path)

      # ---------------- 캡처 시작/정지 ----------------
    def _start_capture(self): 
        """
        Start Capture 버튼 클릭 시 실행.

        1) 현재 UI 값들을 읽어서 CaptureSettings를 구성
        2) overwrite 옵션이면 기존 파일 삭제
        3) QThread + CaptureWorker 생성/연결
        4) thread.start()로 캡처 실행(별도 스레드)
        """
        if self.spi_combo.count() == 0:
            QMessageBox.warning(self, "Warning", "No FT4222 devices found!")
            return

        spi_ud  = self.spi_combo.currentData()
        gpio_ud = self.gpio_combo.currentData()
        if not isinstance(spi_ud, dict) or not isinstance(gpio_ud, dict): 
            QMessageBox.critical(self, "Error", "Device selection data invalid.")
            return

        out_dir = self.out_dir.text().strip()

        # InitEx 모드는 "Auto"면 None으로 두고, 아니면 0/1/2를 정수로 전달합니다.
        initex_text = self.initex_combo.currentText()
        initex_mode = None
        if initex_text != "Auto (1,2,0)":
            initex_mode = int(initex_text)

        # 기능 모듈의 CaptureSettings에 UI 값을 그대로 매핑합니다.
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

        # UI 버튼 상태 변경 (캡처 중에는 Start 비활성, Stop 활성)
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)

        # 진행바 설정
        self.progress.setValue(0)
        self.progress.setMaximum(settings.num_frames if settings.num_frames > 0 else 0)

        # ---------------- QThread + Worker 구성 ----------------
        # QThread는 "스레드"만 제공하고, 실제 작업 코드는 Worker(QObject)가 수행합니다.
        self.capture_thread = QThread()
        self.capture_worker = CaptureWorker(settings)

        # Worker를 해당 스레드로 옮깁니다(중요: moveToThread).
        self.capture_worker.moveToThread(self.capture_thread)

        # 스레드가 시작되면 Worker.start() 실행
        self.capture_thread.started.connect(self.capture_worker.start)

        # Worker 시그널을 MainWindow 슬롯(함수)으로 연결
        self.capture_worker.status.connect(self._on_status)
        self.capture_worker.error.connect(self._on_error)
        self.capture_worker.progress.connect(self._on_progress)
        self.capture_worker.preview.connect(self._on_preview)
        self.capture_worker.finished.connect(self._on_finished)

        # 이제 캡처 스레드 시작(비동기 시작)
        self.capture_thread.start()

    def _stop_capture(self):
        """Stop 버튼 클릭 시: Worker에 중단 요청을 전달합니다."""
        if self.capture_worker:
            self.capture_worker.stop()
        self._log("Stop requested...")

    # ---------------- Worker 시그널 처리(= UI 업데이트) ----------------
    def _on_status(self, msg: str):
        """Worker.status 시그널을 받으면 로그에 추가하고 status bar를 갱신합니다."""
        self.status_label.setText(msg[:100])
        self._log(msg)

    def _on_error(self, msg: str):
        """Worker.error 시그널: 로그 출력 + 팝업 표시."""
        self._log(f"ERROR: {msg}")
        QMessageBox.critical(self, "Error", msg)

    def _on_progress(self, current: int, total: int):
        """Worker.progress 시그널: 진행바를 갱신합니다."""
        if total > 0:
            self.progress.setMaximum(total)
            self.progress.setValue(current)
        else:
            # 무한 캡처(Frames=0)인 경우, 단순히 값만 변화시키는 형태
            self.progress.setValue(current % 100)

    def _on_preview(self, data: bytes):
        """Worker.preview 시그널: 프리뷰 바이트를 HEX 문자열로 출력합니다."""
        hex_str = " ".join(f"{b:02X}" for b in data)
        self._log(f"[PREVIEW] {hex_str}")

    def _on_finished(self):
        """
        Worker.finished 시그널: 캡처 종료 시 호출됩니다.

        - thread.quit(): 스레드의 이벤트 루프 종료 요청
        - thread.wait(): 스레드가 완전히 종료될 때까지 잠시 대기
        """
        if self.capture_thread:
            self.capture_thread.quit()
            self.capture_thread.wait(2000)

        self.capture_thread = None
        self.capture_worker = None
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self._log("Capture finished")


def main():
    """PyQt 앱 실행 엔트리."""
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
