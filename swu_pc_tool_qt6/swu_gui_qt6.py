#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
swu_gui_qt6.py

SWU(Software Update) 전용 PC Tool - PyQt6 GUI
- Control/ACK: Single CAN FD frame (MSG ID 0x32 / 0x50)
- DATA: CustomTP multi-frame over CAN FD (MSG ID 0x33), processed after reassembly on Target

Progress mapping (요구사항 반영):
- ACK_SWU_START        : 5%
- ACK_DOWNLOAD_START   : 20%
- DATA (block progress): 20% ~ 95%
- ACK_VERIFY           : 100%

Scenario update:
- SWU_REQUEST includes ModelID, SWVersion.
- Target(APP) may reject SWU_REQUEST with FailReason:
    * VERSION_MATCH  (already up-to-date)
    * MODEL_MISMATCH (wrong target model)

Requirements:
  pip install PyQt6

Run:
  python swu_gui_qt6.py
"""

from __future__ import annotations

import os
import sys
import math
import zlib
from dataclasses import dataclass
from typing import Optional

from PyQt6.QtCore import Qt, QThread, pyqtSignal
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QFormLayout, QPushButton, QLabel, QLineEdit,
    QSpinBox, QComboBox, QFileDialog, QProgressBar, QTextEdit,
    QMessageBox
)

from pcan_manager import PCANManager
from swu_pc_tool import SWUTool, RetryPolicy
from swu_protocol import FailReason

from swu_constants import (
    DEFAULT_PCAN_CHANNEL,
    DEFAULT_BITRATE_FD,
    DEFAULT_IFG_US,
    TIMEOUT_ACK_DATA_S,
    TIMEOUT_ACK_DOWNLOAD_START_S,
    RETRY_SWU_REQUEST_ATTEMPTS,
    RETRY_SWU_REQUEST_INTERVAL_S,
    RETRY_CTRL_ATTEMPTS,
    RETRY_CTRL_INTERVAL_S,
    RETRY_DATA_BLOCK_ATTEMPTS,
    BLOCK_SIZE_MIN,
    BLOCK_SIZE_MAX,
    BLOCK_SIZE_DEFAULT,
    PROGRESS_START,
    PROGRESS_ACK_SWU_START,
    PROGRESS_ACK_DOWNLOAD_START,
    PROGRESS_DATA_START,
    PROGRESS_DATA_END,
    PROGRESS_ACK_VERIFY,
)

from swu_utils import (
    parse_int_auto,
    extract_sw_version_from_filename,
    format_fail_reason,
)

from swu_exceptions import (
    SWUError,
    SWUVersionMatchError,
    SWUModelMismatchError,
    SWUCancelledError,
    SWUBlockTransferError,
    SWUTimeoutError,
    PCANError,
)

from swu_logging import setup_logging, get_logger, GUILogHandler


@dataclass
class SWUParams:
    bin_path: str
    model_id: int
    sw_version: int
    date_yyyymmdd: int
    block_size: int

    channel: str
    bitrate_fd: str
    ifg_us: int

    ack_req_interval_s: float = RETRY_SWU_REQUEST_INTERVAL_S
    ack_req_attempts: int = RETRY_SWU_REQUEST_ATTEMPTS

    ack_ctrl_interval_s: float = RETRY_CTRL_INTERVAL_S
    ack_ctrl_attempts: int = RETRY_CTRL_ATTEMPTS

    # DOWNLOAD_START는 Target에서 flash erase를 수행할 수 있어 ACK가 늦게 올 수 있습니다.
    ack_download_timeout_s: float = TIMEOUT_ACK_DOWNLOAD_START_S

    ack_data_timeout_s: float = TIMEOUT_ACK_DATA_S
    max_data_retries: int = RETRY_DATA_BLOCK_ATTEMPTS


class SWUWorker(QThread):
    progress = pyqtSignal(int)          # 0..100
    status = pyqtSignal(str)            # short status line
    log = pyqtSignal(str)               # detailed log
    finished = pyqtSignal(bool, str)    # success, message

    def __init__(self, params: SWUParams):
        super().__init__()
        self.params = params
        self._cancel = False
        self._mgr: Optional[PCANManager] = None
        self._tool: Optional[SWUTool] = None
        self._image_size: int = 0
        self._total_blocks: int = 0
        self._crc_prev: int = 0

    def request_cancel(self) -> None:
        self._cancel = True

    def _check_cancel(self) -> None:
        if self._cancel:
            raise SWUCancelledError()

    def _setup_connection(self) -> None:
        """PCAN 연결 설정"""
        self.progress.emit(PROGRESS_START)
        self.status.emit("Opening PCAN...")
        self.log.emit(f"[PCAN] open channel={self.params.channel}, ifg_us={self.params.ifg_us}")

        self._mgr = PCANManager()
        self._mgr.open(
            channel=self.params.channel,
            bitrate_fd=self.params.bitrate_fd,
            is_std=True,
            use_brs=True,
            ifg_us=int(self.params.ifg_us),
        )
        self._mgr.start_rx()

        self._tool = SWUTool(
            self._mgr,
            data_ack_timeout_s=float(self.params.ack_data_timeout_s),
            max_block_retries=int(self.params.max_data_retries),
        )

    def _phase_swu_request(self) -> bool:
        """SWU_REQUEST 단계

        Returns:
            True: 계속 진행, False: 업데이트 불필요
        """
        self._check_cancel()
        self.status.emit("SWU_REQUEST...")

        ok, fr = self._tool.swu_request(
            self.params.model_id,
            self.params.sw_version,
            RetryPolicy(
                interval_s=self.params.ack_req_interval_s,
                attempts=self.params.ack_req_attempts
            )
        )

        if not ok:
            if fr == int(FailReason.VERSION_MATCH):
                self.log.emit("[INFO] SWU_REQUEST rejected: VERSION_MATCH (already up-to-date)")
                self.status.emit("No Update Needed")
                return False
            if fr == int(FailReason.MODEL_MISMATCH):
                raise SWUModelMismatchError()
            raise SWUError(f"SWU_REQUEST 실패/거절: {format_fail_reason(fr)}", fr)

        self.log.emit("[OK] SWU_REQUEST / ACK_SWU_REQUEST")
        return True

    def _phase_swu_start(self) -> None:
        """SWU_START 단계"""
        self._check_cancel()
        self._image_size = os.path.getsize(self.params.bin_path)
        self.status.emit("SWU_START...")

        ok = self._tool.swu_start(
            self.params.model_id,
            self.params.sw_version,
            self.params.date_yyyymmdd,
            self._image_size,
            self.params.block_size,
            RetryPolicy(
                interval_s=self.params.ack_ctrl_interval_s,
                attempts=self.params.ack_ctrl_attempts
            ),
        )
        if not ok:
            raise SWUTimeoutError("SWU_START", self.params.ack_ctrl_interval_s * self.params.ack_ctrl_attempts)

        self.log.emit("[OK] SWU_START / ACK_SWU_START")
        self.progress.emit(PROGRESS_ACK_SWU_START)

    def _phase_download_start(self) -> None:
        """DOWNLOAD_START 단계"""
        self._check_cancel()
        self.status.emit("DOWNLOAD_START...")

        ok = self._tool.download_start(
            RetryPolicy(interval_s=self.params.ack_download_timeout_s, attempts=1)
        )
        if not ok:
            raise SWUTimeoutError("DOWNLOAD_START", self.params.ack_download_timeout_s)

        self.log.emit("[OK] DOWNLOAD_START / ACK_DOWNLOAD_START")
        self.progress.emit(PROGRESS_ACK_DOWNLOAD_START)

    def _transfer_single_block(self, block_idx: int, data: bytes, crc_next: int) -> None:
        """단일 블록 전송

        Args:
            block_idx: 블록 인덱스 (1-based)
            data: 블록 데이터
            crc_next: 예상 CRC
        """
        self.status.emit(f"Downloading... Block {block_idx}/{self._total_blocks}")

        ok, fail_reason = self._tool.send_block(
            block_idx,
            crc_next,
            data,
            ack_timeout_s=float(self.params.ack_data_timeout_s),
            max_retries=int(self.params.max_data_retries),
        )

        if not ok:
            self.log.emit(f"[FAIL] Block {block_idx}: {format_fail_reason(fail_reason)}")
            raise SWUBlockTransferError(block_idx, self._total_blocks, fail_reason)

        self.log.emit(f"[OK] Block {block_idx}/{self._total_blocks} CRC=0x{crc_next:08X}")

    def _phase_data_transfer(self) -> None:
        """데이터 블록 전송 루프"""
        self._total_blocks = int(math.ceil(self._image_size / self.params.block_size)) if self._image_size > 0 else 0
        self.log.emit(
            f"[INFO] ImageSize={self._image_size} bytes, BlockSize={self.params.block_size}, TotalBlocks={self._total_blocks}"
        )

        self._crc_prev = 0
        sent_blocks = 0

        with open(self.params.bin_path, "rb") as f:
            for block_idx in range(1, self._total_blocks + 1):
                self._check_cancel()

                offset = (block_idx - 1) * self.params.block_size
                f.seek(offset, os.SEEK_SET)
                data = f.read(min(self.params.block_size, self._image_size - offset))

                # streaming CRC (payload only)
                crc_next = zlib.crc32(data, self._crc_prev) & 0xFFFFFFFF

                self._transfer_single_block(block_idx, data, crc_next)

                # success
                self._crc_prev = crc_next
                sent_blocks += 1

                # Progress: DATA = 20% ~ 95%
                if self._total_blocks > 0:
                    data_prog = PROGRESS_DATA_START + int(
                        round((sent_blocks / self._total_blocks) * (PROGRESS_DATA_END - PROGRESS_DATA_START))
                    )
                else:
                    data_prog = PROGRESS_DATA_END

                data_prog = max(PROGRESS_DATA_START, min(PROGRESS_DATA_END, data_prog))
                self.progress.emit(data_prog)

    def _phase_verify(self) -> None:
        """VERIFY_IMAGE 단계"""
        self._check_cancel()
        self.status.emit("VERIFY_IMAGE...")

        ok, fail_reason = self._tool.verify_image(
            self._crc_prev,
            RetryPolicy(
                interval_s=self.params.ack_ctrl_interval_s,
                attempts=self.params.ack_ctrl_attempts
            ),
        )
        if not ok:
            self.log.emit(f"[FAIL] VERIFY_IMAGE: {format_fail_reason(fail_reason)}")
            raise SWUError(f"VERIFY_IMAGE 실패: {format_fail_reason(fail_reason)}", fail_reason)

        self.log.emit(f"[OK] VERIFY_IMAGE / ACK_VERIFY (FinalCRC=0x{self._crc_prev:08X})")
        self.progress.emit(PROGRESS_ACK_VERIFY)

    def run(self) -> None:
        """SWU 시퀀스 오케스트레이션"""
        try:
            # Phase 0: PCAN 연결
            self._setup_connection()

            # Phase 1: SWU_REQUEST
            if not self._phase_swu_request():
                # VERSION_MATCH - 업데이트 불필요
                self.finished.emit(True, "이미 동일/최신 SW Version 입니다. 업데이트가 필요 없습니다.")
                return

            # Phase 2: SWU_START
            self._phase_swu_start()

            # Phase 3: DOWNLOAD_START
            self._phase_download_start()

            # Phase 4: DATA blocks
            self._phase_data_transfer()

            # Phase 5: VERIFY_IMAGE
            self._phase_verify()

            self.status.emit("Done")
            self.finished.emit(True, f"SWU 성공 (FinalCRC=0x{self._crc_prev:08X})")

        except SWUCancelledError:
            self.status.emit("Cancelled")
            self.finished.emit(False, "사용자에 의해 중단되었습니다.")

        except SWUVersionMatchError:
            self.status.emit("No Update Needed")
            self.finished.emit(True, "이미 동일/최신 SW Version 입니다. 업데이트가 필요 없습니다.")

        except (SWUError, PCANError) as e:
            self.status.emit("Error")
            self.finished.emit(False, str(e))

        except Exception as e:
            self.status.emit("Error")
            self.finished.emit(False, str(e))

        finally:
            if self._mgr:
                try:
                    self._mgr.close()
                except Exception:
                    pass


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SWU PC Tool (CustomTP) - PyQt6")

        self.worker: Optional[SWUWorker] = None
        self.bin_path: Optional[str] = None
        self.bin_size: int = 0

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # ---- Connection group ----
        grp_conn = QGroupBox("PCAN Connection")
        form_conn = QFormLayout(grp_conn)

        self.cmb_channel = QComboBox()
        for i in range(1, 9):
            self.cmb_channel.addItem(f"PCAN_USBBUS{i}")
        self.cmb_channel.setCurrentText(DEFAULT_PCAN_CHANNEL)

        self.ed_bitrate = QLineEdit(DEFAULT_BITRATE_FD)

        self.sp_ifg = QSpinBox()
        self.sp_ifg.setRange(0, 100000)
        self.sp_ifg.setValue(DEFAULT_IFG_US)
        self.sp_ifg.setSuffix(" us")

        form_conn.addRow("Channel", self.cmb_channel)
        form_conn.addRow("Bitrate FD", self.ed_bitrate)
        form_conn.addRow("IFG", self.sp_ifg)
        root.addWidget(grp_conn)

        # ---- SWU Params group ----
        grp_params = QGroupBox("SWU Parameters")
        form_params = QFormLayout(grp_params)

        # File select
        file_row = QHBoxLayout()
        self.ed_file = QLineEdit()
        self.ed_file.setReadOnly(True)

        self.btn_browse = QPushButton("Load File...")
        self.btn_browse.clicked.connect(self.on_browse)

        file_row.addWidget(self.ed_file, 1)
        file_row.addWidget(self.btn_browse)

        w_file = QWidget()
        w_file.setLayout(file_row)

        self.lb_size = QLabel("-")
        self.ed_model = QLineEdit("0x68440001")
        self.ed_swver = QLineEdit("0x00010001")

        import datetime as _dt
        today = _dt.datetime.now().strftime("%Y%m%d")
        self.ed_date = QLineEdit(today)

        self.sp_block = QSpinBox()
        self.sp_block.setRange(BLOCK_SIZE_MIN, BLOCK_SIZE_MAX)
        self.sp_block.setSingleStep(4096)
        self.sp_block.setValue(BLOCK_SIZE_DEFAULT)
        self.sp_block.setSuffix(" bytes")

        self.lb_total_blocks = QLabel("-")

        self.sp_ack_data_timeout = QSpinBox()
        self.sp_ack_data_timeout.setRange(1, 60)
        self.sp_ack_data_timeout.setValue(int(TIMEOUT_ACK_DATA_S))
        self.sp_ack_data_timeout.setSuffix(" s")

        self.sp_ack_download_timeout = QSpinBox()
        self.sp_ack_download_timeout.setRange(1, 300)
        self.sp_ack_download_timeout.setValue(int(TIMEOUT_ACK_DOWNLOAD_START_S))
        self.sp_ack_download_timeout.setSuffix(" s")

        form_params.addRow("Firmware file", w_file)
        form_params.addRow("File size", self.lb_size)
        form_params.addRow("Model ID", self.ed_model)
        form_params.addRow("SW Version", self.ed_swver)
        form_params.addRow("Date (YYYYMMDD)", self.ed_date)
        form_params.addRow("TransferBlockSize", self.sp_block)
        form_params.addRow("TotalBlocks", self.lb_total_blocks)
        form_params.addRow("ACK_DATA timeout", self.sp_ack_data_timeout)
        form_params.addRow("ACK_DOWNLOAD_START timeout", self.sp_ack_download_timeout)

        root.addWidget(grp_params)

        # ---- Controls ----
        ctrl_row = QHBoxLayout()
        self.btn_start = QPushButton("START")
        self.btn_start.clicked.connect(self.on_start)

        self.btn_stop = QPushButton("STOP")
        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_stop.setEnabled(False)

        ctrl_row.addWidget(self.btn_start)
        ctrl_row.addWidget(self.btn_stop)
        root.addLayout(ctrl_row)

        # ---- Progress ----
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        root.addWidget(self.progress)

        # ---- Status / Log ----
        self.lb_status = QLabel("Idle")
        self.lb_status.setAlignment(Qt.AlignmentFlag.AlignLeft)
        root.addWidget(self.lb_status)

        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        root.addWidget(self.log_view, 1)

        # dynamic updates
        self.sp_block.valueChanged.connect(self.update_block_info)

        self.resize(900, 650)

    def append_log(self, s: str) -> None:
        self.log_view.append(s)

    def update_block_info(self) -> None:
        if not self.bin_path or self.bin_size <= 0:
            self.lb_total_blocks.setText("-")
            return
        bs = int(self.sp_block.value())
        total = int(math.ceil(self.bin_size / bs)) if bs > 0 else 0
        self.lb_total_blocks.setText(str(total))

    def on_browse(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Select firmware image", "", "Binary files (*.bin *.img *.appimage *.*)"
        )
        if not path:
            return
        self.bin_path = path
        self.ed_file.setText(path)

        # Auto-detect SW Version from filename (e.g., _0x00010001.release.appimage)
        auto_swver = extract_sw_version_from_filename(path)
        if auto_swver:
            self.ed_swver.setText(auto_swver)

        try:
            self.bin_size = os.path.getsize(path)
            self.lb_size.setText(f"{self.bin_size} bytes")
        except Exception:
            self.bin_size = 0
            self.lb_size.setText("N/A")

        self.update_block_info()

    def set_running(self, running: bool) -> None:
        self.btn_start.setEnabled(not running)
        self.btn_browse.setEnabled(not running)

        self.sp_block.setEnabled(not running)
        self.ed_model.setEnabled(not running)
        self.ed_swver.setEnabled(not running)
        self.ed_date.setEnabled(not running)

        self.cmb_channel.setEnabled(not running)
        self.ed_bitrate.setEnabled(not running)
        self.sp_ifg.setEnabled(not running)

        self.sp_ack_data_timeout.setEnabled(not running)
        self.sp_ack_download_timeout.setEnabled(not running)

        self.btn_stop.setEnabled(running)

    def on_start(self) -> None:
        if not self.bin_path:
            QMessageBox.warning(self, "SWU", "먼저 firmware 파일을 Load 하십시오.")
            return

        try:
            params = SWUParams(
                bin_path=self.bin_path,
                model_id=parse_int_auto(self.ed_model.text()),
                sw_version=parse_int_auto(self.ed_swver.text()),
                date_yyyymmdd=parse_int_auto(self.ed_date.text()),
                block_size=int(self.sp_block.value()),
                ack_download_timeout_s=float(self.sp_ack_download_timeout.value()),
                channel=self.cmb_channel.currentText(),
                bitrate_fd=self.ed_bitrate.text().strip(),
                ifg_us=int(self.sp_ifg.value()),
                ack_data_timeout_s=float(self.sp_ack_data_timeout.value()),
            )
        except Exception as e:
            QMessageBox.critical(self, "SWU", f"파라미터 파싱 오류: {e}")
            return

        self.log_view.clear()
        self.progress.setValue(0)
        self.lb_status.setText("Starting...")
        self.set_running(True)

        self.worker = SWUWorker(params)
        self.worker.progress.connect(self.progress.setValue)
        self.worker.status.connect(self.lb_status.setText)
        self.worker.log.connect(self.append_log)
        self.worker.finished.connect(self.on_finished)
        self.worker.start()

    def on_stop(self) -> None:
        if self.worker and self.worker.isRunning():
            self.worker.request_cancel()
            self.append_log("[INFO] Cancel requested...")
            self.lb_status.setText("Stopping...")

    def on_finished(self, success: bool, message: str) -> None:
        self.set_running(False)

        if success:
            self.lb_status.setText("Success")
            QMessageBox.information(self, "SWU", message)
        else:
            self.lb_status.setText("Failed")
            QMessageBox.critical(self, "SWU", message)

        self.append_log(f"[DONE] {message}")
        self.worker = None


def main() -> int:
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
