
  #!/usr/bin/env python3
  # -*- coding: utf-8 -*-
"""
can_custom_tp_manager_gui.py (App PDU aware, 2-byte header, 14-bit SEQ)

- PyQt6 GUI for CAN CLI
- TX: prefers CanCustomTpSender.send_app_pdu(msg_id:int, payload:bytes)
      falls back to local framer (HDR + APP_PDU) if sender lacks that API
- RX: CanCustomTpReceiver assembles complete APP_PDUs; GUI interprets them as
      [Len(4B, LE)] + [MsgID(1B)] + Payload and shows MsgID/Payload separately
- Every received complete APP_PDU is saved to a timestamped log file
  ("radar packet YYYY-MM-DD HH-mm-ss.log").

Application Layer            : Application PDU = Len(4B, little-endian) + MsgID(1B) + Payload
Transport   Layer (Custom‑TP): HDR(2B) + APP_PDU( Msg ID(1B) + Payload )
HDR        (2B)              : 
            HDR1             : bit7=1 → LAST, bit7=0 → MIDDLE
bit6       =0 (reserved),      bits5..0     : SEQ high (6 bits)
            HDR2             : SEQ low (8 bits)
       SEQ             = (HDR1[5:0] << 8) | HDR2  (14 bits, 0..16383)
  DATA:    frame[2:64] = 62B

pip install PyQt6
"""

from __future__ import annotations
from typing import List, Optional, Set, Tuple, Callable
import sys, os, re, binascii, queue
from datetime import datetime

  # Add parent directory to path for cross-package imports
_parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _parent_dir not in sys.path: 
    sys.path.insert(0, _parent_dir)

from PyQt6.QtCore import Qt, QThread, pyqtSignal, QObject, QEvent
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFileDialog, QMessageBox,
    QTextEdit, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QCheckBox, QProgressBar, QSplitter, QListWidget, QListWidgetItem, QGroupBox, QSpinBox
)

import glob

  # -------- Backend (TX) --------
try: 
    from can_customtp.can_custom_tp_sender import CanCustomTpSender
    _be_err = None
except Exception as e: 
    CanCustomTpSender = None  # type: ignore
    _be_err           = e

  # -------- Receiver (RX) --------
try: 
    from can_customtp.can_custom_tp_receiver import CanCustomTpReceiver
    _rx_err = None
except Exception as e: 
    CanCustomTpReceiver = None  # type: ignore
    _rx_err             = e

  # -------- PCANManager (fallback용) --------
try: 
    from can_common.pcan_manager import PCANManager
except Exception: 
    PCANManager = None  # type: ignore


                         # ========= Transport constants (14-bit SEQ) =========
CHUNK_DATA_MAX = 62      # bytes per frame after 2-byte HDR
HDR_LAST_BIT   = 0x80    # bit7
SEQ_HIGH_MASK  = 0x3F    # 6 bits for SEQ high
MAX_SEQ        = 0x3FFF  # 14-bit sequence number


  # ================= 유틸: 필터 파서 =================
def _parse_filter_ids(text: str) -> Optional[Set[int]]: 
    s = (text or "").strip()
    if not s: 
        return None
        out                          : Set[int] = set()
    for tok in re.split(r"[,\s]+", s): 
    if  not tok                      : 
            continue
        if "-" in tok: 
            a, b = tok.split("-", 1)
              lo = int(a, 0); hi = int(b, 0)
            if lo > hi: 
                lo, hi = hi, lo
            out.update(range(lo, hi + 1))
        else: 
            out.add(int(tok, 0))
    return out or None


def extract_payload_from_app_pdu(app_pdu: bytes): 
    """
    APP_PDU = [ Len(4B LE) | MSG_ID(1B) | payload(...) | optional padding(...) ]

    - Length: 실제 payload 길이 (바이트)
    - 새 포맷(Length 포함)에서는 Length만 기준으로 payload를 자른다.
    return: 
        msg_id, payload(순수데이터), payload_len
    """
    if not app_pdu: 
        return 0, b"", 0

      # 새 포맷: Len(4B LE) + MsgID(1B) + Payload
    if len(app_pdu) > = 5:
       length         = (
            int(app_pdu[0])
            | (int(app_pdu[1]) << 8)
            | (int(app_pdu[2]) << 16)
            | (int(app_pdu[3]) << 24)
        )
        msg_id = app_pdu[4]

        if length < 0: 
            length = 0

        raw = app_pdu[5:]  # 패딩 제거 안 함, 길이만 신뢰

        if length < = len(raw):
           payload  = raw[:length]
        else: 
              # 선언된 길이가 실제 데이터보다 크면, 있는 만큼만 사용
            payload = raw
            length  = len(payload)

        payload_len = length

      # 구버전 포맷: [MsgID][Payload...]  (fallback)
    elif len(app_pdu) > = 1:
         msg_id         = app_pdu[0]
         payload        = app_pdu[1:].rstrip(b"\x00")
         payload_len    = len(payload)
    else: 
        return 0, b"", 0

    return msg_id, payload, payload_len


  # ================= 송신 작업 스레드 =================
class SendWorker(QThread): 
    progress   = pyqtSignal(int)
    lineSent   = pyqtSignal(str)
    error      = pyqtSignal(str)
    finishedOk = pyqtSignal()

    def __init__(self, send_fn: Callable[[str], None], lines: List[str]): 
        super().__init__()
        self._send_fn = send_fn
        self.lines    = lines
        self._stop    = False

    def run(self): 
        try      : 
            total = len(self.lines)
            for i,          line in enumerate(self.lines, 1): 
            if  self._stop: 
                    break
                self._send_fn(line)
                self.lineSent.emit(line)
                self.progress.emit(int(i * 100 / max(1, total)))
            self.finishedOk.emit()
        except Exception as e: 
            self.error.emit(str(e))

    def stop(self): 
        self._stop = True


  # ================= 파일 저장 전용 워커 =================
class PacketLoggerWorker(QThread): 
    error       = pyqtSignal(str)
    droppedWarn = pyqtSignal(int)  # 누적 드롭 수 알림

    def __init__(self, path: str, flush_every: int = 20): 
        super().__init__()
                self._path                        = path
        self._q: "queue.Queue[tuple[int, bytes]]" = queue.Queue(maxsize=4096)
                self._stop                        = False
                self._file                        = None
                self._flush_every                 = max(1, flush_every)
                self._write_count                 = 0
                self._dropped                     = 0

    def enqueue(self, can_id: int, app_pdu: bytes): 
        try                                       : 
            self._q.put_nowait((can_id, app_pdu))
        except queue.Full: 
               self._dropped       += 1
            if self._dropped % 100 == 1:
                self.droppedWarn.emit(self._dropped)

    def run(self): 
        try      : 
            self._file = open(self._path, "a", encoding="ascii", buffering=1)
        except Exception as e: 
            self.error.emit(f"로그 파일 오픈 실패: {e}")
            return

              try           : 
        while not self._stop: 
              try           : 
                    can_id, app_pdu = self._q.get(timeout=0.2)
                except queue.Empty: 
                    continue

                try: 
                      # === 핵심: APP_PDU -> MSG_ID / payload / pdu_len 파싱 ===
                    msg_id, payload, payload_len = extract_payload_from_app_pdu(app_pdu)

                    payload_hex = payload.hex().upper()
                    ts          = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    
                      # === 로그 저장 (To: 형식) ===
                    self._file.write(
                        f"[{ts}] MSG_ID: 0x{msg_id:02X} "
                        f"PAYLOAD_LEN(B): {payload_len}\n"
                    )
                    self._file.write(f"PAYLOAD: {payload_hex}\n\n")

                    self._write_count                          += 1
                    if (self._write_count % self._flush_every) == 0:
                        self._file.flush()

                except Exception as e: 
                    self.error.emit(f"로그 파일 쓰기 실패: {e}")

           finally   : 
           try       : 
        if self._file: 
                    self._file.flush()
                    self._file.close()
            except Exception: 
                pass


    def stop(self): 
        self._stop = True


  # ================= 폴링 수신 워커 (콜백 실패 시 폴백) =================
class PollWorker(QThread): 
    rxAppPdu = pyqtSignal(bytes, int)  # (app_pdu, can_id)
    error    = pyqtSignal(str)

    def __init__(self, receiver: "CanCustomTpReceiver", timeout_s: float): 
        super().__init__()
        self.receiver  = receiver
        self.timeout_s = timeout_s
        self._stop     = False

    def   run(self)     : 
          try           : 
    while not self._stop: 
                pkt = self.receiver.receive_packet(timeout_s=self.timeout_s)
                if pkt is None: 
                    self.msleep(2)
                    continue
                data, can_id = pkt  # data == APP_PDU
                self.rxAppPdu.emit(data, can_id)
        except Exception as e: 
            self.error.emit(str(e))

    def stop(self): 
        self._stop = True


  # ================= UI 브리지(메인 스레드에서만 UI 갱신) =================
class UiBridge(QObject): 
    rx_app_pdu = pyqtSignal(int, object)  # (can_id, bytes)
    log_msg    = pyqtSignal(str)


  # ================= 메인 윈도우 =================
class MainWindow(QMainWindow): 
def   __init__(self)         : 
        super().__init__()
        self.setWindowTitle("CAN Sender/Receiver (Custom-TP 2-byte header, 14-bit SEQ, AppPDU aware) - PyQt6")
        self.resize(1280, 900)

        if CanCustomTpSender is None: 
            QMessageBox.critical(self, "오류", f"백엔드(can_custom_tp_sender) import 실패: {_be_err}")
        if CanCustomTpReceiver is None: 
            QMessageBox.critical(self, "오류", f"수신기(can_custom_tp_receiver) import 실패: {_rx_err}")

        self.txWorker    : Optional[SendWorker]          = None
        self.pollWorker  : Optional[PollWorker]          = None
        self.loggerWorker: Optional[PacketLoggerWorker]  = None
        self.sender      : Optional["CanCustomTpSender"] = None
        self._rx_receiver: Optional[CanCustomTpReceiver] = None

        self._rx_mgr_owned = False
        self._rx_mgr       = None

          # ---- UI 브리지(메인스레드 시그널로만 UI수정) ----
        self.uiBus = UiBridge()
        self.uiBus.rx_app_pdu.connect(self._on_ui_rx_app_pdu)
        self.uiBus.log_msg.connect(self._append_log)

          # ---------------- 상단: 연결/수신 설정 ----------------
        self.edChannel = QLineEdit("PCAN_USBBUS1")
        self.edBitrate = QLineEdit(
            "f_clock=80000000,nom_brp=2,nom_tseg1=33,nom_tseg2=6,nom_sjw=1,"
            "data_brp=2,data_tseg1=6,data_tseg2=1,data_sjw=1,data_ssp_offset=14"
        )
        self.edIfg       = QSpinBox(); self.edIfg.setRange(0, 20000); self.edIfg.setValue(3000)
        self.edFilterIds = QLineEdit("")                                                         # 빈칸=전체
        self.edFilterIds.setPlaceholderText("예: 0xD1, 0x200-0x20F  (빈칸=전체)")
        self.edTimeout = QSpinBox(); self.edTimeout.setRange(1, 120); self.edTimeout.setValue(30)

          # === App Layer: Msg ID (1B) 설정 ===
        self.edMsgId     = QSpinBox(); self.edMsgId.setRange(0, 255); self.edMsgId.setValue(0xFA)
        self.chkAppendLF = QCheckBox("Append LF on send"); self.chkAppendLF.setChecked(True)

        self.btnConnect    = QPushButton("Connect")
        self.btnDisconnect = QPushButton("Disconnect"); self.btnDisconnect.setEnabled(False)

        row = QHBoxLayout()
        row.addWidget(QLabel("Channel")); row.addWidget(self.edChannel)
        row.addSpacing(8)
        row.addWidget(QLabel("BitrateFD")); row.addWidget(self.edBitrate)
        row.addSpacing(8)
        row.addWidget(QLabel("IFG us")); row.addWidget(self.edIfg)
        row.addSpacing(8)
        row.addWidget(QLabel("Filter IDs")); row.addWidget(self.edFilterIds)
        row.addSpacing(8)
        row.addWidget(QLabel("Timeout s")); row.addWidget(self.edTimeout)
        row.addSpacing(8)
        row.addWidget(QLabel("App MsgID")); row.addWidget(self.edMsgId)
        row.addWidget(self.chkAppendLF)
        row.addStretch(1)
        row.addWidget(self.btnConnect); row.addWidget(self.btnDisconnect)

        gbConn  = QGroupBox("Connection (TX: CanCustomTpSender, RX: CanCustomTpReceiver) + App PDU - 2-byte Header (14-bit SEQ)")
        layConn = QVBoxLayout(); layConn.addLayout(row); gbConn.setLayout(layConn)

          # ---------------- 중앙: 좌/우 ----------------
        self.listLines  = QListWidget()
        self.textEditor = QTextEdit()
        self.textEditor.setPlaceholderText("명령을 입력하거나 파일을 로드하세요. (%, 공백 스킵옵션 적용)")

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(self.listLines)
        splitter.addWidget(self.textEditor)
        splitter.setStretchFactor(1, 1)

          # ---------------- 옵션/버튼 ----------------
        self.chkSkipComment = QCheckBox("주석(%) 스킵"); self.chkSkipComment.setChecked(True)
        self.chkSkipEmpty   = QCheckBox("공백 스킵"); self.chkSkipEmpty.setChecked(True)

        self.btnOpen         = QPushButton("파일 열기")
        self.btnSendAll      = QPushButton("전체 전송 (AppPDU)")
        self.btnSendSelected = QPushButton("선택 전송 (AppPDU)")
        self.btnStop         = QPushButton("중지"); self.btnStop.setEnabled(False)
        self.btnClear        = QPushButton("로그 지우기")

        optRow = QHBoxLayout()
        optRow.addWidget(self.chkSkipComment)
        optRow.addWidget(self.chkSkipEmpty)
        optRow.addStretch(1)
        optRow.addWidget(self.btnOpen)
        optRow.addWidget(self.btnSendSelected)
        optRow.addWidget(self.btnSendAll)
        optRow.addWidget(self.btnStop)
        optRow.addWidget(self.btnClear)

          # ---------------- 진행/로그/콘솔 ----------------
        self.progress = QProgressBar(); self.progress.setRange(0, 100)
        self.log      = QTextEdit(); self.log.setReadOnly(True)
        self.console  = QTextEdit(); self.console.setReadOnly(True)
        self.console.setPlaceholderText("완성 App PDU 요약(hex) 표시")
        self.consoleInput = QLineEdit()
        self.consoleInput.setPlaceholderText("단일 명령 입력 (AppPDU로 전송). ↑/↓=히스토리, Ctrl+L=클리어")
        self.hist: List[str] = []; self.hidx: int = 0

          # 메모리/성능 보호: 최대 라인 제한
        self.console.document().setMaximumBlockCount(5000)
        self.log.document().setMaximumBlockCount(2000)

          # ---------------- 레이아웃 ----------------
        central = QWidget(); root = QVBoxLayout(central)
        root.addWidget(gbConn)
        root.addWidget(splitter, 1)
        root.addLayout(optRow)
        root.addWidget(self.progress)
        root.addWidget(self.log, 1)
        root.addWidget(self.console, 1)
        root.addWidget(self.consoleInput)
        self.setCentralWidget(central)

          # ---------------- 시그널 ----------------
        self.btnOpen.clicked.connect(self.on_open)
        self.btnSendAll.clicked.connect(self.on_send_all)
        self.btnSendSelected.clicked.connect(self.on_send_selected)
        self.btnStop.clicked.connect(self.on_stop)
        self.btnClear.clicked.connect(self.log.clear)
        self.btnConnect.clicked.connect(self.on_connect)
        self.btnDisconnect.clicked.connect(self.on_disconnect)

        self.consoleInput.returnPressed.connect(self.on_repl_enter)
        self.consoleInput.installEventFilter(self)

      # ---------- 유틸 ----------
    def _filter_lines(self, lines: List[str]) -> List[str]: 
        out = []
        for s in lines: 
            t = s.rstrip("\r\n")
            if self.chkSkipEmpty.isChecked() and not t.strip(): 
                continue
            if self.chkSkipComment.isChecked() and t.lstrip().startswith("%"): 
                continue
            out.append(t)
        return out

    def _append_log(self, msg: str): 
        self.log.append(msg)

    def _err(self, msg: str): 
        QMessageBox.critical(self, "오류", msg)

      # ---------- 파일 열기 ----------
    def on_open(self): 
        path, _ = QFileDialog.getOpenFileName(
            self, "명령 파일 선택", "", "Text files (*.txt *.cfg *.cmd *.conf);;All files (*.*)"
        )
        if not path: 
            return
             try                                   : 
        with open(path, "r", encoding="utf-8") as f: 
                content = f.read()
        except UnicodeDecodeError                                     : 
        with   open(path, "r", encoding="cp949", errors="ignore") as f: 
                content = f.read()
        self.textEditor.setPlainText(content)
        self.listLines.clear()
        for line in self._filter_lines(content.splitlines()): 
            self.listLines.addItem(QListWidgetItem(line))
        self._append_log(f"[INFO] 파일 로드: {path}")

      # ---------- Connect / Disconnect ----------
    def on_connect(self)                                        : 
    if  CanCustomTpSender is None or CanCustomTpReceiver is None: 
            self._err("백엔드/수신기 import 실패")
            return

        if self.sender is not None or self._rx_receiver is not None or (self.pollWorker is not None): 
            self._append_log("[WARN] 이미 연결되어 있습니다. 먼저 Disconnect 하세요.")
            return

        try: 
              # --- 기존 .log 파일 삭제 ---
            try: 
                log_files = glob.glob("*.log")

                if not log_files: 
                      # 여기서 그냥 삭제만 생략하고 넘어감 (return 하지 않음)
                    self._append_log("[INFO] 기존 .log 파일 없음 → 삭제 생략")
                else: 
                    deleted = 0
                    for f in log_files: 
                        try           : 
                            os.remove(f)
                            deleted += 1
                            self._append_log(f"[INFO] 이전 로그 삭제: {f}")
                        except Exception as e: 
                            self._append_log(f"[WARN] 로그 삭제 실패 ({f}): {e}")
                    self._append_log(f"[INFO] 기존 .log {deleted}개 삭제 완료")
            except Exception as e: 
                self._append_log(f"[WARN] 로그 삭제 중 예외: {e}")

              # TX
            self.sender = CanCustomTpSender(
                channel    = self.edChannel.text().strip(),
                bitrate_fd = self.edBitrate.text().strip(),
            )
              # apply LF preference to sender if available
            try: 
                self.sender.append_lf = bool(self.chkAppendLF.isChecked())
            except Exception: 
                pass
            self.sender.connect(ifg_us=int(self.edIfg.value()))
            self._append_log("[INFO] TX 연결 완료")

              # RX 매니저
            rx_mgr = None
            try: 
                rx_mgr = self.sender._require_mgr()  # type: ignore[attr-defined]
            except Exception: 
                rx_mgr = None
            if rx_mgr is None     : 
            if PCANManager is None: 
                    self._err("PCANManager 로드 실패: RX 매니저 생성 불가")
                    try: 
                        self.sender.disconnect()
                    except Exception: 
                        pass
                    self.sender = None
                    return
                rx_mgr = PCANManager()
                rx_mgr.open(self.edChannel.text().strip(),
                            self.edBitrate.text().strip(),
                            ifg_us=int(self.edIfg.value()))
                self._rx_mgr_owned = True
                self._append_log("[INFO] 별도 RX 매니저 오픈")
            else: 
            self._rx_mgr_owned = False
            self._rx_mgr       = rx_mgr

              # 로그 파일 준비 (':' 대신 '-' 사용: Windows 호환)
            ts                = datetime.now().strftime("%Y-%m-%d %H-%M-%S")
            log_name          = f"radar packet {ts}.log"
            log_path          = os.path.abspath(log_name)
            self.loggerWorker = PacketLoggerWorker(log_path, flush_every=20)
            self.loggerWorker.error.connect(lambda m: self._append_log(f"[LOG-ERR] {m}"))
            self.loggerWorker.droppedWarn.connect(
                lambda n: self._append_log(f"[LOG-WARN] 저장 큐 포화로 {n}개 드롭됨 (수신은 계속 진행)")
            )
            self.loggerWorker.start()
            self._append_log(f"[INFO] 저장 파일: {log_path}")

              # 수신기
            filter_ids        = _parse_filter_ids(self.edFilterIds.text())
            self._rx_receiver = CanCustomTpReceiver(rx_mgr, filter_can_ids=filter_ids)

              # 콜백(백그라운드 스레드) → 메인 스레드 신호만 발생
            def _on_complete_packet(app_pdu: bytes, can_id: int): 
                try                                             : 
            if  self.loggerWorker is not None                   : 
                        self.loggerWorker.enqueue(int(can_id), bytes(app_pdu))
                except Exception: 
                    pass
                try: 
                    self.uiBus.rx_app_pdu.emit(int(can_id), bytes(app_pdu))
                except Exception: 
                    pass

            started_callback = False
            try: 
                self._rx_receiver.start_rx(on_packet=_on_complete_packet)
                started_callback = True
                self._append_log("[INFO] RX: 콜백 기반 연속 수신 시작(start_rx)")
            except Exception as e: 
                self._append_log(f"[WARN] start_rx 실패 → 폴링으로 폴백: {e}")

              # 폴링 폴백 경로
            if not started_callback: 
                self.pollWorker = PollWorker(
                    receiver  = self._rx_receiver,
                    timeout_s = float(self.edTimeout.value())
                )
                self.pollWorker.rxAppPdu.connect(self._on_worker_rx_app_pdu)
                self.pollWorker.error.connect(lambda msg: self.console.append(f"[RX-ERR] {msg}"))
                self.pollWorker.start()
                self._append_log("[INFO] RX: 폴링 기반 연속 수신 시작")

              # UI 상태
            self.btnConnect.setEnabled(False)
            self.btnDisconnect.setEnabled(True)
            self.statusBar().showMessage("Connected")
            self._append_log("[INFO] 전체 연결 및 수신 시작 완료")

        except Exception as e: 
            self._teardown_on_error()
            self._err(str(e))

      # PollWorker → 메인스레드 슬롯
    def _on_worker_rx_app_pdu(self, app_pdu: bytes, can_id: int): 
        try                                                     : 
    if  self.loggerWorker is not None                           : 
                self.loggerWorker.enqueue(int(can_id), bytes(app_pdu))
        except Exception: 
            pass
        self.uiBus.rx_app_pdu.emit(int(can_id), bytes(app_pdu))

    def _format_hex_dump(self, data: bytes, start_addr: int = 0, bytes_per_line: int = 16) -> str: 
        """Hex 덤프 형식으로 변환 (64바이트로 제한)"""
        data   = data[:64]
        result = []
        for i in range(0, len(data), bytes_per_line): 
            chunk      = data[i:i + bytes_per_line]
            addr       = start_addr + i
            addr_str   = f"{addr:04X}"
            hex_part   = " ".join(f"{b:02X}" for b in chunk).ljust(3 * bytes_per_line - 1)
            ascii_part = "".join(chr(b) if 32 <= b <= 126 else "." for b in chunk)
            result.append(f"{addr_str}  {hex_part}  {ascii_part}")
        if len(data) == 64:
            result.append(f"[TRUNCATED to 64 bytes - original: {len(data)} bytes]")
        return "\n".join(result)

    def _append_console_app_pdu(self, can_id: int, app_pdu: bytes): 
        try                                                       : 
    if  not app_pdu                                               : 
                self.console.append(f"CAN ID: 0x{can_id:X}\n<EMPTY>\n")
                return

                   now_str  = datetime.now().strftime("%Y-%m-%d %H-%M-%S.%f")[:-3]
            msg_id, payload, payload_len = extract_payload_from_app_pdu(app_pdu)
                   header   = f"[{now_str}] MSG_ID: 0x{msg_id:02X} PDU({len(app_pdu)}B) PAYLOAD({payload_len}B)\n"
                   hex_dump = self._format_hex_dump(payload, start_addr=0x0000)
            self.console.append(header + hex_dump + "\n")
        except Exception as e: 
            self.console.append(f"[DISPLAY-ERR] {e}")

    def _on_ui_rx_app_pdu(self, can_id: int, data_obj: object): 
        try                                                   : 
            app_pdu = bytes(data_obj) if not isinstance(data_obj, (bytes, bytearray)) else bytes(data_obj)
        except Exception: 
            return
        self._append_console_app_pdu(can_id, app_pdu)

    def _teardown_on_error(self)   : 
        try                        : 
    if  self.pollWorker is not None: 
                self.pollWorker.stop()
                self.pollWorker.wait(500)
                self.pollWorker = None
        except Exception: 
            pass
           try                          : 
        if self._rx_receiver is not None: 
           try                          : 
                    self._rx_receiver.stop_rx()
                except Exception: 
                    pass
                try: 
                    self._rx_receiver.close()
                except Exception: 
                    pass
                self._rx_receiver = None
        except Exception: 
            pass
           try                          : 
        if self.loggerWorker is not None: 
                self.loggerWorker.stop()
                self.loggerWorker.wait(1000)
                self.loggerWorker = None
        except Exception: 
            pass
        if self._rx_mgr_owned and self._rx_mgr is not None: 
           try                                            : 
        if hasattr(self._rx_mgr, "close")                 : 
                    self._rx_mgr.close()  # type: ignore[attr-defined]
            except Exception: 
                pass
            self._rx_mgr_owned = False
            self._rx_mgr       = None
           try                    : 
        if self.sender is not None: 
                self.sender.disconnect()
        except Exception: 
            pass
        self.sender = None

    def on_disconnect(self)        : 
        try                        : 
    if  self.pollWorker is not None: 
                self.pollWorker.stop()
                self.pollWorker.wait(1000)
                self.pollWorker = None
            if self._rx_receiver is not None: 
               try                          : 
                    self._rx_receiver.close()
                except Exception: 
                    pass
                self._rx_receiver = None
        except Exception: 
            pass

           try                          : 
        if self.loggerWorker is not None: 
                self.loggerWorker.stop()
                self.loggerWorker.wait(1500)
                self.loggerWorker = None
        except Exception: 
            pass

        if self._rx_mgr_owned and self._rx_mgr is not None: 
           try                                            : 
        if hasattr(self._rx_mgr, "close")                 : 
                    self._rx_mgr.close()  # type: ignore[attr-defined]
            except Exception: 
                pass
            self._rx_mgr_owned = False
            self._rx_mgr       = None

        if self.sender is not None: 
           try                    : 
           try                    : 
                    self.sender.stop_repl()
                except Exception: 
                    pass
                self.sender.disconnect()
            except Exception as e: 
                self._err(str(e))
            self.sender = None

        self.btnConnect.setEnabled(True)
        self.btnDisconnect.setEnabled(False)
        self.statusBar().showMessage("Disconnected")
        self._append_log("[INFO] 연결 해제")

      # ---------- 전송 (AppPDU 적용) ----------
    def _build_app_pdu(self, text_line: str, msg_id: int, append_lf: bool) -> bytes: 
            payload = text_line.encode("ascii", errors="ignore")
            if append_lf: 
                payload += b"\n"

            length    = len(payload)
            len_bytes = bytes((
                length & 0xFF,
                (length >> 8) & 0xFF,
                (length >> 16) & 0xFF,
                (length >> 24) & 0xFF,
            ))
              # [Len(4B LE)] + [MsgID(1B)] + Payload
            return len_bytes + bytes([msg_id & 0xFF]) + payload

    def _local_frame_bytes(self, app_pdu: bytes) -> List[bytes]: 
        frames                                                 : List[bytes] = []
        n   = len(app_pdu)
        pos = 0
        seq = 0

        while (n - pos) > CHUNK_DATA_MAX: 
               seq_high    = (seq >> 8) & SEQ_HIGH_MASK
               seq_low     = seq & 0xFF
               hdr1        = seq_high                              # bit7=0 (MIDDLE)
               hdr2        = seq_low
               chunk       = app_pdu[pos:pos + CHUNK_DATA_MAX]
            if len(chunk) != CHUNK_DATA_MAX:
               chunk       = chunk.ljust(CHUNK_DATA_MAX, b"\x00")
               fr          = bytes([hdr1, hdr2]) + chunk
            if len(fr)    != 64:
               fr          = fr.ljust(64, b"\x00")
            frames.append(fr)
            pos += CHUNK_DATA_MAX
            seq  = (seq + 1) & MAX_SEQ

          # LAST
        remain   = n - pos
        seq_high = (seq >> 8) & SEQ_HIGH_MASK
        seq_low  = seq & 0xFF
        hdr1     = HDR_LAST_BIT | seq_high
        hdr2     = seq_low
        if remain > 0: 
            last_chunk = app_pdu[pos:pos + remain]
            pad        = b"\x00" * (CHUNK_DATA_MAX - remain)
            fr         = bytes([hdr1, hdr2]) + last_chunk + pad
        else: 
           fr       = bytes([hdr1, hdr2]) + b"\x00" * CHUNK_DATA_MAX
        if len(fr) != 64:
           fr       = fr.ljust(64, b"\x00")
        frames.append(fr)

        return frames

    def _send_one_line_app(self, line: str): 
    if  self.sender is None                : 
            raise RuntimeError("Sender not connected")

        msg_id  = int(self.edMsgId.value()) & 0xFF
        app_pdu = self._build_app_pdu(line, msg_id, append_lf=self.chkAppendLF.isChecked())

          # ===== 강제: 항상 로컬 프레이밍 사용 =====
        frames = self._local_frame_bytes(app_pdu)

          # 안전 가드 + 로깅
        for i,             fr in enumerate(frames): 
        if  len(fr) != 64: 
                print(f"[BUG] frame#{i} length={len(fr)} != 64 → padding", flush=True)
                fr = fr.ljust(64, b"\x00")
            print(f"[TX-FRAME] #{i} len={len(fr)} hdr1=0x{fr[0]:02X} hdr2=0x{fr[1]:02X}", flush=True)

                                              # 실제 송신
            mgr = self.sender._require_mgr()  # type: ignore[attr-defined]
            can_id = int(getattr(self.sender, "can_id_11bit", 0xC0))
            if hasattr(mgr, "send_bytes"): 
                mgr.send_bytes(can_id, fr)
            elif hasattr(mgr, "send_frame"): 
                mgr.send_frame(can_id, fr)
            elif hasattr(mgr, "write"): 
                mgr.write(can_id, fr)
            else: 
                raise RuntimeError("PCANManager에 전송 함수(send_bytes/send_frame/write)가 없습니다.")

    def _start_send(self, lines: List[str]): 
    if  self.sender is None                : 
            self._err("연결되어 있지 않습니다. 먼저 Connect 하세요.")
            return
        if not lines: 
            self._append_log("[WARN] 전송할 라인이 없습니다.")
            return
        self.progress.setValue(0)
        self.btnSendAll.setEnabled(False)
        self.btnSendSelected.setEnabled(False)
        self.btnStop.setEnabled(True)

        self.txWorker = SendWorker(self._send_one_line_app, lines)
        self.txWorker.progress.connect(self.progress.setValue)
        self.txWorker.lineSent.connect(lambda s: self._append_log(f"[TX] {s}"))
        self.txWorker.error.connect(self._on_worker_error)
        self.txWorker.finishedOk.connect(self._on_worker_done)
        self.txWorker.start()

    def _on_worker_error(self, msg: str): 
        self._append_log(f"[ERR] {msg}")
        self._reset_buttons()

    def _on_worker_done(self): 
        self._append_log("[INFO] 전송 완료")
        self._reset_buttons()

    def _reset_buttons(self): 
        self.btnSendAll.setEnabled(True)
        self.btnSendSelected.setEnabled(True)
        self.btnStop.setEnabled(False)
        self.progress.setValue(100)
        self.txWorker = None

    def on_send_all(self): 
        lines = self._filter_lines(self.textEditor.toPlainText().splitlines())
        self._start_send(lines)

    def on_send_selected(self): 
        sel = self.listLines.selectedItems()
        if sel: 
            lines = [it.text() for it in sel]
        else: 
            cursor = self.textEditor.textCursor()
            lines  = [cursor.block().text()]
        self._start_send(self._filter_lines(lines))

    def on_stop(self)            : 
    if  self.txWorker is not None: 
            self.txWorker.stop()
            self._append_log("[INFO] 중지 요청됨")

      # ---------- REPL ----------
    def on_repl_enter(self): 
    if  self.sender is None: 
            self.console.append("[ERR] 연결되어 있지 않습니다.")
            return
        cmd = self.consoleInput.text().strip()
        if not cmd: 
            return
        self.hist.append(cmd); self.hidx = len(self.hist)
        self.console.append(f">>> {cmd}")
        try: 
            self._send_one_line_app(cmd)
        except Exception as e: 
            self.console.append(f"[ERR] {e}")
        finally: 
            self.consoleInput.clear()

      # ↑/↓ 히스토리, Ctrl+L 클리어
    def eventFilter(self, obj, ev)                                    : 
    if  obj is self.consoleInput and ev.type() == QEvent.Type.KeyPress: 
               key  = ev.key(); mod = ev.modifiers()
            if key == Qt.Key_L and (mod & Qt.KeyboardModifier.ControlModifier):
                self.console.clear(); self.consoleInput.clear(); return True
            if key == Qt.Key_Up: 
            if self.hist       : 
                    self.hidx = max(0, self.hidx - 1)
                    self.consoleInput.setText(self.hist[self.hidx]); return True
            if key == Qt.Key_Down: 
            if self.hist         : 
                       self.hidx  = min(len(self.hist), self.hidx + 1)
                    if self.hidx == len(self.hist): self.consoleInput.clear()
                    else: self.consoleInput.setText(self.hist[self.hidx])
                    return True
        return super().eventFilter(obj, ev)

      # 창 닫을 때 안전 정리
    def closeEvent(self, ev): 
        try                 : 
            self.on_disconnect()
        except Exception: 
            pass
        super().closeEvent(ev)


def main(): 
    app = QApplication(sys.argv)
    w   = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
