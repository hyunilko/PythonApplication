#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_isotp_receiver.py

ISO-TP 완성 패킷 수신기 (Application Layer: [Msg ID | Payload...])
 - python-can + (가능시) python-can-isotp 스택으로 FF/CF 조립
 - 스택 조립 불능 시에도 수동 조립기로 FF/CF를 복구 (12-bit/32-bit FF 모두 지원)
 - 수신 패킷을 (payload, msg_id) 형태로 콜백/큐로 전달
 - GUI(can_isotp_manager_gui.py) 로거 포맷과 호환되는 단일행 로그 출력

필요 패키지:
    pip install python-can can-isotp
"""

from __future__ import annotations
from typing import Optional, Set, Callable, Tuple
import os
import sys
import time
import threading
import queue
import binascii
from datetime import datetime

import can
try:
    import isotp  # python-can-isotp
    _HAS_ISOTP = True
except Exception:
    _HAS_ISOTP = False

# stdout 라인 버퍼링
try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass


# ===== 유틸 =====
def _hex_bytes(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def _hex_compact(b: bytes) -> str:
    return binascii.hexlify(b).decode("ascii").upper()

def _get_attr(obj, name, default=None):
    return getattr(obj, name, default)

def _env_int(name: str, default: Optional[int]) -> Optional[int]:
    v = os.environ.get(name)
    if v is None:
        return default
    try:
        return int(v, 0)
    except Exception:
        return default

def _env_bool(name: str, default: Optional[bool]) -> Optional[bool]:
    v = os.environ.get(name)
    if v is None:
        return default
    v = v.strip().lower()
    if v in ("1", "true", "yes", "y", "on"):
        return True
    if v in ("0", "false", "no", "n", "off"):
        return False
    return default

def _env_float(name: str, default: Optional[float]) -> Optional[float]:
    v = os.environ.get(name)
    if v is None:
        return default
    try:
        return float(v)
    except Exception:
        return default

def _resolve_bus(rx_mgr) -> Optional[can.BusABC]:
    if isinstance(rx_mgr, can.BusABC):
        return rx_mgr
    for name in ("bus", "_bus"):
        b = _get_attr(rx_mgr, name, None)
        if isinstance(b, can.BusABC):
            return b
    for name in ("get_bus", "bus"):
        fn = _get_attr(rx_mgr, name, None)
        if callable(fn):
            try:
                b = fn()
                if isinstance(b, can.BusABC):
                    return b
            except Exception:
                pass
    return None


# ===== Application Layer =====
def unpack_app(pdu: bytes) -> Tuple[int, bytes]:
    if not pdu:
        return (-1, b"")
    return (pdu[0], pdu[1:])

def payload_to_text_or_hex(payload: bytes) -> str:
    return _hex_compact(payload)

def _log_app_pdu(msg_id: int, payload: bytes, preview_max: int = 64) -> None:
    try:
        pv_max = int(os.environ.get("APP_PREVIEW_MAX", str(preview_max)))
    except Exception:
        pv_max = preview_max
    pv = payload[:pv_max]

    now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    print(
        f"[{now_str}] [APP PDU RX] ID=0x{msg_id:02X} LEN={len(payload)} DATA[:{len(pv)}]={_hex_bytes(pv)}",
        flush=True
    )


# ===== 보조: SF 판별/추출 =====
def _looks_like_isotp_sf(data: bytes) -> bool:
    if len(data) < 2:
        return False
    pci = data[0]
    ptype = (pci >> 4) & 0xF
    if ptype != 0:
        return False
    if len(data) <= 8:
        sf_len = (pci & 0xF)
        return (sf_len > 0) and ((1 + sf_len) <= len(data))
    # CAN FD SF: low nibble 0, next byte = length
    if (pci & 0xF) != 0 or len(data) < 2:
        return False
    sf_len = data[1]
    return (2 + sf_len) <= len(data)

def _extract_isotp_sf_payload(data: bytes) -> Optional[bytes]:
    if not _looks_like_isotp_sf(data):
        return None
    if len(data) <= 8:
        return data[1:1 + (data[0] & 0xF)]
    return data[2:2 + data[1]]


# ===== 진행 감시(워치독) =====
class _IsoTpFlowWatch(can.Listener):
    def __init__(self, src_can_id: int,
                 first_cf_timeout_s: float,
                 cf_gap_timeout_s: float):
        super().__init__()
        self._src = int(src_can_id)
        self._first_cf_to = float(first_cf_timeout_s)
        self._cf_gap_to = float(cf_gap_timeout_s)
        self._in_rx = False
        self._t_ff = 0.0
        self._t_last_cf = 0.0
        self.reset_needed = False

    @staticmethod
    def _pci_type(data: bytes) -> Optional[int]:
        if not data:
            return None
        return (data[0] >> 4) & 0xF

    def on_message_received(self, msg: "can.Message"):
        try:
            if int(msg.arbitration_id) != self._src:
                return
            data = bytes(msg.data or b"")
            ptype = self._pci_type(data)
            now = time.monotonic()

            if ptype == 1:  # FF
                self._in_rx = True
                self._t_ff = now
                self._t_last_cf = 0.0
            elif ptype == 2:  # CF
                if self._in_rx:
                    self._t_last_cf = now
            elif ptype == 0:  # SF
                self._in_rx = False
                self._t_ff = 0.0
                self._t_last_cf = 0.0

            if self._in_rx:
                if self._t_last_cf == 0.0:
                    if (now - self._t_ff) > self._first_cf_to:
                        self.reset_needed = True
                else:
                    if (now - self._t_last_cf) > self._cf_gap_to:
                        self.reset_needed = True
        except Exception:
            pass

    def clear(self):
        self._in_rx = False
        self._t_ff = 0.0
        self._t_last_cf = 0.0
        self.reset_needed = False


class _RawFlowControlResponder(can.Listener):
    """
    FF 감지 → FC(CTS) 1회 즉시 전송 (STmin=0, BS=0).
    수신 스택이 FC 파라미터를 못 바꿀 때 강제용.
    """
    def __init__(self, bus: can.BusABC, txid: int, rxid: int, extended: bool,
                 stmin: int = 0, blocksize: int = 0, log_fn=None):
        super().__init__()
        self.bus = bus
        self.txid = int(txid)     # 우리가 보내는 FC ID (상대가 듣는 쪽)
        self.rxid = int(rxid)     # 상대가 보내는 FF/CF ID (우리가 듣는 쪽)
        self.extended = bool(extended)
        self.stmin = int(stmin) & 0xFF
        self.blocksize = int(blocksize) & 0xFF
        self._inflight = False
        self._log = log_fn or (lambda s: None)

    @staticmethod
    def _ptype(data: bytes) -> int:
        return (data[0] >> 4) & 0xF if data else -1

    def on_message_received(self, msg: "can.Message"):
        try:
            if int(msg.arbitration_id) != self.rxid:
                return
            data = bytes(msg.data or b"")
            if len(data) < 2:
                return
            ptype = self._ptype(data)

            if ptype == 1:  # FF
                if self._inflight:
                    return
                self._inflight = True

                fc = bytearray(8)
                fc[0] = 0x30         # FC + FS(CTS)
                fc[1] = self.blocksize
                fc[2] = self.stmin

                msg_fc = can.Message(
                    arbitration_id=self.txid,
                    is_extended_id=self.extended,
                    is_fd=False,
                    data=bytes(fc)
                )
                self.bus.send(msg_fc)
                self._log(f"[RAW-FC] Sent CTS: BS={self.blocksize} STmin={self.stmin} to 0x{self.txid:X}")

            elif ptype in (0, 3):  # SF/FC → 세션 종료
                self._inflight = False
        except Exception as e:
            self._log(f"[RAW-FC] error: {e}")

    def reset(self):
        self._inflight = False


# ===== RAW 탭(디버그) =====
class _TapAllListener(can.Listener):
    @staticmethod
    def _ptype(data: bytes):
        if not data:
            return None
        return (data[0] >> 4) & 0xF

    def on_message_received(self, msg: "can.Message"):
        try:
            cid = int(msg.arbitration_id)
            data = bytes(msg.data or b"")
            ptype = self._ptype(data)
            tag = {0: "SF", 1: "FF", 2: "CF", 3: "FC"}.get(ptype, "??")
            hx_head = " ".join(f"{b:02X}" for b in data[:16])

            if ptype == 3 and len(data) >= 3:
                fs = data[0] & 0x0F
                bs = data[1]
                st = data[2]
                if st <= 0x7F:
                    st_str = f"{st} ms"
                elif st == 0xF0:
                    st_str = "100 us"
                elif 0xF1 <= st <= 0xF9:
                    st_str = f"{(st-0xF0)}00 us"
                else:
                    st_str = f"0x{st:02X} (reserved)"
                print(
                    f"[RAW RX] ID=0x{cid:X} LEN={len(data)} PCI=FC FS={fs} BS={bs} STmin={st_str} DATA[:16]={hx_head}",
                    flush=True
                )
            else:
                print(
                    f"[RAW RX] ID=0x{cid:X} LEN={len(data)} PCI={tag} DATA[:16]={hx_head}",
                    flush=True
                )
        except Exception:
            pass


# ===== 폴백: SF만 APP 전달 =====
class _RawListener(can.Listener):
    def __init__(self,
                 out_q: "queue.Queue[Tuple[bytes,int]]",
                 emit_app: Callable[[int, bytes], None],
                 filter_ids: Optional[Set[int]],
                 debug_raw: bool = False):
        super().__init__()
        self._q = out_q
        self._emit_app = emit_app
        self._filter = set(filter_ids) if filter_ids else None
        self._debug_raw = debug_raw

    def on_message_received(self, msg: "can.Message"):
        try:
            can_id = int(msg.arbitration_id)
            data = bytes(msg.data or b"")
            if (self._filter is not None) and (can_id not in self._filter):
                return
            payload = _extract_isotp_sf_payload(data)
            if payload is None:
                return
            msg_id, body = unpack_app(payload)
            self._emit_app(msg_id, body)
        except Exception:
            pass


# ===== 수동 조립기(FF 12-bit/32-bit 모두 지원) =====
class _IsoTpProbeAssembler(can.Listener):
    """
    ID 하나에 대해 FF/CF 조립 (수신만).
    - FF 12-bit: 0x1L, L=high4 bits then next byte low8
    - FF 32-bit: 0x10 0x00 LEN32_BE
    """
    def __init__(self, target_rxid: int, emit: Callable[[int, bytes], None],
                 first_cf_timeout_s: float = 1.5, cf_gap_timeout_s: float = 0.8):
        super().__init__()
        self._rxid = int(target_rxid)
        self._emit = emit
        self._first_cf_to = float(first_cf_timeout_s)
        self._cf_gap_to = float(cf_gap_timeout_s)
        self._reset()

    def _reset(self):
        self._in_rx = False
        self._need_len = 0
        self._buf = bytearray()
        self._expect_sn = 1
        self._t_start = 0.0
        self._t_last = 0.0

    @staticmethod
    def _ptype(data: bytes) -> int:
        return (data[0] >> 4) & 0xF if data else -1

    def on_message_received(self, msg: "can.Message"):
        try:
            if int(msg.arbitration_id) != self._rxid:
                return
            data = bytes(msg.data or b"")
            if not data:
                return
            ptype = self._ptype(data)
            now = time.monotonic()

            if ptype == 0:  # SF
                if len(data) <= 8:
                    sf_len = data[0] & 0xF
                    pay = data[1:1 + sf_len]
                else:
                    if (data[0] & 0xF) != 0 or len(data) < 2:
                        return
                    sf_len = data[1]
                    pay = data[2:2 + sf_len]
                if not pay:
                    return
                msg_id = pay[0]
                body = pay[1:]
                self._emit(msg_id, body)
                self._reset()
                return

            if ptype == 1:  # FF (12-bit 또는 32-bit)
                if len(data) < 2:
                    return
                hi = data[0] & 0x0F
                if hi != 0:
                    # 12-bit
                    total = (hi << 8) | data[1]
                    pay_off = 2
                else:
                    # 32-bit (0x10 0x00 LEN32_BE)
                    if len(data) < 6 or data[1] != 0x00:
                        return
                    total = ((data[2] << 24) |
                             (data[3] << 16) |
                             (data[4] << 8) |
                             data[5])
                    pay_off = 6
                if total <= 0:
                    return

                self._buf = bytearray(data[pay_off:])
                self._need_len = int(total)
                self._in_rx = True
                self._expect_sn = 1
                self._t_start = now
                self._t_last = 0.0
                return

            if ptype == 2 and self._in_rx:  # CF
                sn = data[0] & 0x0F
                if self._t_last == 0.0:
                    if (now - self._t_start) > self._first_cf_to:
                        self._reset()
                        return
                else:
                    if (now - self._t_last) > self._cf_gap_to:
                        self._reset()
                        return
                if sn != self._expect_sn:
                    self._reset()
                    return

                self._expect_sn = (self._expect_sn + 1) & 0x0F
                self._buf.extend(data[1:])
                self._t_last = now

                if len(self._buf) >= self._need_len:
                    pdu = bytes(self._buf[:self._need_len])
                    if pdu:
                        msg_id = pdu[0]
                        body = pdu[1:]
                        self._emit(msg_id, body)
                    self._reset()
                return

            # FC/기타: 무시
        except Exception:
            pass


# ===== 공개 수신기 =====
class CanIsoTpReceiver:
    """
    ISO-TP 완성 패킷 수신기
      - 가능: NotifierBasedCanStack으로 FF/CF까지 조립 후 Application Layer 언패킹
      - 불가: 수동 조립기로 복구
    콜백 시그니처: on_packet(payload: bytes, msg_id: int)
    저장 포맷: "MSG_ID: 0x%X\n<PAYLOAD_HEX>\n\n"
    """

    def __init__(self, rx_mgr, filter_can_ids: Optional[Set[int]] = None, save_path: Optional[str] = None):
        self._mgr = rx_mgr
        self._bus: Optional[can.BusABC] = _resolve_bus(rx_mgr)
        if self._bus is None:
            raise RuntimeError("RX 매니저에서 CAN Bus를 얻지 못했습니다.")

        self._txid = _env_int("ISOTP_TXID", _get_attr(rx_mgr, "txid", None))
        self._rxid = _env_int("ISOTP_RXID", _get_attr(rx_mgr, "rxid", None))
        self._extended = _env_bool("ISOTP_EXT", _get_attr(rx_mgr, "extended", None))
        self._fd = _env_bool("ISOTP_FD", _get_attr(rx_mgr, "fd", None))

        self._q: "queue.Queue[Tuple[bytes,int]]" = queue.Queue(maxsize=4096)
        self._on_packet: Optional[Callable[[bytes,int], None]] = None
        self._stop_evt = threading.Event()

        self._use_isotp = _HAS_ISOTP and (self._txid is not None) and (self._rxid is not None)
        self._stack: Optional[isotp.NotifierBasedCanStack] = None
        self._thread: Optional[threading.Thread] = None
        self._notifier: Optional[can.Notifier] = None
        self._own_notifier: bool = False
        self._tap: Optional[_TapAllListener] = None
        self._raw_listener: Optional[_RawListener] = None
        self._manual: Optional[_IsoTpProbeAssembler] = None
        self._raw_fc: Optional[_RawFlowControlResponder] = None

        self._filter_ids = set(filter_can_ids) if filter_can_ids else None
        self._flow_watch: Optional[_IsoTpFlowWatch] = None
        self._address = None
        self._params = None

        self._save_path = save_path
        self._raw_tap_enabled = os.environ.get("RAW_TAP", "1").lower() not in ("0", "false", "no", "off")

        # 중복 방지(스택/수동 동시조립 시)
        self._last_app_key = None
        self._last_app_ts = 0.0
        self._dedup_window_s = float(os.environ.get("APP_DEDUP_SEC", "0.25"))

    # 저장
    def _save_record(self, msg_id: int, payload: bytes):
        if not self._save_path:
            return
        try:
            with open(self._save_path, "a", encoding="utf-8", newline="") as f:
                f.write(f"MSG_ID: 0x{int(msg_id):X}\n")
                f.write(payload_to_text_or_hex(payload))
                f.write("\n\n")
        except Exception:
            pass

    # 중복 방지 포함 APP emit
    def _emit_app(self, msg_id: int, body: bytes):
        key = (int(msg_id) & 0xFF, len(body), bytes(body[:8]))
        now = time.monotonic()
        if self._last_app_key == key and (now - self._last_app_ts) < self._dedup_window_s:
            return
        self._last_app_key = key
        self._last_app_ts = now

        _log_app_pdu(msg_id, body)
        try:
            self._q.put_nowait((body, msg_id))
        except queue.Full:
            pass
        self._save_record(msg_id, body)
        if self._on_packet:
            try:
                self._on_packet(body, msg_id)
            except Exception:
                pass

    def set_raw_tap(self, enabled: bool):
        self._raw_tap_enabled = bool(enabled)
        if not self._notifier:
            return
        if not enabled and self._tap is not None:
            try:
                self._notifier.remove_listener(self._tap)
            except Exception:
                pass
            self._tap = None
        elif enabled and self._tap is None:
            self._tap = _TapAllListener()
            try:
                self._notifier.add_listener(self._tap)
            except Exception:
                pass

    def start_rx(self, on_packet: Optional[Callable[[bytes,int], None]] = None):
        self._on_packet = on_packet
        self._stop_evt.clear()

        # Notifier: 지연 단축
        shared_notifier = _get_attr(self._mgr, "notifier", None)
        if isinstance(shared_notifier, can.Notifier):
            self._notifier = shared_notifier
            self._own_notifier = False
        else:
            self._notifier = can.Notifier(
                self._bus, [], timeout=float(os.environ.get("CAN_NOTIFIER_TIMEOUT", "0.005"))
            )
            self._own_notifier = True

        # RAW 탭
        if self._raw_tap_enabled and self._tap is None:
            self._tap = _TapAllListener()
            try:
                self._notifier.add_listener(self._tap)
            except Exception:
                pass

        # 수동 조립기(항상 장착)
        first_cf_to = float(os.environ.get("APP_FIRST_CF_TO", "1.5"))
        cf_gap_to = float(os.environ.get("APP_CF_GAP_TO", "0.8"))
        self._manual = _IsoTpProbeAssembler(
            int(self._rxid or 0), self._emit_app,
            first_cf_timeout_s=first_cf_to,
            cf_gap_timeout_s=cf_gap_to
        )
        try:
            self._notifier.add_listener(self._manual)
        except Exception:
            pass

        # 디버그: FORCE_RAW_FC 값 확인
        print(f"[DBG] FORCE_RAW_FC in receiver = {os.environ.get('FORCE_RAW_FC')}", flush=True)

        # FORCE_RAW_FC=1 → Raw FlowControl Responder 활성화
        try:
            force_raw_fc = os.environ.get("FORCE_RAW_FC", "0").lower() in ("1", "true", "yes", "on")
            if force_raw_fc and (self._txid is not None) and (self._rxid is not None):
                self._raw_fc = _RawFlowControlResponder(
                    bus=self._bus,
                    txid=int(self._txid),
                    rxid=int(self._rxid),
                    extended=bool(self._extended),
                    stmin=0,
                    blocksize=0,
                    log_fn=lambda s: print(s, flush=True)
                )
                try:
                    self._notifier.add_listener(self._raw_fc)
                    print("[INFO] RAW FC responder enabled (STmin=0, BS=0)", flush=True)
                except Exception as e:
                    print(f"[WARN] RAW FC responder add_listener failed: {e}", flush=True)
            else:
                self._raw_fc = None
        except Exception as e:
            print(f"[WARN] RAW FC responder init failed: {e}", flush=True)

        if self._use_isotp:
            addr_mode = (isotp.AddressingMode.Normal_29bits
                         if (self._extended is True)
                         else isotp.AddressingMode.Normal_11bits)
            address = isotp.Address(addr_mode, txid=int(self._txid), rxid=int(self._rxid))

            rx_fc_to_ms = int(float(os.environ.get("ISOTP_RX_FC_TO_MS", "7000")))
            rx_cf_to_ms = int(float(os.environ.get("ISOTP_RX_CF_TO_MS", "7000")))
            wftmax = int(float(os.environ.get("ISOTP_WFTMAX", "8")))
            params = dict(
                stmin=0,
                blocksize=0,
                wftmax=wftmax,
                can_fd=bool(self._fd),
                tx_data_length=64 if self._fd else 8,
                tx_data_min_length=64 if self._fd else 8,
                tx_padding=0x00,
                rx_flowcontrol_timeout=rx_fc_to_ms,
                rx_consecutive_frame_timeout=rx_cf_to_ms,
            )

            self._address = address
            self._params = params

            print(
                f"[ISOTP] use_stack=1 txid=0x{int(self._txid):X} rxid=0x{int(self._rxid):X} "
                f"fd={bool(self._fd)} ext={bool(self._extended)}",
                flush=True
            )

            # 진행 감시(수신 지연 감지)
            wd_first_cf_ms = _env_float("ISOTP_WD_FIRST_CF_MS", 1500.0)
            wd_cf_gap_ms = _env_float("ISOTP_WD_CF_GAP_MS", 800.0)
            self._flow_watch = _IsoTpFlowWatch(
                src_can_id=int(self._rxid),
                first_cf_timeout_s=(wd_first_cf_ms or 1500.0) / 1000.0,
                cf_gap_timeout_s=(wd_cf_gap_ms or 800.0) / 1000.0
            )
            try:
                self._notifier.add_listener(self._flow_watch)
            except Exception:
                pass

            # 에러 핸들러: 타임아웃 등 발생 시 스택 리셋
            def _on_isotp_error(err: isotp.IsoTpError):
                try:
                    print(f"IsoTP error: {err.__class__.__name__}: {str(err)}", flush=True)
                except Exception:
                    pass
                try:
                    reset_fn = getattr(self._stack, "reset", None)
                    if callable(reset_fn):
                        reset_fn()
                    else:
                        self._stack = isotp.NotifierBasedCanStack(
                            self._bus, self._notifier, address=self._address, params=self._params
                        )
                except Exception:
                    pass

            # ISO-TP 스택
            try:
                self._stack = isotp.NotifierBasedCanStack(
                    self._bus, self._notifier, address=address,
                    params=params, error_handler=_on_isotp_error
                )
            except Exception as e:
                raise RuntimeError(f"isotp.NotifierBasedCanStack 생성 실패: {e}")

            # 스택 처리 루프
            def _loop():
                while not self._stop_evt.is_set():
                    try:
                        stack = self._stack
                        if stack is None:
                            break    # 또는 time.sleep(0.01); continue

                        self._stack.process()

                        if self._stack.available():
                            app = bytes(self._stack.recv())   # [MsgID][Payload...]
                            msg_id, body = unpack_app(app)
                            self._emit_app(msg_id, body)
                            if self._flow_watch:
                                self._flow_watch.clear()

                        # 워치독 트리거 시 스택 리셋
                        if self._flow_watch and self._flow_watch.reset_needed:
                            print("[WARN] ISO-TP RX watchdog fired -> reset stack", flush=True)
                            self._flow_watch.clear()
                            try:
                                reset_fn = getattr(self._stack, "reset", None)
                                if callable(reset_fn):
                                    reset_fn()
                                else:
                                    self._stack = isotp.NotifierBasedCanStack(
                                        self._bus, self._notifier,
                                        address=self._address, params=self._params
                                    )
                            except Exception:
                                time.sleep(0.05)

                        time.sleep(0.001)
                    except Exception:
                        time.sleep(0.01)

            self._thread = threading.Thread(target=_loop, daemon=False)
            self._thread.start()

        else:
            # ISO-TP 스택 미사용 시: SF 폴백 리스너
            self._raw_listener = _RawListener(self._q, self._emit_app, self._filter_ids)
            try:
                self._notifier.add_listener(self._raw_listener)
            except Exception:
                pass

    def receive_packet(self, timeout_s: float = 1.0, verbose: bool = False) -> Optional[Tuple[bytes, int]]:
        try:
            return self._q.get(timeout=max(0.001, float(timeout_s)))
        except queue.Empty:
            return None

    def stop_rx(self):
        self._stop_evt.set()

        if self._thread is not None:
            try:
                self._thread.join(timeout=1.0)
            except Exception:
                pass
            self._thread = None

        self._stack = None

        if self._notifier is not None:
            for lst_attr in ("_tap", "_flow_watch", "_manual", "_raw_listener", "_raw_fc"):
                lst = getattr(self, lst_attr, None)
                if lst is not None:
                    try:
                        self._notifier.remove_listener(lst)
                    except Exception:
                        pass
                    setattr(self, lst_attr, None)

        if self._own_notifier and self._notifier is not None:
            try:
                self._notifier.stop()
            except Exception:
                pass
        self._notifier = None

        try:
            while True:
                self._q.get_nowait()
        except queue.Empty:
            pass

    def close(self):
        self.stop_rx()


# ===== 단독 실행 =====
if __name__ == "__main__":
    rx_mgr = None
    save_path = os.environ.get("SAVE_PATH", None)

    # 1) 송신기 백엔드 경로 시도
    try:
        from can_isotp_sender import CanIsoTpSender
        fd_flag = _env_bool("PCAN_FD", True)
        sender = CanIsoTpSender(
            channel=os.environ.get("PCAN_CHANNEL", "PCAN_USBBUS1"),
            bitrate=int(os.environ.get("PCAN_CLASSIC_BITRATE", "500000")),
            fd=bool(fd_flag),
            f_clock_mhz=_env_int("PCAN_F_CLOCK_MHZ", 80) if "PCAN_F_CLOCK" not in os.environ else None,
            f_clock=_env_int("PCAN_F_CLOCK", None),
            nom_brp=_env_int("PCAN_NOM_BRP", 2),  # norminal bitrate = f_clock / (brp * (sjw + tseg1 + tseg2)) = 80 MHz / (2 * (1 + 33 + 6))) = 1 Mbps
            nom_tseg1=_env_int("PCAN_NOM_TSEG1", 33),
            nom_tseg2=_env_int("PCAN_NOM_TSEG2", 6),
            nom_sjw=_env_int("PCAN_NOM_SJW", 1),
            data_brp=_env_int("PCAN_DATA_BRP", 2), # Data bitrate = f_clock / (brp * (sjw + tseg1 + tseg2)) = 80 MHz / (2 * (1 + 6 + 1))) = 5 Mbps
            data_tseg1=_env_int("PCAN_DATA_TSEG1", 6),
            data_tseg2=_env_int("PCAN_DATA_TSEG2", 1),
            data_sjw=_env_int("PCAN_DATA_SJW", 1),
            txid=_env_int("ISOTP_TXID", 0xC0),
            rxid=_env_int("ISOTP_RXID", 0xC8),
            extended=_env_bool("ISOTP_EXT", False),
            save_path=None,
        )
        sender.connect(ifg_us=int(os.environ.get("PCAN_IFG_US", "3000")))
        rx_mgr = sender._require_mgr()
        print("[INFO] Bus 확보: sender._require_mgr() 사용")
    except Exception as e:
        print(f"[WARN] CanIsoTpSender 경로 실패: {e}")

    # 2) 직접 Bus 오픈
    if rx_mgr is None:
        try:
            channel = os.environ.get("PCAN_CHANNEL", "PCAN_USBBUS1")
            fd_flag = _env_bool("PCAN_FD", True)

            if fd_flag:
                fd_kwargs = dict(
                    f_clock_mhz=_env_int("PCAN_F_CLOCK_MHZ", 80) if "PCAN_F_CLOCK" not in os.environ else None,
                    f_clock=_env_int("PCAN_F_CLOCK", None),
                    nom_brp=_env_int("PCAN_NOM_BRP", 2),
                    nom_tseg1=_env_int("PCAN_NOM_TSEG1", 33),
                    nom_tseg2=_env_int("PCAN_NOM_TSEG2", 6),
                    nom_sjw=_env_int("PCAN_NOM_SJW", 1),
                    data_brp=_env_int("PCAN_DATA_BRP", 2),
                    data_tseg1=_env_int("PCAN_DATA_TSEG1", 6),
                    data_tseg2=_env_int("PCAN_DATA_TSEG2", 1),
                    data_sjw=_env_int("PCAN_DATA_SJW", 1),
                )
                fd_kwargs = {k: v for k, v in fd_kwargs.items() if v is not None}
                bus = can.Bus(interface="pcan", channel=channel, fd=True, **fd_kwargs)
            else:
                classic_bitrate = int(os.environ.get("PCAN_CLASSIC_BITRATE", "500000"))
                bus = can.Bus(interface="pcan", channel=channel, bitrate=classic_bitrate)

            class _MiniMgr:
                def __init__(self, b, fdv: bool):
                    self._bus = b
                    self.txid = _env_int("ISOTP_TXID", 0xC0)
                    self.rxid = _env_int("ISOTP_RXID", 0xC8)
                    self.extended = _env_bool("ISOTP_EXT", False)
                    self.fd = bool(fdv)
                def get_bus(self):
                    return self._bus

            rx_mgr = _MiniMgr(bus, fd_flag)
            print(f"[INFO] Bus 확보: python-can 직접 오픈 (fd={fd_flag})")
        except Exception as e:
            print(f"[ERR] Bus 직접 오픈 실패: {e}")
            raise SystemExit(1)

    rx = CanIsoTpReceiver(rx_mgr, save_path=save_path)

    def _on_pkt(payload: bytes, msg_id: int):
        try:
            txt = payload.decode("utf-8").rstrip("\r\n")
            print(f"[RX] MsgID={msg_id} {len(payload)}B ASCII: {txt}")
        except UnicodeDecodeError:
            hx = " ".join(f"{b:02X}" for b in payload)
            print(f"[RX] MsgID={msg_id} {len(payload)}B HEX  : {hx}")

    try:
        rx.start_rx(on_packet=_on_pkt)
        rx.set_raw_tap(False)
        print("listening... Ctrl+C to stop")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rx.close()
        except Exception:
            pass
        try:
            if 'sender' in globals():
                sender.disconnect()
        except Exception:
            pass
