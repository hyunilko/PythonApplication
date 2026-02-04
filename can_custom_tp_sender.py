#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_custom_tp_sender.py (class-based, Custom-TP)

Application Layer:
    APP_PDU = Len(4B, little-endian) + MsgID(1B) + Payload

Transport Layer (Custom-TP over CAN FD 64B):
    Frame = HDR(2B) + DATA(62B)

    HDR:
        HDR1:
            bit7 = 1 → LAST frame
            bit7 = 0 → MIDDLE frame
            bit6 = 0 (reserved)
            bit5..0 = SEQ high (6 bits)
        HDR2:
            SEQ low (8 bits)

        SEQ = (SEQ_high << 8) | SEQ_low   # 14 bits, 0..16383

    DATA:
        frame[2:64] = 62 bytes
        - APP_PDU 를 앞에서부터 순차적으로 잘라 채움
        - MIDDLE: 항상 62B 사용
        - LAST  : 남은 길이(<62)이면 zero padding
"""

from __future__ import annotations
from typing import List, Optional, Callable, Dict, Tuple
import argparse
import sys
import os
import time

# ========= 기본 정책값 =========
DEFAULT_CHANNEL: str = "PCAN_USBBUS1"
DEFAULT_BITRATE_FD: str = (
    "f_clock=80000000,"
    "nom_brp=2,nom_tseg1=33,nom_tseg2=6,nom_sjw=1,"    # norminal bitrate = f_clock / (brp * (sjw + tseg1 + tseg2)) = 80 MHz / (2 * (1 + 33 + 6))) = 1 Mbps
    "data_brp=2,data_tseg1=6,data_tseg2=1,data_sjw=1," # Data bitrate = f_clock / (brp * (sjw + tseg1 + tseg2)) = 80 MHz / (2 * (1 + 6 + 1))) = 5 Mbps
    "data_ssp_offset=14"
)
DEFAULT_CAN_ID_11BIT: int = 0xC0          # Link-layer CAN ID
DEFAULT_COMMAND_MSG_ID: int = 0xFA        # 기본 Application MsgID (텍스트 모드용)
DEFAULT_APPEND_LF: bool = True

CHUNK_DATA_MAX: int = 62                  # 64 - 2 (HDR)
DEFAULT_FRAME_GAP_US: int = 3000          # 프레임 간 간격 (pcan_manager에 전달)
DEFAULT_CMD_GAP_MS: int = 20              # 한 논리 명령(APP_PDU) 전송 후 대기(ms)
# ===============================

# HDR 비트 정의
HDR_LAST_BIT = 0x80
SEQ_HIGH_MASK = 0x3F       # HDR1 bits5..0
MAX_SEQ = 0x3FFF           # 14-bit

# ===== pcan_manager 백엔드 =====
try:
    from pcan_manager import PCANManager
except Exception as _PM_EXC:
    PCANManager = None  # type: ignore
    _PM_IMPORT_ERROR = _PM_EXC
else:
    _PM_IMPORT_ERROR = None


MAX_SEQ = 0x3FFF           # 14-bit

def u32_to_le4(n: int) -> bytes:
    """uint32 값을 4바이트 little-endian으로 변환"""
    n = int(n) & 0xFFFFFFFF
    return bytes((
        n & 0xFF,
        (n >> 8) & 0xFF,
        (n >> 16) & 0xFF,
        (n >> 24) & 0xFF,
    ))


class CanCustomTpSender:
    """
    Custom-TP 송신기

    - Application PDU:
        [Len(4B, LE)] + [MsgID(1B)] + Payload

    - Transport Layer:
        각 프레임 = [HDR1,HDR2] + 62B 데이터
        APP_PDU 전체를 순차적으로 자르며,
        MIDDLE 프레임은 62B 풀로 사용,
        LAST 프레임은 남은 바이트 + zero padding.
    """

    def __init__(
        self,
        channel: str = DEFAULT_CHANNEL,
        bitrate_fd: str = DEFAULT_BITRATE_FD,
        can_id_11bit: int = DEFAULT_CAN_ID_11BIT,
        app_msg_id: int = DEFAULT_COMMAND_MSG_ID,
        append_lf: bool = DEFAULT_APPEND_LF,
        frame_gap_us: int = DEFAULT_FRAME_GAP_US,
        cmd_gap_ms: int = DEFAULT_CMD_GAP_MS,
    ):
        # 설정값
        self.channel = channel
        self.bitrate_fd = bitrate_fd
        self.can_id_11bit = int(can_id_11bit) & 0x7FF
        self.app_msg_id = int(app_msg_id) & 0xFF
        self.append_lf = bool(append_lf)
        self.frame_gap_us = max(0, int(frame_gap_us))
        self.cmd_gap_ms = max(0, int(cmd_gap_ms))

        # 런타임
        self._mgr: Optional["PCANManager"] = None
        self._on_rx_cb: Optional[Callable[[str, int], None]] = None
        # can_id → (expected_seq, buffer)
        self._rx_assemblers: Dict[int, Tuple[int, bytearray]] = {}

    # ========================= 연결/해제 =========================
    def connect(
        self,
        channel: Optional[str] = None,
        bitrate_fd: Optional[str] = None,
        ifg_us: Optional[int] = None,
    ) -> None:
        """PCAN 연결 및 프레임 간격 설정."""
        if PCANManager is None:
            raise ImportError(f"pcan_manager import 실패: {_PM_IMPORT_ERROR}")

        if channel is not None:
            self.channel = channel
        if bitrate_fd is not None:
            self.bitrate_fd = bitrate_fd
        if ifg_us is not None:
            self.frame_gap_us = max(0, int(ifg_us))

        if self._mgr is None:
            self._mgr = PCANManager()

        mgr = self._mgr
        if hasattr(mgr, "open"):
            mgr.open(self.channel, self.bitrate_fd, ifg_us=self.frame_gap_us)      # type: ignore[attr-defined]
        elif hasattr(mgr, "connect"):
            mgr.connect(self.channel, self.bitrate_fd, ifg_us=self.frame_gap_us)   # type: ignore[attr-defined]
        else:
            raise RuntimeError("PCANManager에 open/connect 메서드가 없습니다.")

    def disconnect(self) -> None:
        """PCAN 링크 해제."""
        try:
            self.stop_repl()
        except Exception:
            pass

        mgr = self._mgr
        if not mgr:
            self._mgr = None
            return

        if hasattr(mgr, "close"):
            try:
                mgr.close()  # type: ignore[attr-defined]
            except Exception:
                pass
        elif hasattr(mgr, "disconnect"):
            try:
                mgr.disconnect()  # type: ignore[attr-defined]
            except Exception:
                pass

        self._mgr = None

    def _require_mgr(self) -> "PCANManager":
        if self._mgr is None:
            self.connect(self.channel, self.bitrate_fd, ifg_us=self.frame_gap_us)
        assert self._mgr is not None
        return self._mgr

    # ========================= 프레임 빌드 =========================
    def _build_frames_from_app_pdu(self, app_pdu: bytes) -> List[bytes]:
        """
        APP_PDU (= MsgID(1B) + Payload) 를
        Custom-TP 규약에 맞는 64B 프레임들로 분할.

        - MIDDLE: HDR1.bit7=0, DATA=62B (full)
        - LAST  : HDR1.bit7=1, DATA=남은 APP_PDU + zero padding
        """
        frames: List[bytes] = []
        n = len(app_pdu)

        if n <= 0:
            # 빈 PDU인 경우: SEQ=0, LAST 플래그 + 1바이트 dummy + padding
            seq = 0
            hdr1 = HDR_LAST_BIT | ((seq >> 8) & SEQ_HIGH_MASK)
            hdr2 = seq & 0xFF
            data = b"\x00" + (b"\x00" * (CHUNK_DATA_MAX - 1))
            frames.append(bytes((hdr1, hdr2)) + data)
            return frames

        seq = 0
        pos = 0

        # ----- MIDDLE 프레임들 (62B씩 풀로 채움) -----
        while (n - pos) > CHUNK_DATA_MAX:
            seq_high = (seq >> 8) & SEQ_HIGH_MASK
            seq_low = seq & 0xFF
            hdr1 = seq_high                # bit7=0 (MIDDLE)
            hdr2 = seq_low

            chunk = app_pdu[pos:pos + CHUNK_DATA_MAX]
            if len(chunk) < CHUNK_DATA_MAX:
                # 이 while 조건상 일반적으로 발생 X, 방어적 패딩
                chunk = chunk.ljust(CHUNK_DATA_MAX, b"\x00")

            frames.append(bytes((hdr1, hdr2)) + chunk)
            pos += CHUNK_DATA_MAX
            seq = (seq + 1) & MAX_SEQ

        # ----- LAST 프레임 -----
        remain = n - pos
        seq_high = (seq >> 8) & SEQ_HIGH_MASK
        seq_low = seq & 0xFF
        hdr1 = HDR_LAST_BIT | seq_high    # bit7=1 (LAST)
        hdr2 = seq_low

        if remain > 0:
            last_chunk = app_pdu[pos:pos + remain]
            if len(last_chunk) < CHUNK_DATA_MAX:
                last_chunk = last_chunk.ljust(CHUNK_DATA_MAX, b"\x00")
            data = last_chunk
        else:
            # 정확히 62의 배수인 경우: payload 없음, 62B zero padding
            data = b"\x00" * CHUNK_DATA_MAX

        frames.append(bytes((hdr1, hdr2)) + data)

        # 모든 프레임은 64B
        # (검사 필요시: assert all(len(f) == 64 for f in frames))

        return frames

    # ========================= 송신 API =========================
    def send_app_pdu(self, msg_id: int, payload: bytes) -> None:
            """
            Application PDU 전송:
            APP_PDU = [Len(4B, LE)] + [MsgID(1B)] + Payload
            를 Custom-TP 프레임들로 분할하여 송신.
            """
            mgr = self._require_mgr()
            can_id = int(self.can_id_11bit) & 0x7FF

            payload = payload or b""
            length = len(payload)
            len_bytes = u32_to_le4(length)

            # [Len(4B LE)] + [MsgID(1B)] + Payload
            app_pdu = len_bytes + bytes([msg_id & 0xFF]) + payload
            frames = self._build_frames_from_app_pdu(app_pdu)

            for fr in frames:
                if len(fr) != 64:
                    fr = fr.ljust(64, b"\x00")

                if hasattr(mgr, "send_bytes"):
                    mgr.send_bytes(can_id, fr)      # type: ignore[attr-defined]
                elif hasattr(mgr, "send_frame"):
                    mgr.send_frame(can_id, fr)      # type: ignore[attr-defined]
                elif hasattr(mgr, "write"):
                    mgr.write(can_id, fr)           # type: ignore[attr-defined]
                else:
                    raise RuntimeError("PCANManager에 전송 함수(send_bytes/send_frame/write)가 없습니다.")

            if self.cmd_gap_ms > 0:
                time.sleep(self.cmd_gap_ms / 1000.0)

    def _build_framed_payloads_from_text(self, text: str) -> List[bytes]:
            """
            텍스트 한 줄을:
            APP_PDU = [Len(4B, LE)] + [self.app_msg_id] + text(+ LF)
            로 만들고, Custom-TP 프레임들로 분할.
            """
            s = text
            if self.append_lf:
                s = s.rstrip("\r\n") + "\n"
            payload = s.encode("ascii", errors="ignore")

            length = len(payload)
            len_bytes = u32_to_le4(length)
            app_pdu = len_bytes + bytes([self.app_msg_id & 0xFF]) + payload
            return self._build_frames_from_app_pdu(app_pdu)


    def send_line(self, text: str) -> None:
        """텍스트 한 줄을 APP_PDU로 감싸 Custom-TP로 전송."""
        if text is None:
            return
        line = text.strip()
        if not line and not self.append_lf:
            return

        mgr = self._require_mgr()
        can_id = int(self.can_id_11bit) & 0x7FF

        frames = self._build_framed_payloads_from_text(line)
        for fr in frames:
            if len(fr) != 64:
                fr = fr.ljust(64, b"\x00")

            if hasattr(mgr, "send_bytes"):
                mgr.send_bytes(can_id, fr)      # type: ignore[attr-defined]
            elif hasattr(mgr, "send_frame"):
                mgr.send_frame(can_id, fr)      # type: ignore[attr-defined]
            elif hasattr(mgr, "write"):
                mgr.write(can_id, fr)           # type: ignore[attr-defined]
            else:
                raise RuntimeError("PCANManager에 전송 함수(send_bytes/send_frame/write)가 없습니다.")

        if self.cmd_gap_ms > 0:
            time.sleep(self.cmd_gap_ms / 1000.0)

    def send_lines(self, lines: List[str]) -> None:
        for line in lines:
            self.send_line(line)

    # ========================= 수신 조립 (Optional) =========================
    def _assembler_reset(self, can_id: int, start_seq: int = 0) -> None:
        self._rx_assemblers[can_id] = (start_seq & MAX_SEQ, bytearray())

    def _assembler_append(
            self,
            can_id: int,
            seq: int,
            is_last: bool,
            data62: bytes,
        ):
            """
            Custom-TP 수신 조립 (REPL 용도)
            APP_PDU = [Len(4B LE)] + [MsgID(1B)] + Payload(+padding...)
            여기서는 zero padding 제거하지 않고,
            Length 필드만 신뢰해서 payload를 자른다.
            """
            expected, buf = self._rx_assemblers.get(can_id, (0, bytearray()))

            if not is_last:
                # MIDDLE 프레임: SEQ 검사 (간단 버전, 불일치 시 리셋 후 새로 시작)
                if seq != expected:
                    buf = bytearray()
                    expected = seq

                buf.extend(data62)
                expected = (expected + 1) & MAX_SEQ
                self._rx_assemblers[can_id] = (expected, buf)
                return None

            else:
                # LAST 프레임
                buf.extend(data62)

                # 새 포맷: APP_PDU = [Len(4B LE)][MsgID(1B)][Payload...]
                if len(buf) >= 5:
                    length = (
                        int(buf[0])
                        | (int(buf[1]) << 8)
                        | (int(buf[2]) << 16)
                        | (int(buf[3]) << 24)
                    )
                    app_msg_id = buf[4]

                    if length < 0:
                        length = 0

                    raw = buf[5:]  # zero padding 제거하지 않음

                    if length <= len(raw):
                        payload = raw[:length]
                    else:
                        # 선언된 길이가 실제 데이터보다 크면, 있는 만큼만 사용
                        payload = raw
                        length = len(payload)

                elif len(buf) >= 1:
                    # 구버전 포맷 fallback: [MsgID][Payload...]
                    app_msg_id = buf[0]
                    # 구버전은 padding 없다는 가정이 많아서 여기만 rstrip 유지해도 되고,
                    # 완전하게 padding 유지하고 싶으면 아래 한 줄도:
                    # payload = buf[1:]
                    payload = buf[1:].rstrip(b"\x00")
                else:
                    app_msg_id = 0
                    payload = b""

                text = payload.decode("ascii", errors="ignore")

                # 조립 완료 후 리셋
                self._assembler_reset(can_id, start_seq=0)
                return (text, app_msg_id)

    def _on_frame_dispatch(self, payload_64b: bytes, can_id: int):
        """pcan_manager에서 콜백된 64B 페이로드를 Custom-TP 규약에 따라 조립."""
        if len(payload_64b) < 2:
            return

        hdr1 = payload_64b[0]
        hdr2 = payload_64b[1]
        is_last = bool(hdr1 & HDR_LAST_BIT)
        seq_high = hdr1 & SEQ_HIGH_MASK
        seq_low = hdr2
        seq = ((seq_high << 8) | seq_low) & MAX_SEQ

        data62 = payload_64b[2:64]
        res = self._assembler_append(can_id, seq, is_last, data62)
        if res and self._on_rx_cb:
            text, app_msg_id = res
            self._on_rx_cb(text, app_msg_id)

    def start_repl(self, on_rx: Callable[[str, int], None]) -> None:
        """수신 REPL: 완성된 APP_PDU를 텍스트로 콜백."""
        self._on_rx_cb = on_rx
        self._rx_assemblers.clear()
        mgr = self._require_mgr()
        if hasattr(mgr, "start_rx"):
            mgr.start_rx(self._on_frame_dispatch)  # type: ignore[attr-defined]

    def stop_repl(self) -> None:
        mgr = self._mgr
        if not mgr:
            self._rx_assemblers.clear()
            return
        if hasattr(mgr, "stop_rx"):
            try:
                mgr.stop_rx()  # type: ignore[attr-defined]
            except Exception:
                pass
        self._rx_assemblers.clear()

    # ========================= 유틸 =========================
    @staticmethod
    def load_text_file(path: str) -> List[str]:
        try:
            with open(path, "r", encoding="utf-8") as f:
                content = f.read()
        except UnicodeDecodeError:
            with open(path, "r", encoding="cp949", errors="ignore") as f:
                content = f.read()

        lines: List[str] = []
        for s in content.splitlines():
            t = s.rstrip("\r\n")
            if not t.strip():
                continue
            if t.lstrip().startswith("%"):
                continue
            lines.append(t)
        return lines

    @staticmethod
    def parse_can_id(x: str) -> int:
        x = x.strip().lower()
        base = 16 if x.startswith("0x") else 10
        v = int(x, base)
        if not (0 <= v < (1 << 11)):
            raise ValueError("CAN 11-bit ID 범위(0..0x7FF) 초과")
        return v

    @staticmethod
    def parse_u8(x: str) -> int:
        x = x.strip().lower()
        base = 16 if x.startswith("0x") else 10
        return int(x, base) & 0xFF


# ========================= 메인(REPL) =========================
def _print_rx_line(text: str, app_msg_id: int):
    sys.stdout.write(f"[MsgID=0x{app_msg_id:02X}] {text}\n")
    sys.stdout.flush()


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="CAN Custom-TP sender (64B FD, 2B HDR, APP_PDU = MsgID+Payload)"
    )
    parser.add_argument("--channel", default=DEFAULT_CHANNEL)
    parser.add_argument("--bitrate-fd", default=DEFAULT_BITRATE_FD)
    parser.add_argument("--id", dest="can_id", default=hex(DEFAULT_CAN_ID_11BIT))
    parser.add_argument("--app-msg-id", default=hex(DEFAULT_COMMAND_MSG_ID))
    parser.add_argument("--no-lf", action="store_true")
    parser.add_argument("--send", nargs="*", default=None)
    parser.add_argument("--load", default=None)
    parser.add_argument("--ifg-us", type=int, default=DEFAULT_FRAME_GAP_US)
    parser.add_argument("--cmd-gap-ms", type=int, default=DEFAULT_CMD_GAP_MS)
    args = parser.parse_args(argv)

    try:
        can_id = CanCustomTpSender.parse_can_id(str(args.can_id))
        app_msg_id = CanCustomTpSender.parse_u8(str(args.app_msg_id))
    except Exception as e:
        print(f"[ERR] 인자 파싱 실패: {e}", file=sys.stderr)
        return 2

    sender = CanCustomTpSender(
        channel=args.channel,
        bitrate_fd=args.bitrate_fd,
        can_id_11bit=can_id,
        app_msg_id=app_msg_id,
        append_lf=not args.no_lf,
        frame_gap_us=args.ifg_us,
        cmd_gap_ms=args.cmd_gap_ms,
    )

    try:
        sender.connect()
    except Exception as e:
        print(f"[ERR] Connect 실패: {e}", file=sys.stderr)
        return 1

    # 단발 전송
    if args.send:
        try:
            sender.send_lines(args.send)
        except Exception as e:
            print(f"[ERR] 전송 실패: {e}", file=sys.stderr)
            sender.disconnect()
            return 1
        sender.disconnect()
        return 0

    # 파일 전송
    if args.load:
        try:
            lines = CanCustomTpSender.load_text_file(args.load)
            sender.send_lines(lines)
        except Exception as e:
            print(f"[ERR] 파일 전송 실패: {e}")
            sender.disconnect()
            return 1
        sender.disconnect()
        return 0

    # REPL
    print(f"[INFO] Connected. Channel={sender.channel}, LinkID=0x{sender.can_id_11bit:03X}, "
          f"AppMsgID=0x{sender.app_msg_id:02X}, LF={'on' if sender.append_lf else 'off'}")
    print("[INFO] REPL 시작. ':help' ':quit' 등 사용 가능.")

    sender.start_repl(_print_rx_line)

    try:
        import readline  # type: ignore
        readline.set_history_length(1000)
    except Exception:
        pass

    try:
        while True:
            try:
                line = input("CMD> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if not line:
                continue

            if line in (":quit", ":exit"):
                break
            if line == ":help":
                print(
                    "meta commands:\n"
                    "  :help            - 도움말\n"
                    "  :quit | :exit    - 종료\n"
                    "  :load <file>     - 파일 라인 전송\n"
                    "  :id <hex|int>    - Link-layer CAN ID 변경\n"
                    "  :msgid <hex|int> - Application MsgID 변경\n"
                    "  :lf on|off       - 개행 추가 on/off\n"
                )
                continue
            if line.startswith(":load "):
                path = line[6:].strip().strip('"').strip("'")
                if not path:
                    print("[ERR] 경로를 지정하세요.")
                    continue
                if not os.path.isfile(path):
                    print(f"[ERR] 파일 없음: {path}")
                    continue
                try:
                    lines = CanCustomTpSender.load_text_file(path)
                    sender.send_lines(lines)
                    print(f"[INFO] 전송 완료: {len(lines)} lines from {path}")
                except Exception as e:
                    print(f"[ERR] 파일 전송 실패: {e}")
                continue
            if line.startswith(":id "):
                val = line[4:].strip()
                try:
                    sender.can_id_11bit = CanCustomTpSender.parse_can_id(val)
                    print(f"[INFO] CAN ID 변경: 0x{sender.can_id_11bit:03X}")
                except Exception as e:
                    print(f"[ERR] 잘못된 ID: {e}")
                continue
            if line.startswith(":msgid "):
                val = line[7:].strip()
                try:
                    sender.app_msg_id = CanCustomTpSender.parse_u8(val)
                    print(f"[INFO] App MsgID 변경: 0x{sender.app_msg_id:02X}")
                except Exception as e:
                    print(f"[ERR] 잘못된 MsgID: {e}")
                continue
            if line.startswith(":lf "):
                tok = line[4:].strip().lower()
                if tok in ("on", "1", "true", "yes"):
                    sender.append_lf = True
                elif tok in ("off", "0", "false", "no"):
                    sender.append_lf = False
                else:
                    print("[ERR] :lf on|off")
                    continue
                print(f"[INFO] LF = {'on' if sender.append_lf else 'off'}")
                continue

            # 일반 명령 전송
            try:
                sender.send_line(line)
            except Exception as e:
                print(f"[ERR] 전송 실패: {e}")

    finally:
        sender.stop_repl()
        sender.disconnect()
        print("[INFO] Bye.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
