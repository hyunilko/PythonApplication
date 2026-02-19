  #!/usr/bin/env python3
"""
file: ft4232h_cg_usb_gui_master.pyy  (FW header = u32_to_be(cnt))  [SAVE PER FRAME + CNT IN FILENAME]

Firmware (가정): 
- 1 frame = 131072 bytes (128KB)
- 2 chunks x 65536 bytes (64KB)
- Frame header: 
    byte[0..3] = u32_to_be(cnt)
    byte[4..]  = (i-4) & 0xFF sequence  (테스트 패턴)

Host (PC): 
- HOST_INTR(active-low) LOW 될 때마다 chunk를 64KB씩 SPI로 읽음
- byteswap32 옵션으로 원래 메모리 바이트 순서를 복원
- 프레임마다 파일 저장: frame_0000_cnt_7C7D7E7F.bin

개선 포인트(중요): 
- Stop 버튼 눌렀을 때 즉시 멈추도록 wait_intr_low에 stop_cb 추가
- byteswap은 '읽은 chunk -> view로 변환' 흐름으로 일관(실수로 2번 swap 방지)
- infinite 모드(Frames=0) 진행률바 UX 개선
"""

import sys
import time
import os
import glob
import array
from datetime import datetime
from dataclasses import dataclass
from typing import Callable, Optional

  # ---- FTDI D2XX 라이브러리 ----
try: 
    import ftd2xx as ftd
except ImportError: 
    print("ERROR: ftd2xx library not found. Install with: pip install ftd2xx")
    sys.exit(1)

  # ---- PyQt6 (GUI/Thread/Signal) ----
from PyQt6.QtCore import QThread, pyqtSignal, QObject, pyqtSlot
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QPlainTextEdit, QLineEdit,
    QCheckBox, QMessageBox, QGroupBox, QFileDialog, QSpinBox,
    QProgressBar
)

                                                # ---------------- Firmware-fixed sizes ----------------
FW_FRAME_SIZE    = 131072                       # 128KB 고정(펌웨어 가정)
CHUNK_SIZE       = 65536                        # 64KB
CHUNKS_PER_FRAME = FW_FRAME_SIZE // CHUNK_SIZE  # 2

  # ======================== FTDI MPSSE Configuration ========================
  # 아래 바이트들은 FTDI MPSSE 명령(데이터시트/AN108 등 참고)

FTDI_CFG_60MHZ_SYSCLK = b'\x8A'          # divide-by-5 끄기 -> 60MHz
FTDI_CFG_NO_ADAPT_CLK = b'\x97'          # adaptive clocking off
FTDI_CFG_3PHAS_CLK    = b'\x8C'          # 3-phase clock on
FTDI_CFG_NO_3PHAS_CLK = b'\x8D'          # 3-phase clock off
FTDI_CFG_NO_LOOPBACK  = b'\x85'          # loopback off
FTDI_CFG_SPI_4PIN_CFG = b'\x80\x08\x0B'  # low byte 값/방향 설정 (SCK/MOSI/CS outputs)

FTDI_CMD_CS_LOW     = b'\x80\x00\x0B'  # CS low
FTDI_CMD_CS_HIGH    = b'\x80\x08\x0B'  # CS high
FTDI_CMD_READ_BYTES = b'\x20'          # clock data bytes in (MSB first)
FTDI_CMD_READ_GPIO  = b'\x81'          # read low-byte GPIO
FTDI_CMD_SEND_IMM   = b'\x87'          # SEND_IMMEDIATE: 응답 즉시 flush


  # ======================== Low-level helpers ========================

def set_clk(handle, hz_req: int) -> int: 
    """
    SPI SCK 설정.
    MPSSE 공식: actual_clk = 60MHz / (2*(div+1))

    div = ceil(60MHz/(2*hz_req)) - 1  -> actual < = requested

    반환: 실제 설정된 actual_clk(Hz)
    """
    if hz_req > 30_000_000: 
        raise ValueError("Max SCK rate is 30MHz")
    if hz_req < = 0:
        raise ValueError("Invalid SCK rate")

      # 정수로 올림(ceil) 구현: (A + B - 1)//B 형태
    div    = max(0, (60_000_000 + 2 * hz_req - 1) // (2 * hz_req) - 1)
    actual = 60_000_000 // (2 * (div + 1))

      # 0x86: set divisor (FTDI 규격)
    cmd = bytes((0x86, div & 0xFF, (div >> 8) & 0xFF))
    handle.write(cmd)
    return actual


def set_device(handle,
               clk_speed_req : int  = 15_000_000,
               latency_timer : int  = 1,
               rw_timeout_ms : int  = 1000,           # ✅ 개선: stop 반응/블로킹 감소 위해 기본 1초
               use_3phase_clk: bool = False) -> int:
    """
    FTDI 장치를 MPSSE 모드로 초기화하고 SPI 설정을 적용.
    반환: 실제 SPI 클럭(Hz)
    """
    handle.resetDevice()

      # (혹시 남아있는 RX 데이터) 제거
    try: 
        rx_bytes, _, _ = handle.getStatus()
        if rx_bytes > 0: 
            handle.read(rx_bytes)
    except Exception: 
        pass

      # USB 전송 파라미터 및 타임아웃 설정
    handle.setUSBParameters(65535, 65535)
    handle.setChars(False, 0, False, 0)
    handle.setTimeouts(rw_timeout_ms, rw_timeout_ms)
    handle.setLatencyTimer(latency_timer)

                             # MPSSE enable
    handle.setBitMode(0, 0)  # reset
    handle.setBitMode(0, 2)  # MPSSE
    time.sleep(0.050)

      # 60MHz + adaptive off
    handle.write(FTDI_CFG_60MHZ_SYSCLK)
    handle.write(FTDI_CFG_NO_ADAPT_CLK)

      # 3-phase clock 옵션
    handle.write(FTDI_CFG_3PHAS_CLK if use_3phase_clk else FTDI_CFG_NO_3PHAS_CLK)

      # SPI 핀 방향 설정(SCK/MOSI/CS output)
    handle.write(FTDI_CFG_SPI_4PIN_CFG)

      # SPI 속도 설정
    actual = set_clk(handle, clk_speed_req)
    time.sleep(0.010)

      # loopback off
    handle.write(FTDI_CFG_NO_LOOPBACK)
    time.sleep(0.010)

    return actual


def read_gpio(handle) -> int: 
    """
    GPIO 상태 1바이트 읽기.
    SEND_IMMEDIATE를 붙여서 응답을 즉시 flush 시킴(지연 감소).
    """
    handle.write(FTDI_CMD_READ_GPIO + FTDI_CMD_SEND_IMM)
    b = handle.read(1)
    if not b: 
        raise TimeoutError("GPIO read timeout (empty)")
    return int.from_bytes(b, "big")


def spi_read_exact(handle, length: int) -> bytes: 
    """
    SPI로 length 바이트를 '정확히' 읽음.
    ftd2xx.read()가 부분 read를 반환할 수 있으므로 while로 누적.
    """
    if length < 1 or length > 0x10000: 
        raise ValueError("Length must be 1..65536")

      # FTDI MPSSE는 length-1을 2바이트 little endian으로 넣음
    len_bytes = int(length - 1).to_bytes(2, "little")

      # CS low -> read bytes -> length -> CS high -> SEND_IMM(바로 내놔)
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
    """
    32-bit(4바이트) 단위로 byteswap(ABCD -> DCBA).
    """
      # 4의 배수 아니면 padding(이번 프로젝트에서는 chunk=64KB라 사실상 항상 4의 배수)
    if len(data) % 4 != 0:
       data           = data + b"\x00" * (4 - (len(data) % 4))

       a           = array.array("I")
    if a.itemsize != 4:
          # 아주 특수한 플랫폼 대비(보통은 여기 안 탐)
        b = bytearray(data)
        for i in range(0, len(b), 4): 
            b[i:i+4] = b[i:i+4][::-1]
        return bytes(b)

    a.frombytes(data)
    a.byteswap()
    return a.tobytes()


def list_ftdi_devices() -> list[tuple[int, str, str]]: 
    """
    연결된 FTDI 디바이스 목록을 [(index, desc, serial), ...] 형태로 반환.
    GUI 콤보박스 채우는 용도.
    """
    devices: list[tuple[int, str, str]] = []
    try    : 
        n = None

          # createDeviceInfoList()가 지원되면 그걸 우선 사용
        try: 
            n = ftd.createDeviceInfoList()
        except Exception: 
            devs = ftd.listDevices()
            if devs is None: 
                n = 0
            elif isinstance(devs, (list, tuple)): 
                n = len(devs)
            else: 
            try : 
                    n = int(devs)
                except Exception: 
                    n = 0

          # 인덱스별 오픈해서 description/serial 읽기
        for i in range(n or 0): 
            try               : 
                dev    = ftd.open(i)
                info   = dev.getDeviceInfo()
                desc   = info.get("description", b"Unknown").decode(errors="ignore")
                serial = info.get("serial", b"").decode(errors="ignore")
                devices.append((i, desc, serial))
                dev.close()
            except Exception: 
                pass

    except Exception: 
        pass

    return devices


def intr_active_low(gpio_val: int, mask: int) -> bool: 
    """
    HOST_INTR active-low 가정.
    gpio_val & mask == 0이면 LOW(ready)로 판단.
    """
    return (gpio_val & mask) == 0


def wait_intr_low(dev_gpio,
                  mask         : int,
                  timeout_s    : float,
                  poll_sleep_us: int,
                  settle_us    : int,
                  stop_cb      : Optional[Callable[[], bool]] = None) -> int: 
    """
    HOST_INTR이 LOW가 될 때까지 대기.

    ✅ 개선: 
    - stop_cb가 True면 즉시 InterruptedError로 탈출(Stop 버튼 즉시 반응)

    poll_sleep_us: 
      - 0이면 매우 빠르게 폴링(대신 CPU 점유↑)
      - 10~100us 정도면 안정적이고 CPU도 적당히 사용

    settle_us: 
      - LOW 감지 직후 약간 기다림(고속에서 FW 타이밍 여유)
    """
    t0   = time.perf_counter()
    last = 0

    sleep_s  = max(poll_sleep_us, 0) / 1_000_000.0
    settle_s = max(settle_us, 0) / 1_000_000.0

    while True: 
          # stop 요청이면 즉시 탈출
        if stop_cb is not None and stop_cb(): 
            raise InterruptedError("Stopped by user")

        last = read_gpio(dev_gpio)

        if intr_active_low(last, mask): 
        if settle_s > 0               : 
                time.sleep(settle_s)
            return last

        if (time.perf_counter() - t0) > timeout_s: 
            raise TimeoutError(f"HOST_INTR LOW timeout (GPIO=0x{last:02X}, mask=0x{mask:02X})")

        if sleep_s > 0: 
            time.sleep(sleep_s)


def chunk1_signature(byteswapped: bool) -> bytes: 
    """
    테스트 FW에서 chunk1 시작 시그니처(view 기준).
    byteswap ON이면 FC FD FE FF 형태를 기대(프로젝트 가정).
    """
    return b"\xFC\xFD\xFE\xFF" if byteswapped else b"\xFF\xFE\xFD\xFC"


def looks_like_frame_start(chunk_view: bytes, byteswapped: bool) -> bool: 
    """
    chunk0(프레임 시작)인지 간단하게 판별하는 휴리스틱(테스트 패턴 FW 기준): 

    - chunk1 signature면 false
    - byte[4..15]가 00..0B이면 true
    """
    if len(chunk_view) < 16: 
        return False

      # chunk1이면 프레임 시작이 아님
    if chunk_view[:4] == chunk1_signature(byteswapped):
        return False

      # 테스트 패턴: 4..15가 00..0B
    return chunk_view[4:16] == bytes(range(0x00, 0x0C))


def clear_bin_files(folder: str) -> int: 
    """
    폴더 내 *.bin 파일 삭제.
    반환: 삭제한 파일 개수
    """
    removed = 0
    for fpath in glob.glob(os.path.join(folder, "*.bin")): 
        try                                              : 
            os.remove(fpath)
            removed += 1
        except Exception: 
            pass
    return removed


def read_chunk_view(dev_spi, byteswap32: bool) -> bytes: 
    """
    ✅ 개선: '읽기(raw) -> 필요시 byteswap -> view 반환'을 한 곳으로 통일.
    이렇게 하면 어디서든 swap이 딱 1번만 일어나도록 유지하기 쉬움.
    """
    raw    = spi_read_exact(dev_spi, CHUNK_SIZE)        # SPI로 64KB 읽기(정확히)
    return byteswap32_fast(raw) if byteswap32 else raw  # 옵션에 따라 view 반환


  # ======================== Capture Worker ========================

@dataclass
class CaptureSettings: 
      # 디바이스 선택
    spi_dev_index : int
    gpio_dev_index: int

      # SPI 설정
    clock_hz      : int
    use_3phase_clk: bool

                         # 캡처 옵션
    num_frames    : int  # 0이면 무한
    host_intr_mask: int  # 예: 0xA0

      # 저장 옵션
    save_to_folder: bool
    out_folder    : str

                      # 연결 방식(사용자 프로젝트 옵션)
    device_type: str  # "AOP" or "FCCSP"

      # 타이밍(폴링/settle)
    poll_sleep_us: int
    settle_us    : int

      # UI 출력 옵션
    preview_every: int
    log_every    : int

      # 데이터 변환/동기
    byteswap32     : bool
    resync_on_start: bool


class SpiCaptureWorker(QObject): 
    """
    Worker는 QThread에서 돌아가며, 실제 캡처 로직을 수행합니다.
    GUI와는 signal로만 통신합니다(스레드 안전).
    """
    status   = pyqtSignal(str)       # 로그/상태 메시지
    error    = pyqtSignal(str)       # 에러 메시지(Trace 포함)
    progress = pyqtSignal(int, int)  # (현재 프레임, 총 프레임)
    preview  = pyqtSignal(bytes)     # 프리뷰 바이트(예: 첫 64바이트)
    finished = pyqtSignal()          # 종료 신호

    def __init__(self, settings: CaptureSettings): 
        super().__init__()
        self.settings = settings
        self._running = False

          # FTDI 핸들
        self._dev_spi  = None
        self._dev_gpio = None

    @pyqtSlot()
    def start(self): 
        """
        캡처 시작(스레드에서 실행).
        """
        self._running = True

          # stop_cb는 wait_intr_low에서 stop 반응을 즉시 하기 위한 콜백
        stop_cb = lambda: (not self._running)

        try: 
              # ---- 시작 로그 ----
            self.status.emit(f"Frame size: {FW_FRAME_SIZE:,} bytes ({CHUNKS_PER_FRAME} x 64KB)")
            self.status.emit(f"HOST_INTR mask: 0x{self.settings.host_intr_mask:02X} (ready when GPIO&mask==0)")
            self.status.emit(f"byteswap32: {'ON' if self.settings.byteswap32 else 'OFF'}")
            self.status.emit(f"Resync on start: {'ON' if self.settings.resync_on_start else 'OFF'}")
            self.status.emit(f"poll_sleep_us: {self.settings.poll_sleep_us} us, settle_us: {self.settings.settle_us} us")
            self.status.emit(f"3-phase clock: {'ON' if self.settings.use_3phase_clk else 'OFF'}")

              # ---- SPI 디바이스 오픈 ----
            self._dev_spi = ftd.open(self.settings.spi_dev_index)
            self.status.emit(f"SPI device opened: index {self.settings.spi_dev_index}")

              # SPI 디바이스 초기화/설정
            actual_clk = set_device(
                self._dev_spi,
                self.settings.clock_hz,
                latency_timer  = 1,
                rw_timeout_ms  = 1000,                         # ✅ 개선: stop 반응 위해 1초 권장(상황에 따라 2000~5000 가능)
                use_3phase_clk = self.settings.use_3phase_clk
            )

              # 실제 적용된 SPI 클럭 로그
            if actual_clk != self.settings.clock_hz:
                self.status.emit(
                    f"SPI configured: requested {self.settings.clock_hz/1e6:.1f} MHz -> actual {actual_clk/1e6:.1f} MHz"
                )
            else: 
                self.status.emit(f"SPI configured: {actual_clk/1e6:.1f} MHz")

              # ---- GPIO 디바이스 오픈 ----
              # AOP에서 SPI와 GPIO가 다른 FTDI 채널인 경우를 고려
            if (self.settings.device_type == "AOP"
                and self.settings.gpio_dev_index != self.settings.spi_dev_index):

                self._dev_gpio = ftd.open(self.settings.gpio_dev_index)

                  # GPIO 읽기만 해도 MPSSE enable은 필요(READ_GPIO 커맨드 사용)
                set_device(self._dev_gpio, 1_000_000, latency_timer=1, rw_timeout_ms=1000, use_3phase_clk=False)

                self.status.emit(f"GPIO device opened: index {self.settings.gpio_dev_index} (separate)")
            else: 
                  # 같은 디바이스로 GPIO 읽기
                self._dev_gpio = self._dev_spi
                self.status.emit("GPIO device: same as SPI device")

              # ---- 출력 폴더 준비 ----
            out_folder = None
            if self.settings.save_to_folder: 
                out_folder = self.settings.out_folder.strip()
                if not out_folder: 
                    raise ValueError("Output folder path is empty")

                os.makedirs(out_folder, exist_ok=True)

                  # 요구사항: 항상 기존 *.bin 삭제
                removed = clear_bin_files(out_folder)
                self.status.emit(f"Cleared existing *.bin in output folder: removed {removed} file(s)")
                self.status.emit(f"Saving per frame to folder: {out_folder} (frame_XXXX_cnt_YYYYYYYY.bin)")
            else: 
                self.status.emit("Output: (not saving)")

            self.status.emit("Waiting for sensor streaming...")

              # ---- 통계 변수 ----
            frame_count = 0
            total_bytes = 0
            start_t     = time.perf_counter()

              # 목표 프레임 수(0이면 무한)
            target_frames      = self.settings.num_frames if self.settings.num_frames > 0 else float("inf")
            total_for_progress = int(target_frames) if target_frames != float("inf") else 0

              # ---- resync: 시작 시 프레임 경계(chunk0)를 찾기 ----
            pending_chunk0_view = None

            if self.settings.resync_on_start: 
                self.status.emit("Syncing to frame boundary (looking for chunk0 header)...")
                discards = 0

                while self._running: 
                      # HOST_INTR LOW 기다림(Stop 즉시 반응)
                    wait_intr_low(self._dev_gpio,
                                  self.settings.host_intr_mask,
                                  timeout_s     = 0.25,                          # ✅ 개선: 짧게 반복(Stop 반응 좋음)
                                  poll_sleep_us = self.settings.poll_sleep_us,
                                  settle_us     = self.settings.settle_us,
                                  stop_cb       = stop_cb)

                      # 64KB 읽기 + 필요시 byteswap 적용한 view 생성
                    chunk_view = read_chunk_view(self._dev_spi, self.settings.byteswap32)

                      # chunk0처럼 보이면 동기 완료
                    if looks_like_frame_start(chunk_view, self.settings.byteswap32): 
                        pending_chunk0_view = chunk_view
                        break

                      # 아니면 버리고 계속 탐색
                       discards     += 1
                    if discards % 5 == 0:
                       b0            = " ".join(f"{x:02X}" for x in chunk_view[:8])
                        self.status.emit(f"Resync: discarded {discards} chunks (head={b0})")

                  # stop으로 빠져나온 경우
                if (not self._running): 
                    raise InterruptedError("Stopped by user during resync")

                  # 실행 중인데도 못 찾으면 오류
                if pending_chunk0_view is None: 
                    raise RuntimeError("Resync failed: no frame boundary detected")

              # ---- 캡처 메인 루프 ----
            while self._running and frame_count < target_frames: 
                  # ---------- chunk0 확보 ----------
                if pending_chunk0_view is not None: 
                    chunk0_view         = pending_chunk0_view
                    pending_chunk0_view = None
                else: 
                    wait_intr_low(self._dev_gpio,
                                  self.settings.host_intr_mask,
                                  timeout_s     = 0.25,
                                  poll_sleep_us = self.settings.poll_sleep_us,
                                  settle_us     = self.settings.settle_us,
                                  stop_cb       = stop_cb)

                    chunk0_view = read_chunk_view(self._dev_spi, self.settings.byteswap32)

                  # resync 모드라면 chunk0 검증(프레임 시작 아닌 경우 재동기)
                if self.settings.resync_on_start and not looks_like_frame_start(chunk0_view, self.settings.byteswap32): 
                    b0 = " ".join(f"{x:02X}" for x in chunk0_view[:8])
                    self.status.emit(f"Lost alignment, resyncing... (head={b0})")

                    discards = 0
                    while self._running: 
                        wait_intr_low(self._dev_gpio,
                                      self.settings.host_intr_mask,
                                      timeout_s     = 0.25,
                                      poll_sleep_us = self.settings.poll_sleep_us,
                                      settle_us     = self.settings.settle_us,
                                      stop_cb       = stop_cb)

                        raw_view = read_chunk_view(self._dev_spi, self.settings.byteswap32)

                        if looks_like_frame_start(raw_view, self.settings.byteswap32): 
                            pending_chunk0_view = raw_view
                            break

                           discards     += 1
                        if discards % 5 == 0:
                           b1            = " ".join(f"{x:02X}" for x in raw_view[:8])
                            self.status.emit(f"Resync: discarded {discards} chunks (head={b1})")

                      # stop이면 종료
                    if not self._running: 
                        break

                      # 동기 찾았으면 다음 루프로 넘어가서 chunk0부터 다시 시작
                    continue

                  # ---------- 프레임 카운터(cnt) 읽기 ----------
                  # chunk0_view의 첫 4바이트는 big-endian frame counter
                frame_no_u32 = int.from_bytes(chunk0_view[0:4], "big")
                first64      = chunk0_view[:64]                         # 프리뷰로 쓸 64바이트

                  # ---------- chunk1 읽기 ----------
                wait_intr_low(self._dev_gpio,
                              self.settings.host_intr_mask,
                              timeout_s     = 0.25,
                              poll_sleep_us = self.settings.poll_sleep_us,
                              settle_us     = self.settings.settle_us,
                              stop_cb       = stop_cb)

                chunk1_view = read_chunk_view(self._dev_spi, self.settings.byteswap32)

                  # chunk1 헤더(테스트 패턴) 확인
                   exp              = chunk1_signature(self.settings.byteswap32)
                if chunk1_view[:4] != exp:
                    self.status.emit(
                        f"Warning: chunk1 head unexpected: "
                        f"{' '.join(f'{x:02X}' for x in chunk1_view[:8])} (exp {exp.hex().upper()})"
                    )
                      # 정책 선택:
                      # - 지금은 '경고만' 하고 저장 진행
                      # - 원하시면 여기서 resync로 강제 복구하도록 바꿀 수도 있습니다.

                  # ---------- 저장(프레임 단위 파일) ----------
                if out_folder is not None: 
                      # 예: frame_0000_cnt_7C7D7E7F.bin
                    fname      = f"frame_{frame_count:04d}_cnt_{frame_no_u32:08X}.bin"
                    frame_file = os.path.join(out_folder, fname)

                    with open(frame_file, "wb") as f: 
                        f.write(chunk0_view)
                        f.write(chunk1_view)

                  # ---------- 통계/진행 ----------
                frame_count += 1
                total_bytes += FW_FRAME_SIZE

                self.progress.emit(frame_count, total_for_progress)

                  # ---------- 프리뷰 출력 ----------
                if self.settings.preview_every > 0 and (frame_count % self.settings.preview_every) == 0:
                    self.preview.emit(first64)

                  # ---------- 로그 출력 ----------
                if self.settings.log_every > 0 and (frame_count % self.settings.log_every) == 0:
                   elapsed                                                                  = time.perf_counter() - start_t
                   mbps                                                                     = (total_bytes / (1024 * 1024)) / elapsed if elapsed > 0 else 0.0
                    self.status.emit(
                        f"Frame {frame_count}: {FW_FRAME_SIZE} bytes | Total: {total_bytes/1024:.1f} KB | "
                        f"{mbps:.2f} MB/s | cnt=0x{frame_no_u32:08X}"
                    )

              # ---- 루프 종료 후 요약 ----
            elapsed = time.perf_counter() - start_t
            mbps    = (total_bytes / (1024 * 1024)) / elapsed if elapsed > 0 else 0.0

            if self._running: 
                self.status.emit(f"Capture complete: {frame_count} frames, {total_bytes} bytes, {mbps:.2f} MB/s")
            else: 
                self.status.emit(f"Capture stopped: {frame_count} frames, {total_bytes} bytes, {mbps:.2f} MB/s")

        except InterruptedError: 
              # Stop 버튼으로 정상 중단한 케이스
            self.status.emit("Capture stopped by user.")

        except Exception as e: 
              # 에러 발생 시 traceback 포함해 GUI에 전달
            import traceback
            self.error.emit(f"Capture error: {e}\n{traceback.format_exc()}")

        finally: 
              # 어떤 경우든 장치 닫기
            self._cleanup()
            self.finished.emit()

    @pyqtSlot()
    def stop(self): 
        """
        Stop 버튼 눌렀을 때 호출됨.
        _running = False로 바꾸면 start() 루프가 멈추게 설계.
        """
        self._running = False

    def _cleanup(self): 
        """
        FTDI 핸들 close(예외 나도 무시).
        """
           try          : 
        if self._dev_spi: 
                self._dev_spi.close()
        except Exception: 
            pass

           try                                               : 
        if self._dev_gpio and self._dev_gpio != self._dev_spi: 
                self._dev_gpio.close()
        except Exception: 
            pass

        self._dev_spi  = None
        self._dev_gpio = None
        self.status.emit("Devices closed")


  # ======================== GUI ========================

class MainWindow(QMainWindow): 
    """
    GUI는 사용자 입력(옵션)과 로그 표시를 담당.
    실제 캡처는 Worker(QThread)에서 수행.
    """
    def __init__(self): 
        super().__init__()
        self.setWindowTitle("AWRL6844 SPI Data Capture Tool (Save per frame + cnt in filename)")
        self.resize(1080, 880)

        self.capture_thread: Optional[QThread]          = None
        self.capture_worker: Optional[SpiCaptureWorker] = None

        self._build_ui()
        self._refresh_devices()

    def _build_ui(self): 
        """
        화면 레이아웃 구성.
        """
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

          # ---------------- Device selection ----------------
        dev_group  = QGroupBox("FTDI Device Selection")
        dev_layout = QHBoxLayout(dev_group)

        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self._refresh_devices)

        self.device_type_combo = QComboBox()
        self.device_type_combo.addItems(["AOP", "FCCSP"])
        self.device_type_combo.setCurrentText("AOP")

        self.spi_combo  = QComboBox()
        self.gpio_combo = QComboBox()

        dev_layout.addWidget(self.btn_refresh)
        dev_layout.addWidget(QLabel("Type:"))
        dev_layout.addWidget(self.device_type_combo)
        dev_layout.addWidget(QLabel("SPI Device:"))
        dev_layout.addWidget(self.spi_combo, 2)
        dev_layout.addWidget(QLabel("GPIO Device:"))
        dev_layout.addWidget(self.gpio_combo, 2)

        root.addWidget(dev_group)

          # ---------------- Firmware info ----------------
        fw_group  = QGroupBox("Firmware Frame Info")
        fw_layout = QHBoxLayout(fw_group)

        lbl = QLabel(f"Fixed Frame Size: {FW_FRAME_SIZE:,} bytes  (= {CHUNKS_PER_FRAME} x 64KB)")
        lbl.setStyleSheet("font-weight: bold;")
        fw_layout.addWidget(lbl)
        fw_layout.addStretch(1)

        root.addWidget(fw_group)

          # ---------------- HOST_INTR ----------------
        intr_group  = QGroupBox("HOST_INTR GPIO Configuration")
        intr_layout = QHBoxLayout(intr_group)

        self.intr_bit_combo = QComboBox()
        self.intr_bit_combo.addItems([
            "Bit 4 (0x10)  (FCCSP typical)",
            "Bit 5 (0x20)  (AOP option)",
            "Bit 7 (0x80)  (AOP option)",
            "Bits 5+7 (0xA0) (legacy)"
        ])
        self.intr_bit_combo.setCurrentIndex(3)  # default 0xA0

        intr_layout.addWidget(QLabel("HOST_INTR mask:"))
        intr_layout.addWidget(self.intr_bit_combo, 2)
        intr_layout.addStretch(1)

        root.addWidget(intr_group)

          # ---------------- Options ----------------
        opt_group  = QGroupBox("Capture Options")
        opt_layout = QHBoxLayout(opt_group)

        self.clock_combo = QComboBox()
        self.clock_combo.addItems(["30000000", "24000000", "15000000", "12000000", "10000000", "6000000", "1000000"])
        self.clock_combo.setCurrentText("30000000")

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

          # ---------------- Output folder ----------------
        out_group  = QGroupBox("Output Folder (1 file per frame: frame_0000_cnt_XXXXXXXX.bin)")
        out_layout = QHBoxLayout(out_group)

        self.cb_save = QCheckBox("Save frames")
        self.cb_save.setChecked(True)

          # 요구사항: 항상 삭제(강제)
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

          # ---------------- Control ----------------
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

          # ---------------- Log ----------------
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
        """
        로그창에 타임스탬프 붙여 출력.
        """
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_view.appendPlainText(f"[{ts}] {msg}")

          # 커서를 끝으로 이동(자동 스크롤)
        c = self.log_view.textCursor()
        c.movePosition(c.MoveOperation.End)
        self.log_view.setTextCursor(c)

    def _refresh_devices(self): 
        """
        FTDI 장치 목록을 다시 검색해서 콤보박스에 채움.
        """
        devices = list_ftdi_devices()

        self.spi_combo.clear()
        self.gpio_combo.clear()

        for idx, desc, serial in devices: 
            label = f"[{idx}] {desc} (SN: {serial})"
            self.spi_combo.addItem(label, idx)
            self.gpio_combo.addItem(label, idx)

          # 기본 선택(디바이스 2개 이상이면 SPI=0, GPIO=1)
        if len(devices) > = 2:
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
        """
        폴더 선택 다이얼로그.
        """
        path = QFileDialog.getExistingDirectory(self, "Select Output Folder", self.out_folder.text())
        if path: 
            self.out_folder.setText(path)

    def _get_host_intr_mask(self) -> int: 
        """
        콤보박스 선택값 -> 마스크 값으로 변환.
        """
        idx   = self.intr_bit_combo.currentIndex()
        masks = [0x10, 0x20, 0x80, 0xA0]
        return masks[idx] if idx < len(masks) else 0xA0

    def _start_capture(self): 
        """
        Start 버튼 클릭: 
        - GUI 옵션을 settings로 모으고
        - Worker/QThread를 생성해 캡처 시작
        """
        if self.spi_combo.count() == 0:
            QMessageBox.warning(self, "Warning", "No FTDI devices found!")
            return

        settings = CaptureSettings(
            spi_dev_index   = self.spi_combo.currentData(),
            gpio_dev_index  = self.gpio_combo.currentData(),
            clock_hz        = int(self.clock_combo.currentText()),
            use_3phase_clk  = self.cb_3phase.isChecked(),
            num_frames      = self.spin_frames.value(),
            host_intr_mask  = self._get_host_intr_mask(),
            save_to_folder  = self.cb_save.isChecked(),
            out_folder      = self.out_folder.text(),
            device_type     = self.device_type_combo.currentText(),
            poll_sleep_us   = self.spin_poll_us.value(),
            settle_us       = self.spin_settle_us.value(),
            preview_every   = self.spin_preview.value(),
            log_every       = self.spin_log.value(),
            byteswap32      = self.cb_byteswap.isChecked(),
            resync_on_start = self.cb_resync.isChecked(),
        )

          # 시작 로그
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

          # 버튼/진행바 상태 변경
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)

          # ✅ 개선: 무한 모드(0)면 busy indicator
        if settings.num_frames == 0:
            self.progress.setRange(0, 0)  # busy
        else: 
            self.progress.setRange(0, settings.num_frames)
            self.progress.setValue(0)

          # 스레드/워커 생성
        self.capture_thread = QThread()
        self.capture_worker = SpiCaptureWorker(settings)
        self.capture_worker.moveToThread(self.capture_thread)

          # 시그널 연결
        self.capture_thread.started.connect(self.capture_worker.start)
        self.capture_worker.status.connect(self._on_status)
        self.capture_worker.error.connect(self._on_error)
        self.capture_worker.progress.connect(self._on_progress)
        self.capture_worker.preview.connect(self._on_preview)
        self.capture_worker.finished.connect(self._on_finished)

        # 스레드 시작
        self.capture_thread.start()

    def _stop_capture(self):
        """
        Stop 버튼 클릭:
        - Worker.stop() 호출 -> _running=False -> wait_intr_low가 즉시 탈출(개선됨)
        """
        if self.capture_worker:
            self.capture_worker.stop()
        self._log("Stop requested...")

    def _on_status(self, msg: str):
        """
        Worker가 status signal을 보내면 호출.
        """
        self.status_label.setText(msg)
        self._log(msg)

    def _on_error(self, msg: str):
        """
        Worker 에러 발생 시 호출.
        """
        self._log(f"ERROR: {msg}")
        QMessageBox.critical(self, "Error", msg)

    def _on_progress(self, current: int, total: int):
        """
        진행률 업데이트.
        total>0이면 정상 진행바, 0이면 infinite(여기서는 busy를 이미 설정).
        """
        if total > 0:
            self.progress.setRange(0, total)
            self.progress.setValue(current)

    def _on_preview(self, data: bytes):
        """
        프리뷰(첫 64바이트) 표시.
        """
        hex_str = " ".join(f"{b:02X}" for b in data)
        self._log(f"[PREVIEW] {hex_str}")

    def _on_finished(self):
        """
        Worker가 finished signal을 보내면 호출.
        스레드 종료/정리 후 버튼 상태 복구.
        """
        if self.capture_thread:
            self.capture_thread.quit()
            self.capture_thread.wait(2000)

        self.capture_thread = None
        self.capture_worker = None

        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)

        # 진행바 busy 해제
        self.progress.setRange(0, 100)
        self.progress.setValue(0)

        self._log("Capture finished")


def main():
    """
    Qt 앱 시작.
    """
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
