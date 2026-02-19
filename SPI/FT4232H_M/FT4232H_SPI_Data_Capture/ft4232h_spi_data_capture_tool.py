#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ft4232h_spi_data_capture_tool.py  (CLI / 1 frame per file)

목표:
- HOST_INTR(Active-Low) 핸드셰이크로 64KB chunk를 받아서 1프레임(예: 128KB)을 구성
- byteswap32 옵션으로 32-bit 워드 엔디안 보정
- resync_on_start: 시작 시 프레임 경계(=chunk0)를 찾을 때까지 버리고 동기 맞춤
- fast_mode: 캡처 중에는 메모리에 쌓고, 캡처 후 파일 저장(디스크 IO로 인한 fps 저하 방지)

중요 개선점:
- resync 단계에서 raw 데이터를 byteswap 해서 저장하지 않습니다.
  (검사용 view만 만들고, 실제 저장/출력용 byteswap은 최종 단계에서 1번만)
- seq_fail 체크는 expected_next(다음에 와야 할 값) 기준으로 비교합니다.
"""

import os
import sys
import time
import glob
import array
import argparse
from typing import List, Tuple, Optional, Dict, Any

try:
    import ftd2xx as ftd
except ImportError:
    print("ERROR: ftd2xx library not found. Install with: pip install ftd2xx")
    sys.exit(1)

# ======================== FTDI MPSSE Configuration ========================

# MPSSE 기본 설정 바이트들(FTDI 문서/AN108/AN2232 참고)
FTDI_CFG_60MHZ_SYSCLK = b"\x8A"         # Disable Clk Divide by 5 -> 60MHz
FTDI_CFG_NO_ADAPT_CLK = b"\x97"         # Turn Off Adaptive clocking
FTDI_CFG_3PHAS_CLK    = b"\x8C"         # Enable 3-Phase Data Clocking
FTDI_CFG_NO_3PHAS_CLK = b"\x8D"         # Disable 3-Phase Data Clocking
FTDI_CFG_NO_LOOPBACK  = b"\x85"         # Disable loopback
FTDI_CFG_SPI_4PIN_CFG = b"\x80\x08\x0B" # Value=0x08, Dir=0x0B (SCK/MOSI/CS outputs)

# MPSSE Commands
FTDI_CMD_CS_LOW     = b"\x80\x00\x0B"   # CS low
FTDI_CMD_CS_HIGH    = b"\x80\x08\x0B"   # CS high
FTDI_CMD_READ_BYTES = b"\x20"           # Clock Data Bytes In on +ve edge MSB first
FTDI_CMD_READ_GPIO  = b"\x81"           # Read Data bits LowByte

FTDI_MAX_CHUNK = 65536  # MAXSPISIZEFTDI (MPSSE read limit)

# ======================== Default Configuration ========================

DEFAULT_SPI_INDEX = 0
DEFAULT_GPIO_INDEX = 1
DEFAULT_HOST_INTR_MASK = 0xA0

DEFAULT_SPI_CLOCK_HZ = 30_000_000
DEFAULT_USE_3PHASE_CLK = True

# Frame size parameters (예시)
DEFAULT_ADC_SAMPLES = 256
DEFAULT_CHIRPS_PER_BURST = 64
DEFAULT_BURSTS_PER_FRAME = 1
DEFAULT_RX_ANTENNAS = 4

DEFAULT_NUM_FRAMES = 20
DEFAULT_BYTESWAP32 = True
DEFAULT_RESYNC_ON_START = True

DEFAULT_POLL_SLEEP_US = 0
DEFAULT_SETTLE_US = 0

DEFAULT_OUT_DIR = r"Z:\Texas_Instruments\AWRL6844\Python_App\capture_out"
DEFAULT_CLEAN_OLD_BINS = True
DEFAULT_PREVIEW_EVERY = 1
DEFAULT_LOG_EVERY = 1

# Period tracking
DEFAULT_PERIOD_MS = 64.0
DEFAULT_PERIOD_TOL_MS = 15.0
DEFAULT_WARMUP_FRAMES = 3

# Fast mode
DEFAULT_FAST_MODE = True

# (테스트 패턴 FW 기준) byteswap32 ON 상태에서 chunk1 시작 패턴(view 기준)
# i=65536 위치부터 (i-4)&0xFF => 0xFC,0xFD,0xFE,0xFF
CHUNK1_HEAD_VIEW_BSWAP_ON  = b"\xFC\xFD\xFE\xFF"
CHUNK1_HEAD_VIEW_BSWAP_OFF = b"\xFF\xFE\xFD\xFC"


# ======================== Low-level helpers ========================

def list_ftdi_devices() -> List[Tuple[int, str, str]]:
    """
    FTDI 디바이스 목록 반환.
    반환: [(index, description, serial), ...]
    """
    devices: List[Tuple[int, str, str]] = []
    try:
        # createDeviceInfoList()가 되는 환경이면 가장 정확
        try:
            n = ftd.createDeviceInfoList()
        except Exception:
            # 대체 루트(listDevices)
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
                # 한 장치가 열리지 않아도 전체는 계속 진행
                pass
    except Exception:
        pass

    return devices


def set_clk(handle, hz_req: int) -> int:
    """
    60MHz 시스템 클럭 기반으로 SPI SCK 설정.

    MPSSE 공식:
      actual_clk = 60MHz / (2 * (div + 1))

    div 계산:
      div = ceil(60MHz / (2*hz_req)) - 1
    => actual_clk은 요청 hz_req 이하(<=)가 되도록 설정.

    반환: actual_clk(Hz)
    """
    if hz_req > 30_000_000:
        raise ValueError("Max SCK rate is 30MHz")

    # 정수 올림 나눗셈 형태로 divider 계산
    div = max(0, (60_000_000 + 2 * hz_req - 1) // (2 * hz_req) - 1)
    actual_hz = 60_000_000 // (2 * (div + 1))

    # 0x86: Set TCK/SK Divisor (FTDI MPSSE command)
    cmd = bytes((0x86, div & 0xFF, (div >> 8) & 0xFF))
    handle.write(cmd)

    return actual_hz


def set_device(handle,
               clk_speed: int = 15_000_000,
               latency_timer: int = 1,
               rw_timeout_ms: int = 5000,
               use_3phase_clk: bool = False) -> int:
    """
    FTDI 장치를 MPSSE(SPI) 모드로 초기화.

    - resetDevice()
    - USB 파라미터/timeout/latency 설정
    - bitmode로 MPSSE 활성화
    - 60MHz sysclk, adaptive off, 3phase on/off
    - SPI 핀 direction 설정
    - set_clk로 원하는 SPI 클럭 설정
    - loopback off

    반환: actual SPI clock (Hz)
    """
    handle.resetDevice()

    # 혹시 남아있는 RX 데이터 제거(이전 실행 찌꺼기)
    try:
        rx_bytes, _, _ = handle.getStatus()
        if rx_bytes and rx_bytes > 0:
            handle.read(rx_bytes)
    except Exception:
        pass

    # USB 파이프 설정(최대치에 가깝게)
    handle.setUSBParameters(65535, 65535)
    handle.setChars(False, 0, False, 0)
    handle.setTimeouts(rw_timeout_ms, rw_timeout_ms)
    handle.setLatencyTimer(latency_timer)

    # bitmode: 0=reset, 2=MPSSE enable
    handle.setBitMode(0, 0)
    handle.setBitMode(0, 2)
    time.sleep(0.050)

    # MPSSE 공통 설정
    handle.write(FTDI_CFG_60MHZ_SYSCLK)
    handle.write(FTDI_CFG_NO_ADAPT_CLK)

    # 3-phase clock 옵션(고속에서 안정성에 도움될 때가 있음)
    handle.write(FTDI_CFG_3PHAS_CLK if use_3phase_clk else FTDI_CFG_NO_3PHAS_CLK)

    # SPI IO 방향 설정 (SCK/MOSI/CS outputs)
    handle.write(FTDI_CFG_SPI_4PIN_CFG)

    # SPI SCK 설정
    actual_clk = set_clk(handle, clk_speed)
    time.sleep(0.010)

    # 내부 loopback 끔(실제 핀로만 통신)
    handle.write(FTDI_CFG_NO_LOOPBACK)
    time.sleep(0.010)

    return actual_clk


def read_gpio(handle) -> int:
    """
    FTDI GPIO 상태를 1바이트 읽어 정수로 반환.
    """
    handle.write(FTDI_CMD_READ_GPIO)   # "GPIO read" 명령
    res = handle.read(1)               # 1 byte read
    if not res:
        raise TimeoutError("GPIO read timeout/empty")
    return int.from_bytes(res, "big")  # bytes -> int


def spi_read_exact(handle, length: int) -> bytes:
    """
    SPI로 정확히 length 바이트를 읽습니다.

    주의: ftd2xx.read(length)는 length보다 적게 줄 수 있습니다.
          그래서 while로 끝까지 모아야 합니다.
    """
    if length < 1 or length > 0x10000:
        raise ValueError("Length must be between 1 and 65536")

    # MPSSE 명령은 "length-1"을 넣는 형태(FTDI 규격)
    len_bytes = int(length - 1).to_bytes(2, "little")

    # CS LOW -> READ BYTES -> length -> CS HIGH
    cmd = FTDI_CMD_CS_LOW + FTDI_CMD_READ_BYTES + len_bytes + FTDI_CMD_CS_HIGH
    handle.write(cmd)

    out = bytearray()
    while len(out) < length:
        chunk = handle.read(length - len(out))
        if not chunk:
            raise TimeoutError("SPI read timeout/empty")
        out.extend(chunk)

    return bytes(out)


def byteswap32_fast(data: bytes) -> bytes:
    """
    32-bit word 단위로 byteswap (ABCD -> DCBA).

    - 길이가 4의 배수가 아니면 0으로 패딩 후 처리합니다.
    - array('I')가 4바이트를 보장하는 환경이면 가장 빠릅니다.
    """
    if len(data) % 4 != 0:
        data = data + b"\x00" * (4 - (len(data) % 4))

    a = array.array("I")
    if a.itemsize != 4:
        # 매우 드문 케이스(플랫폼이 다를 때) 대비 수동 처리
        b = bytearray(data)
        for i in range(0, len(b), 4):
            b[i:i+4] = b[i:i+4][::-1]
        return bytes(b)

    a.frombytes(data)
    a.byteswap()
    return a.tobytes()


def intr_active_low(gpio_val: int, mask: int) -> bool:
    """
    HOST_INTR active-low 가정.
    (gpio_val & mask) == 0 이면 "ready(LOW)"로 판단.
    """
    return (gpio_val & mask) == 0


def wait_intr_low(dev_gpio,
                  mask: int,
                  timeout_s: float = 5.0,
                  poll_sleep_us: int = 10,
                  settle_us: int = 100) -> int:
    """
    HOST_INTR이 LOW가 될 때까지 대기.

    - poll_sleep_us: 폴링 루프에서 sleep(마이크로초). 0이면 최대 속도로 폴링(대신 CPU 점유↑)
    - settle_us: LOW 감지 직후 약간 기다림(펌웨어/타이밍 여유 확보용)
    """
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


def chunk1_head_view(byteswapped: bool) -> bytes:
    """
    "검사용 view 기준" chunk1 시작 패턴 반환.
    byteswap32 ON이면 view에서 FC FD FE FF를 기대.
    """
    return CHUNK1_HEAD_VIEW_BSWAP_ON if byteswapped else CHUNK1_HEAD_VIEW_BSWAP_OFF


def make_view(data_raw: bytes, byteswapped: bool) -> bytes:
    """
    raw 데이터를 변경하지 않고, 검사용 view를 만듭니다.
    - byteswapped=True면 byteswap32 적용한 결과를 반환
    - byteswapped=False면 raw 그대로 반환
    """
    return byteswap32_fast(data_raw) if byteswapped else data_raw


def looks_like_chunk1_view(chunk_view: bytes, byteswapped: bool) -> bool:
    """
    view 기준으로 chunk1처럼 보이는지(앞 4바이트 시그니처) 검사.
    """
    return len(chunk_view) >= 4 and chunk_view[:4] == chunk1_head_view(byteswapped)


def looks_like_chunk0_view(chunk_view: bytes, byteswapped: bool) -> bool:
    """
    매우 단순한 휴리스틱:
    - chunk0는 chunk1 헤더와 같지 않다.
    (더 정확히는 verify_frame_view 같은 검증을 함께 쓰는 것이 좋음)
    """
    return len(chunk_view) >= 4 and chunk_view[:4] != chunk1_head_view(byteswapped)


def verify_frame_view(frame_view: bytes, sample_checks: int = 256) -> Dict[str, Any]:
    """
    테스트 패턴 FW 기준으로 프레임이 정상인지 검증(=view 기준).

    기대 패턴:
    - byte[0..3] = big-endian frame counter
    - byte[4..15] = 00 01 02 ... 0B
    - byte[i] = (i-4) & 0xFF  (i>=4)  -> 샘플링으로 mismatch 개수 측정

    반환 dict:
      {
        "cnt": int or None,
        "seq_ok": bool,
        "sample_mismatch": int
      }
    """
    res: Dict[str, Any] = {"cnt": None, "seq_ok": False, "sample_mismatch": 0}
    if len(frame_view) < 16:
        return res

    # frame counter (big-endian)
    res["cnt"] = int.from_bytes(frame_view[0:4], "big")

    # 4..15 시퀀스 체크
    res["seq_ok"] = (frame_view[4:16] == bytes(range(0x00, 0x0C)))

    # 프레임 전체에서 균등 샘플링으로 mismatch 측정
    n = len(frame_view)
    if n > 20 and sample_checks > 0:
        step = max((n - 4) // sample_checks, 1)
        mism = 0
        pos = 4
        for _ in range(sample_checks):
            if pos >= n:
                break
            exp = (pos - 4) & 0xFF
            if frame_view[pos] != exp:
                mism += 1
            pos += step
        res["sample_mismatch"] = mism

    return res


# ======================== Capture Core ========================

def capture_1frame_1file(
    spi_index: int,
    gpio_index: int,
    clock_req_hz: int,
    host_intr_mask: int,
    num_frames: int,
    out_folder: str,
    byteswap32: bool,
    resync_on_start: bool,
    poll_sleep_us: int,
    settle_us: int,
    use_3phase_clk: bool,
    # radar config (frame size)
    adc_samples: int,
    chirps_per_burst: int,
    bursts_per_frame: int,
    rx_antennas: int,
    preview_every: int,
    log_every: int,
    clean_old_bins: bool,
    # period tracking
    period_ms: float = 64.0,
    period_tol_ms: float = 15.0,
    warmup_frames: int = 3,
    # fast mode
    fast_mode: bool = True,
    # verify
    sample_checks: int = 256,
):
    """
    캡처 본체 함수.

    핵심 흐름:
    1) 프레임 크기 계산
    2) FTDI SPI/GPIO 장치 열고 초기화
    3) resync_on_start면 chunk0 경계 찾기
    4) 반복 루프:
       - chunk들을 모아 1프레임 구성
       - fast_mode면 메모리에 저장 후 continue
       - normal mode면 byteswap/검증/파일 저장/통계 출력
    """

    # 프레임 크기 계산 (예: chirps*bursts*samples*rx*2bytes)
    frame_size = chirps_per_burst * bursts_per_frame * adc_samples * rx_antennas * 2

    # 한 프레임이 몇 chunk로 구성되는지 계산 (ceil division)
    chunks_per_frame = (frame_size + FTDI_MAX_CHUNK - 1) // FTDI_MAX_CHUNK
    chunks_per_frame = max(chunks_per_frame, 1)

    dev_spi = None
    dev_gpio = None

    try:
        print("\n===== Capture Start =====")
        print(f"SPI index       : {spi_index}")
        print(f"GPIO index      : {gpio_index}")
        print(f"HOST_INTR mask  : 0x{host_intr_mask:02X} (ready when GPIO&mask==0)")
        print(f"byteswap32      : {'ON' if byteswap32 else 'OFF'}")
        print(f"resync_on_start : {'ON' if resync_on_start else 'OFF'}")
        print(f"poll_sleep_us   : {poll_sleep_us} us")
        print(f"settle_us       : {settle_us} us")
        print(f"3-phase clock   : {'ON' if use_3phase_clk else 'OFF'}")
        print("")
        print(f"Frame size calc : chirps({chirps_per_burst}) * bursts({bursts_per_frame}) * adc({adc_samples}) * rx({rx_antennas}) * 2")
        print(f"Frame Size      : {frame_size:,} bytes")
        print(f"Chunks/Frame    : {chunks_per_frame} (max {FTDI_MAX_CHUNK} bytes per chunk)")
        print(f"Expected period : {period_ms:.1f} ms (tol ±{period_tol_ms:.1f} ms)")
        print(f"Warmup frames   : {warmup_frames}")
        print(f"Fast mode       : {'ON (buffer in memory)' if fast_mode else 'OFF'}")
        print("")

        # 출력 폴더 준비
        os.makedirs(out_folder, exist_ok=True)

        # 기존 bin 삭제 옵션
        if clean_old_bins:
            old = glob.glob(os.path.join(out_folder, "*.bin"))
            for fpath in old:
                try:
                    os.remove(fpath)
                except Exception:
                    pass
            print(f"Output folder   : {out_folder} (cleaned {len(old)} old .bin)")
        else:
            print(f"Output folder   : {out_folder}")

        # 장치 열기 + SPI 채널 초기화
        dev_spi = ftd.open(spi_index)
        actual_clk = set_device(dev_spi,
                                clk_speed=clock_req_hz,
                                latency_timer=1,
                                rw_timeout_ms=5000,
                                use_3phase_clk=use_3phase_clk)

        # GPIO 채널이 다른 인덱스면 별도 오픈(읽기만 할 것이지만 MPSSE enable 필요)
        if gpio_index != spi_index:
            dev_gpio = ftd.open(gpio_index)
            set_device(dev_gpio, clk_speed=1_000_000, latency_timer=1, rw_timeout_ms=5000, use_3phase_clk=False)
        else:
            # 같은 인덱스를 쓰면 dev_spi로 GPIO도 읽음
            dev_gpio = dev_spi

        # 실제 SPI 클럭 출력
        if actual_clk != clock_req_hz:
            print(f"SPI clock       : requested {clock_req_hz/1e6:.1f} MHz -> actual {actual_clk/1e6:.1f} MHz")
        else:
            print(f"SPI clock       : {actual_clk/1e6:.1f} MHz")

        print("\nWaiting for sensor streaming...\n")

        # --------- RESYNC ON START ---------
        # pending_chunks에는 "raw(수신 그대로)"만 넣습니다.
        pending_chunks: List[bytes] = []

        if resync_on_start:
            discards = 0
            print("Syncing to frame boundary (searching chunk0)...")

            while True:
                # 1) chunk0 후보 읽기
                try:
                    wait_intr_low(dev_gpio, host_intr_mask,
                                  timeout_s=0.25,
                                  poll_sleep_us=poll_sleep_us,
                                  settle_us=settle_us)
                except TimeoutError:
                    continue

                raw0 = spi_read_exact(dev_spi, min(FTDI_MAX_CHUNK, frame_size))

                # raw0를 view로 바꿔서 검사(하지만 raw0 자체는 그대로 보관)
                v0 = make_view(raw0, byteswap32)

                # 더 정확하게는 verify_frame_view로 seq_ok까지 확인하는 것도 가능
                # 여기서는 chunk0인지 여부를 간단히 체크
                if not looks_like_chunk0_view(v0, byteswap32):
                    discards += 1
                    if discards % 5 == 0:
                        head8 = " ".join(f"{b:02X}" for b in v0[:8])
                        print(f"  resync: discarded {discards} chunks (view head={head8})")
                    continue

                if chunks_per_frame == 1:
                    pending_chunks = [raw0]
                    break

                # 2) chunk1 후보 읽고, chunk1 헤더(view 기준)로 확인
                try:
                    wait_intr_low(dev_gpio, host_intr_mask,
                                  timeout_s=0.25,
                                  poll_sleep_us=poll_sleep_us,
                                  settle_us=settle_us)
                except TimeoutError:
                    discards += 1
                    continue

                raw1 = spi_read_exact(dev_spi, min(FTDI_MAX_CHUNK, frame_size - len(raw0)))
                v1 = make_view(raw1, byteswap32)

                if looks_like_chunk1_view(v1, byteswap32):
                    pending_chunks = [raw0, raw1]
                    break

                discards += 1
                if discards % 5 == 0:
                    head8 = " ".join(f"{b:02X}" for b in v0[:8])
                    print(f"  resync: discarded {discards} chunks (view head={head8})")

            print("Sync OK.\n")

        # --------- CAPTURE LOOP ---------
        frame_count = 0
        total_bytes = 0
        t0 = time.perf_counter()

        prev_frame_t = None
        period_fail = 0

        # seq 체크: expected_next(다음에 와야 할 프레임 카운터)
        expected_next: Optional[int] = None
        seq_fail = 0
        gap_sum = 0

        # 워밍업 관리
        warmup_done = False

        # fast mode면 raw 프레임을 메모리에 저장
        # (주의: num_frames가 크면 메모리 사용량이 커집니다)
        frame_buffer: Optional[List[bytes]] = [] if fast_mode else None

        target = num_frames if num_frames > 0 else float("inf")

        while frame_count < target:
            remain = frame_size
            chunks_to_use = pending_chunks
            pending_chunks = []

            frame_raw = bytearray()  # 프레임 raw 데이터(수신 그대로)

            # ---- chunk0 확보 ----
            if chunks_to_use:
                # resync에서 미리 읽어둔 chunk 사용
                chunk0_raw = chunks_to_use.pop(0)
                frame_t = time.perf_counter()  # 프레임 타임스탬프(동기 맞춘 시점)
            else:
                # HOST_INTR LOW 기다린 뒤 SPI read
                while True:
                    try:
                        wait_intr_low(dev_gpio, host_intr_mask,
                                      timeout_s=0.25,
                                      poll_sleep_us=poll_sleep_us,
                                      settle_us=settle_us)
                        break
                    except TimeoutError:
                        continue

                frame_t = time.perf_counter()
                chunk0_raw = spi_read_exact(dev_spi, min(FTDI_MAX_CHUNK, remain))

            frame_raw.extend(chunk0_raw)
            remain -= len(chunk0_raw)

            # ---- 나머지 chunk들 ----
            while remain > 0:
                if chunks_to_use:
                    chunk_raw = chunks_to_use.pop(0)
                else:
                    while True:
                        try:
                            wait_intr_low(dev_gpio, host_intr_mask,
                                          timeout_s=0.25,
                                          poll_sleep_us=poll_sleep_us,
                                          settle_us=settle_us)
                            break
                        except TimeoutError:
                            continue

                    chunk_raw = spi_read_exact(dev_spi, min(FTDI_MAX_CHUNK, remain))

                frame_raw.extend(chunk_raw)
                remain -= len(chunk_raw)

            frame_count += 1
            total_bytes += frame_size

            # ---- 워밍업 처리 ----
            is_warmup = (frame_count <= warmup_frames)

            # warmup이 끝나는 시점에 통계 리셋(원하시는 정책에 따라 유지해도 됨)
            if (not warmup_done) and (frame_count > warmup_frames):
                warmup_done = True
                period_fail = 0
                seq_fail = 0
                gap_sum = 0
                expected_next = None
                prev_frame_t = None

            # ---- fast_mode: 메모리에만 저장하고 다음 프레임으로 ----
            if fast_mode:
                assert frame_buffer is not None
                frame_buffer.append(bytes(frame_raw))  # raw 그대로 저장

                # Preview (view 기준으로 출력)
                if preview_every > 0 and (frame_count % preview_every) == 0:
                    v = make_view(bytes(frame_raw[:32]), byteswap32)
                    print("[FAST][PREVIEW] " + " ".join(f"{b:02X}" for b in v))

                # 로그(속도)
                if log_every > 0 and (frame_count % log_every) == 0:
                    elapsed = time.perf_counter() - t0
                    fps = frame_count / elapsed if elapsed > 0 else 0.0
                    print(f"[FAST] Frame {frame_count}: {fps:.1f} fps")

                continue

            # ---- normal mode: byteswap 적용 후 저장/검증 ----
            frame_view = make_view(bytes(frame_raw), byteswap32)  # 최종 출력/검증 기준 view

            # 프레임 검증(패턴 FW 기준)
            v = verify_frame_view(frame_view, sample_checks=sample_checks)
            cnt = v["cnt"]

            # seq 체크 (warmup 제외)
            if cnt is not None and (not is_warmup):
                if expected_next is None:
                    # 첫 유효 cnt를 기준으로 "다음에 와야 할 값" 설정
                    expected_next = (cnt + 1) & 0xFFFFFFFF
                else:
                    if cnt != expected_next:
                        seq_fail += 1
                        # gap 계산: 기대값 대비 얼마나 점프했는지
                        delta = (cnt - expected_next) & 0xFFFFFFFF
                        gap_sum += delta if delta != 0 else 1
                        # mismatch면 resync: 다음 기대값을 cnt+1로 재설정
                        expected_next = (cnt + 1) & 0xFFFFFFFF
                    else:
                        expected_next = (expected_next + 1) & 0xFFFFFFFF

            # period 체크 (warmup 제외)
            if prev_frame_t is not None:
                period = (frame_t - prev_frame_t) * 1000.0
                if (abs(period - period_ms) > period_tol_ms) and (not is_warmup):
                    period_fail += 1
            else:
                period = 0.0
            prev_frame_t = frame_t

            # 파일 저장(1 frame per file)
            out_file = os.path.join(out_folder, f"frame_{frame_count:06d}.bin")  # 000001부터
            with open(out_file, "wb") as f:
                f.write(frame_view)

            # Preview
            if preview_every > 0 and (frame_count % preview_every) == 0:
                first32 = frame_view[:32]
                print("[PREVIEW] " + " ".join(f"{b:02X}" for b in first32))

            # Log
            if log_every > 0 and (frame_count % log_every) == 0:
                elapsed = time.perf_counter() - t0
                fps = frame_count / elapsed if elapsed > 0 else 0.0
                period_str = f"{period:.1f}ms" if period > 0 else "-"
                warmup_tag = "[WARMUP] " if is_warmup else ""
                cnt_hex = f"{cnt:08X}" if cnt is not None else "????????"
                print(
                    f"{warmup_tag}Frame {frame_count}: FrameNo={cnt_hex} | "
                    f"seq_ok={int(v['seq_ok'])} sample_mis={v['sample_mismatch']} | "
                    f"period={period_str} period_fail={period_fail} seq_fail={seq_fail} gap_sum={gap_sum} | "
                    f"fps={fps:.2f}"
                )

        # --------- fast_mode 후처리: 파일 저장 + seq 검증 ---------
        capture_elapsed = time.perf_counter() - t0
        fps = frame_count / capture_elapsed if capture_elapsed > 0 else 0.0
        mbps = (total_bytes / (1024 * 1024)) / capture_elapsed if capture_elapsed > 0 else 0.0

        if fast_mode and frame_buffer:
            print(f"\n\n[FAST] Capture done. Processing {len(frame_buffer)} frames...")

            # 저장
            save_start = time.perf_counter()
            for i, raw_frame in enumerate(frame_buffer, start=1):
                frame_view = make_view(raw_frame, byteswap32)
                out_file = os.path.join(out_folder, f"frame_{i:06d}.bin")
                with open(out_file, "wb") as f:
                    f.write(frame_view)

                if i % 20 == 0:
                    print(f"\r[FAST] Saved {i}/{len(frame_buffer)} files...", end="", flush=True)

            save_elapsed = time.perf_counter() - save_start
            print(f"\n[FAST] Files saved in {save_elapsed:.2f}s")

            # seq 검증(패턴 fw의 cnt가 연속인지)
            print("[FAST] Verifying sequence...")
            seq_errors = 0
            prev_cnt = None
            gap_sum = 0

            for i, raw_frame in enumerate(frame_buffer, start=1):
                frame_view = make_view(raw_frame, byteswap32)
                if len(frame_view) >= 4:
                    cnt = int.from_bytes(frame_view[0:4], "big")
                    if prev_cnt is not None:
                        exp = (prev_cnt + 1) & 0xFFFFFFFF
                        if cnt != exp:
                            seq_errors += 1
                            delta = (cnt - exp) & 0xFFFFFFFF
                            gap_sum += delta if delta != 0 else 1
                            if seq_errors <= 5:
                                print(f"  Frame {i}: expected {exp:08X}, got {cnt:08X} (delta={delta})")
                    prev_cnt = cnt

            if seq_errors == 0:
                print("[FAST] Sequence OK!")
            else:
                print(f"[FAST] Sequence errors={seq_errors}, gap_sum={gap_sum}")

            seq_fail = seq_errors

        print("\n===== Capture Done =====")
        print(f"Frames captured : {frame_count}")
        print(f"Total bytes     : {total_bytes:,}")
        print(f"Capture speed   : {mbps:.2f} MB/s")
        print(f"Capture fps     : {fps:.2f}")
        print(f"Saved to        : {out_folder}")

    finally:
        # 장치 닫기(에러가 나도 닫도록 finally에서 처리)
        try:
            if dev_spi:
                dev_spi.close()
        except Exception:
            pass
        try:
            if dev_gpio and dev_gpio != dev_spi:
                dev_gpio.close()
        except Exception:
            pass


# ======================== Argparse ========================

def build_argparser():
    """
    CLI 옵션(명령줄 인자) 정의.
    예)
      python script.py --spi-index 0 --gpio-index 1 --frames 100 --byteswap32
    """
    p = argparse.ArgumentParser(
        description="FT4232H SPI Data Capture Tool (improved/annotated)"
    )

    p.add_argument("--list-devices", action="store_true",
                   help="List FTDI devices and exit")

    p.add_argument("--spi-index", type=int, default=DEFAULT_SPI_INDEX,
                   help=f"SPI device index (default: {DEFAULT_SPI_INDEX})")
    p.add_argument("--gpio-index", type=int, default=DEFAULT_GPIO_INDEX,
                   help=f"GPIO device index (default: {DEFAULT_GPIO_INDEX})")

    # 16진수 입력(0xA0)도 받기 위해 int(x,0) 사용
    p.add_argument("--host-intr-mask", type=lambda x: int(x, 0), default=DEFAULT_HOST_INTR_MASK,
                   help=f"HOST_INTR mask hex (default: 0x{DEFAULT_HOST_INTR_MASK:02X})")

    p.add_argument("--spi-clock-hz", type=int, default=DEFAULT_SPI_CLOCK_HZ,
                   help=f"SPI clock Hz (default: {DEFAULT_SPI_CLOCK_HZ // 1_000_000}MHz)")

    p.add_argument("--3phase-clk", dest="use_3phase_clk", action="store_true",
                   default=DEFAULT_USE_3PHASE_CLK,
                   help="Enable 3-phase clocking")
    p.add_argument("--no-3phase-clk", dest="use_3phase_clk", action="store_false",
                   help="Disable 3-phase clocking")

    # Frame size params
    p.add_argument("--adc-samples", type=int, default=DEFAULT_ADC_SAMPLES)
    p.add_argument("--chirps", type=int, default=DEFAULT_CHIRPS_PER_BURST)
    p.add_argument("--bursts", type=int, default=DEFAULT_BURSTS_PER_FRAME)
    p.add_argument("--rx-antennas", type=int, default=DEFAULT_RX_ANTENNAS)

    # Capture options
    p.add_argument("--frames", type=int, default=DEFAULT_NUM_FRAMES,
                   help="Number of frames (0=infinite)")
    p.add_argument("--byteswap32", action="store_true", default=DEFAULT_BYTESWAP32,
                   help="Apply byteswap32")
    p.add_argument("--no-byteswap32", dest="byteswap32", action="store_false",
                   help="Disable byteswap32")

    p.add_argument("--resync", action="store_true", default=DEFAULT_RESYNC_ON_START,
                   help="Resync on start")
    p.add_argument("--no-resync", dest="resync", action="store_false",
                   help="Disable resync on start")

    # Timing
    p.add_argument("--poll-us", type=int, default=DEFAULT_POLL_SLEEP_US,
                   help="GPIO poll sleep us")
    p.add_argument("--settle-us", type=int, default=DEFAULT_SETTLE_US,
                   help="Settle time after LOW detect us")

    # Output
    p.add_argument("--out-dir", default=DEFAULT_OUT_DIR)
    p.add_argument("--clean", action="store_true", default=DEFAULT_CLEAN_OLD_BINS,
                   help="Remove old .bin files")
    p.add_argument("--no-clean", dest="clean", action="store_false",
                   help="Keep old .bin files")

    p.add_argument("--preview-every", type=int, default=DEFAULT_PREVIEW_EVERY)
    p.add_argument("--log-every", type=int, default=DEFAULT_LOG_EVERY)

    # Period tracking
    p.add_argument("--period-ms", type=float, default=DEFAULT_PERIOD_MS)
    p.add_argument("--period-tol-ms", type=float, default=DEFAULT_PERIOD_TOL_MS)
    p.add_argument("--warmup-frames", type=int, default=DEFAULT_WARMUP_FRAMES)

    # Fast mode
    p.add_argument("--fast-mode", action="store_true", default=DEFAULT_FAST_MODE,
                   help="Buffer frames in memory, write after capture")
    p.add_argument("--no-fast-mode", dest="fast_mode", action="store_false",
                   help="Disable fast mode")

    # Verify
    p.add_argument("--sample-checks", type=int, default=256,
                   help="Sample checks across frame for pattern mismatch count")

    return p


# ======================== Main ========================

def main():
    """
    프로그램 시작점.
    """
    args = build_argparser().parse_args()

    if args.list_devices:
        devs = list_ftdi_devices()
        if not devs:
            print("No FTDI devices found.")
        else:
            print("Detected FTDI devices:")
            for i, desc, sn in devs:
                print(f"  [{i}] {desc} (SN:{sn})")
        return

    capture_1frame_1file(
        spi_index=args.spi_index,
        gpio_index=args.gpio_index,
        clock_req_hz=args.spi_clock_hz,
        host_intr_mask=args.host_intr_mask,
        num_frames=args.frames,
        out_folder=args.out_dir,
        byteswap32=args.byteswap32,
        resync_on_start=args.resync,
        poll_sleep_us=args.poll_us,
        settle_us=args.settle_us,
        use_3phase_clk=args.use_3phase_clk,
        adc_samples=args.adc_samples,
        chirps_per_burst=args.chirps,
        bursts_per_frame=args.bursts,
        rx_antennas=args.rx_antennas,
        preview_every=args.preview_every,
        log_every=args.log_every,
        clean_old_bins=args.clean,
        period_ms=args.period_ms,
        period_tol_ms=args.period_tol_ms,
        warmup_frames=args.warmup_frames,
        fast_mode=args.fast_mode,
        sample_checks=args.sample_checks,
    )


if __name__ == "__main__":
    main()
