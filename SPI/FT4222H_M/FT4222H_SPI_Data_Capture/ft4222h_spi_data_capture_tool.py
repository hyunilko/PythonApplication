#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ft4222h_spi_data_capture_tool.py
AWRL6844EVM SPI(A) Header + UMFT4222EV(FT4222H) CNFMODE0(00) capture tool

주요 기능:
1) 프로그램 시작 시 이전 저장 파일 전부 삭제 (adc_data_*.bin)
2) 128KB(=131072B) 단위로 프레임별 파일 1개 저장: adc_data_000001.bin ...
3) PREVIEW 강화: raw / byteswap 적용 후 둘 다 출력 + 패턴 검증
   - [0..3] = BE frame counter (FW u32_to_be(cnt))
   - [4..]  = (i-4)&0xFF 시퀀스
   - chunk1 시작 4바이트는 (byteswap ON 기준) FC FD FE FF 기대
4) fast_mode: 캡처 중에는 메모리에 쌓고, 캡처 후 파일 저장(디스크 IO 성능 저하 방지)
5) 실시간 데이터 전송률 통계 (MB/s, fps, 총 바이트)
"""

import os
import time
import struct
import argparse
import glob
import array
from datetime import datetime
from typing import Optional, List, Tuple, Any

import ft4222
from ft4222.SPI import Cpha, Cpol
from ft4222.SPIMaster import Mode, Clock, SlaveSelect
from ft4222.GPIO import Dir, Port

API_SAFE_MAX_READ = 65535  # NOTE: translated to English.

# --------------------------
# Device helpers
# --------------------------

def list_ft4222_devices() -> List[Tuple[int, Any]]:
    out: List[Tuple[int, Any]] = []
    try:
        n = ft4222.createDeviceInfoList()
    except Exception:
        return out

    for i in range(n):
        try:
            info = ft4222.getDeviceInfoDetail(i, update=False)
        except TypeError:
            info = ft4222.getDeviceInfoDetail(i)
        except Exception as e:
            info = f"<getDeviceInfoDetail failed: {e}>"
        out.append((i, info))
    return out


def open_by_desc_or_index(desc: str, index: Optional[int]):
    # NOTE: translated to English.
    try:
        return ft4222.openByDescription(desc)
    except TypeError:
        return ft4222.openByDescription(desc.encode("utf-8"))
    except Exception:
        if index is None:
            raise
        return ft4222.open(index)


# --------------------------
# SPI/GPIO utils
# --------------------------

def byteswap32_fast(data: bytes) -> bytes:
    """Fast 32-bit word byteswap using array('I').byteswap()."""
    if len(data) % 4 != 0:
        data = data + b"\x00" * (4 - (len(data) % 4))

    a = array.array("I")
    if a.itemsize != 4:
        b = bytearray(data)
        for i in range(0, len(b), 4):
            b[i:i+4] = b[i:i+4][::-1]
        return bytes(b)

    a.frombytes(data)
    a.byteswap()
    return a.tobytes()


def spi_mode_to_cpol_cpha(mode: int):
    if mode == 0:
        return (Cpol.IDLE_LOW,  Cpha.CLK_LEADING)
    if mode == 1:
        return (Cpol.IDLE_LOW,  Cpha.CLK_TRAILING)
    if mode == 2:
        return (Cpol.IDLE_HIGH, Cpha.CLK_LEADING)
    if mode == 3:
        return (Cpol.IDLE_HIGH, Cpha.CLK_TRAILING)
    raise ValueError("SPI mode must be 0..3")


def ss_enum(ss: int) -> SlaveSelect:
    if ss == 0:
        return SlaveSelect.SS0
    if ss == 1:
        return SlaveSelect.SS1
    if ss == 2:
        return SlaveSelect.SS2
    if ss == 3:
        return SlaveSelect.SS3
    raise ValueError("SS must be 0..3")


def port_enum(p: int) -> Port:
    if p == 0:
        return Port.P0
    if p == 1:
        return Port.P1
    if p == 2:
        return Port.P2
    if p == 3:
        return Port.P3
    raise ValueError("GPIO port must be 0..3")


def set_sysclock_for_target(dev_spi, target_hz: int) -> tuple:
    """
    Set FT4222H system clock to best match target SPI frequency.
    - 30MHz SPI: need 60MHz sysclock (60/2=30)
    - 24MHz SPI: need 48MHz sysclock (48/2=24)
    - 20MHz SPI: need 80MHz sysclock (80/4=20)
    Returns (sysclock_hz, sysclock_name).
    """
    if not hasattr(ft4222, "SysClock") or not hasattr(dev_spi, "setClock"):
        return (60_000_000, "60MHz (default)")

    # Choose sysclock based on target SPI frequency
    if target_hz >= 30_000_000:
        sysclk_enum = ft4222.SysClock.CLK_60
        sysclk_hz = 60_000_000
        sysclk_name = "60MHz"
    elif target_hz >= 24_000_000:
        sysclk_enum = ft4222.SysClock.CLK_48
        sysclk_hz = 48_000_000
        sysclk_name = "48MHz"
    elif target_hz >= 20_000_000:
        sysclk_enum = ft4222.SysClock.CLK_80
        sysclk_hz = 80_000_000
        sysclk_name = "80MHz"
    else:
        sysclk_enum = ft4222.SysClock.CLK_60
        sysclk_hz = 60_000_000
        sysclk_name = "60MHz"

    try:
        dev_spi.setClock(sysclk_enum)
        print(f"[OK] setClock -> {sysclk_name} (for {target_hz/1e6:.1f}MHz SPI)")
    except Exception as e:
        print(f"[WARN] setClock({sysclk_name}) failed: {e}")

    # Verify actual sysclock
    try:
        clk = dev_spi.getClock()
        if clk == ft4222.SysClock.CLK_24:
            sysclk_hz = 24_000_000
            sysclk_name = "24MHz"
        elif clk == ft4222.SysClock.CLK_48:
            sysclk_hz = 48_000_000
            sysclk_name = "48MHz"
        elif clk == ft4222.SysClock.CLK_60:
            sysclk_hz = 60_000_000
            sysclk_name = "60MHz"
        elif clk == ft4222.SysClock.CLK_80:
            sysclk_hz = 80_000_000
            sysclk_name = "80MHz"
    except Exception:
        pass

    return (sysclk_hz, sysclk_name)


def pick_clock_div_for_target(dev_spi, target_hz: int) -> tuple:
    """
    FT4222: SCK = SysClock / DIV
    - Sets sysclock first, then picks best divider
    - Returns (Clock enum, actual SPI frequency, sysclock Hz)
    """
    sys_hz, sys_name = set_sysclock_for_target(dev_spi, target_hz)

    table = [
        (Clock.DIV_2,   sys_hz // 2),
        (Clock.DIV_4,   sys_hz // 4),
        (Clock.DIV_8,   sys_hz // 8),
        (Clock.DIV_16,  sys_hz // 16),
        (Clock.DIV_32,  sys_hz // 32),
        (Clock.DIV_64,  sys_hz // 64),
        (Clock.DIV_128, sys_hz // 128),
        (Clock.DIV_256, sys_hz // 256),
        (Clock.DIV_512, sys_hz // 512),
    ]

    best_div = table[-1][0]
    actual_hz = table[-1][1]
    for div_enum, hz in table:
        if hz <= target_hz:
            best_div = div_enum
            actual_hz = hz
            break

    return (best_div, actual_hz, sys_hz)


def gpio_init_input_all(dev_gpio):
    # NOTE: translated to English.
    try:
        dev_gpio.gpio_Init(gpio0=Dir.INPUT, gpio1=Dir.INPUT, gpio2=Dir.INPUT, gpio3=Dir.INPUT)
    except TypeError:
        dev_gpio.gpio_Init(Dir.INPUT, Dir.INPUT, Dir.INPUT, Dir.INPUT)


def gpio_wait_low(dev_gpio, port: Port, timeout_s: float, stable_reads: int = 0, stable_sleep_us: int = 20):
    """
    HOST_INTR LOW(=False) 대기.
    - gpio_Wait 지원하면 사용
    - 없으면 폴링
    - stable_reads>0이면 LOW 감지 후 추가로 N번 연속 LOW인지 확인(글리치 방지)
    """
    timeout_ms = int(timeout_s * 1000)

    def is_low() -> bool:
        v = dev_gpio.gpio_Read(port)
        return (v is False) or (v == 0)

    if hasattr(dev_gpio, "gpio_Wait"):
        try:
            dev_gpio.gpio_Wait(port, False, timeout=timeout_ms, sleep=0)
        except TypeError:
            dev_gpio.gpio_Wait(port, False, timeout_ms)

        if stable_reads > 0:
            for _ in range(stable_reads):
                if not is_low():
                    # NOTE: translated to English.
                    return gpio_wait_low(dev_gpio, port, timeout_s, stable_reads, stable_sleep_us)
                if stable_sleep_us > 0:
                    time.sleep(stable_sleep_us / 1_000_000.0)
        return

    t0 = time.perf_counter()
    while True:
        if is_low():
            if stable_reads > 0:
                ok = True
                for _ in range(stable_reads):
                    if stable_sleep_us > 0:
                        time.sleep(stable_sleep_us / 1_000_000.0)
                    if not is_low():
                        ok = False
                        break
                if ok:
                    return
            else:
                return

        if (time.perf_counter() - t0) > timeout_s:
            raise TimeoutError(f"HOST_INTR LOW timeout (port={port})")
        time.sleep(0.0001)


def pick_spi_single_mode():
    candidates = [
        "SPI_IO_SINGLE", "SINGLE", "SINGLE_IO", "IO_SINGLE",
        "SPI_SINGLE", "SINGLE_MODE",
    ]
    for name in candidates:
        if hasattr(Mode, name):
            return getattr(Mode, name)

    for name in dir(Mode):
        if "SINGLE" in name.upper():
            return getattr(Mode, name)

    raise AttributeError(f"Cannot find SINGLE mode in ft4222.SPIMaster.Mode. Available: {dir(Mode)}")


def spi_master_init_compat(dev_spi, clk_div, cpol, cpha, ss):
    io_mode = pick_spi_single_mode()
    try:
        dev_spi.spiMaster_Init(io_mode, clk_div, cpol, cpha, ss)
    except TypeError:
        dev_spi.spiMaster_Init(clk_div, cpol, cpha, ss)


def spi_single_read_compat(dev_spi, length: int, is_end: bool) -> bytes:
    try:
        return dev_spi.spiMaster_SingleRead(length, is_end)
    except TypeError:
        try:
            return dev_spi.spiMaster_SingleRead(length, isEndTransaction=is_end)
        except TypeError:
            b = dev_spi.spiMaster_SingleRead(length)
            if is_end and hasattr(dev_spi, "spiMaster_EndTransaction"):
                dev_spi.spiMaster_EndTransaction()
            return b


def spi_read_keep_cs(dev_spi, nbytes: int) -> bytes:
    out = bytearray()
    remain = nbytes
    while remain > 0:
        part = API_SAFE_MAX_READ if remain > API_SAFE_MAX_READ else remain
        remain -= part
        is_end = (remain == 0)

        b = spi_single_read_compat(dev_spi, part, is_end)
        if len(b) != part:
            raise RuntimeError(f"spiMaster_SingleRead returned {len(b)} bytes, expected {part}")
        out.extend(b)
    return bytes(out)


# --------------------------
# Verify helpers
# --------------------------

def expected_chunk1_signature(byteswapped: bool) -> bytes:
    # NOTE: translated to English.
    # NOTE: translated to English.
    return b"\xFC\xFD\xFE\xFF" if byteswapped else b"\xFF\xFE\xFD\xFC"


def verify_frame(frame: bytes, byteswapped: bool, expected_cnt: Optional[int], sample_checks: int = 256) -> dict:
    """
    FW 패턴 검증:
    - header: [0..3] BE cnt
    - seq:    [4..15] = 00..0B
    - sample: 랜덤/균등 샘플로 (i-4)&0xFF 맞는지 확인
    """
    res = {
        "header_ok": False,
        "seq_ok": False,
        "cnt": None,
        "cnt_ok": None,
        "sample_mismatch": 0,
    }
    if len(frame) < 16:
        return res

    cnt = struct.unpack(">I", frame[0:4])[0]
    res["cnt"] = cnt
    if expected_cnt is not None:
        res["cnt_ok"] = (cnt == expected_cnt)

    res["seq_ok"] = (frame[4:16] == bytes(range(0x00, 0x0C)))
    res["header_ok"] = True  # NOTE: translated to English.

    # NOTE: translated to English.
    # NOTE: translated to English.
    n = len(frame)
    if n > 20 and sample_checks > 0:
        step = max((n - 4) // sample_checks, 1)
        mism = 0
        pos = 4
        for _ in range(sample_checks):
            if pos >= n:
                break
            exp = (pos - 4) & 0xFF
            if frame[pos] != exp:
                mism += 1
            pos += step
        res["sample_mismatch"] = mism

    return res


def safe_clean_previous(out_dir: str, pattern: str = "adc_data_*.bin"):
    os.makedirs(out_dir, exist_ok=True)
    removed = 0
    for p in glob.glob(os.path.join(out_dir, pattern)):
        try:
            os.remove(p)
            removed += 1
        except Exception:
            pass
    return removed


# --------------------------
# Capture
# --------------------------

def capture(args):
    if args.list_devices:
        devs = list_ft4222_devices()
        if not devs:
            print("FT4222 디바이스를 찾지 못했습니다.")
        else:
            print("Detected FT4222 devices:")
            for i, info in devs:
                print(f"  [{i}] {info}")
        return

    out_dir = args.out_dir
    os.makedirs(out_dir, exist_ok=True)
    if args.clean:
        removed = safe_clean_previous(out_dir, "adc_data_*.bin")
        print(f"[CLEAN] Removed {removed} file(s) in: {out_dir}")

    # Open devices
    dev_spi = open_by_desc_or_index(args.spi_desc, args.spi_index)
    dev_gpio = open_by_desc_or_index(args.gpio_desc, args.gpio_index)

    try:
        for fn, val in (("setSuspendOut", False), ("setWakeUpInterrupt", False)):
            if hasattr(dev_gpio, fn):
                try:
                    getattr(dev_gpio, fn)(val)
                except Exception:
                    pass

        # GPIO init
        gpio_init_input_all(dev_gpio)
        intr_port = port_enum(args.host_intr_port)

        # SPI init
        cpol, cpha = spi_mode_to_cpol_cpha(args.spi_mode)
        clk_div, actual_spi_hz, sys_hz = pick_clock_div_for_target(dev_spi, args.spi_clock_hz)
        spi_master_init_compat(dev_spi, clk_div, cpol, cpha, ss_enum(args.ss))

        if actual_spi_hz != args.spi_clock_hz:
            print(f"[INFO] SPI clock: requested {args.spi_clock_hz/1e6:.1f}MHz -> actual {actual_spi_hz/1e6:.1f}MHz (sysclk={sys_hz/1e6:.0f}MHz)")
        else:
            print(f"[INFO] SPI clock: {actual_spi_hz/1e6:.1f}MHz (sysclk={sys_hz/1e6:.0f}MHz)")

        # Summary
        print("\n===== FT4222H Capture (CNFMODE0) =====")
        print(f"SPI desc       : {args.spi_desc} (fallback index={args.spi_index})")
        print(f"GPIO desc      : {args.gpio_desc} (fallback index={args.gpio_index})")
        print(f"SPI mode       : {args.spi_mode}  (Target: POL0/PHA0 => mode0)")
        print(f"SPI target     : {args.spi_clock_hz/1e6:.1f} MHz (divider auto)")
        print(f"SS             : SS{args.ss}")
        print(f"HOST_INTR      : P{args.host_intr_port} (LOW=ready)")
        print(f"Frame size     : {args.frame_size:,} bytes")
        print(f"Chunk size     : {args.chunk_size:,} bytes (expect 2 chunks/frame)")
        print(f"Byteswap32     : {'ON' if args.byteswap32 else 'OFF'}")
        print(f"Fast mode      : {'ON (buffer in memory)' if args.fast_mode else 'OFF'}")
        print(f"OUT dir        : {out_dir}")
        print(f"Expected period: {args.period_ms:.1f} ms (tol ±{args.period_tol_ms:.1f} ms)")
        print("=====================================\n")

        total_bytes = 0
        start = time.perf_counter()
        prev_frame_t = None
        period_fail = 0
        seq_fail = 0
        gap = 0
        timeout_cnt = 0
        expected_cnt = None

        # fast_mode: 메모리에 버퍼링
        frame_buffer: Optional[List[bytes]] = [] if args.fast_mode else None

        # 실시간 통계용
        last_stat_time = start
        last_stat_bytes = 0
        last_stat_frames = 0

        for frame_idx in range(1, args.frames + 1 if args.frames > 0 else 10**12):
            # --- chunk0 ---
            try:
                gpio_wait_low(
                    dev_gpio,
                    intr_port,
                    args.intr_timeout_s,
                    stable_reads=args.intr_stable_reads,
                    stable_sleep_us=args.intr_stable_sleep_us
                )
            except TimeoutError:
                timeout_cnt += 1
                print(f"[TIMEOUT] waiting chunk0 intr low (timeout={timeout_cnt})")
                if args.stop_on_error:
                    break
                continue

            frame_t = time.perf_counter()
            if args.post_intr_delay_us > 0:
                time.sleep(args.post_intr_delay_us / 1_000_000.0)

            raw0 = spi_read_keep_cs(dev_spi, args.chunk_size)

            # --- chunk1 ---
            try:
                gpio_wait_low(
                    dev_gpio,
                    intr_port,
                    args.intr_timeout_s,
                    stable_reads=args.intr_stable_reads,
                    stable_sleep_us=args.intr_stable_sleep_us
                )
            except TimeoutError:
                timeout_cnt += 1
                print(f"[TIMEOUT] waiting chunk1 intr low (timeout={timeout_cnt})")
                if args.stop_on_error:
                    break
                continue

            if args.post_intr_delay_us > 0:
                time.sleep(args.post_intr_delay_us / 1_000_000.0)

            raw1 = spi_read_keep_cs(dev_spi, args.chunk_size)

            # 프레임 조합 (raw 상태)
            raw_frame = raw0 + raw1
            total_bytes += len(raw_frame)

            # period check
            if prev_frame_t is not None:
                period = (frame_t - prev_frame_t) * 1000.0
                if abs(period - args.period_ms) > args.period_tol_ms:
                    period_fail += 1
            else:
                period = 0.0
            prev_frame_t = frame_t

            # --- fast_mode: 메모리에만 저장하고 통계 출력 ---
            if args.fast_mode:
                assert frame_buffer is not None
                frame_buffer.append(raw_frame)

                # 실시간 통계 (log_every 간격)
                if args.log_every > 0 and (frame_idx % args.log_every) == 0:
                    now = time.perf_counter()
                    elapsed_total = now - start
                    elapsed_interval = now - last_stat_time

                    # 구간 통계
                    interval_bytes = total_bytes - last_stat_bytes
                    interval_frames = frame_idx - last_stat_frames
                    interval_mbps = (interval_bytes / (1024 * 1024)) / elapsed_interval if elapsed_interval > 0 else 0.0
                    interval_fps = interval_frames / elapsed_interval if elapsed_interval > 0 else 0.0

                    # 전체 통계
                    total_mbps = (total_bytes / (1024 * 1024)) / elapsed_total if elapsed_total > 0 else 0.0
                    total_fps = frame_idx / elapsed_total if elapsed_total > 0 else 0.0

                    print(
                        f"[FAST] Frame {frame_idx}: "
                        f"Interval({interval_mbps:.2f} MB/s, {interval_fps:.1f} fps) | "
                        f"Total({total_mbps:.2f} MB/s, {total_fps:.1f} fps, {total_bytes/(1024*1024):.1f} MB) | "
                        f"timeout={timeout_cnt}"
                    )

                    last_stat_time = now
                    last_stat_bytes = total_bytes
                    last_stat_frames = frame_idx

                # Preview (view 기준)
                if args.preview_every > 0 and (frame_idx % args.preview_every) == 0:
                    if args.byteswap32:
                        preview_data = byteswap32_fast(raw0[:32])
                    else:
                        preview_data = raw0[:32]
                    print(f"[FAST][PREVIEW] " + " ".join(f"{b:02X}" for b in preview_data))

                continue

            # --- normal mode: byteswap 적용 후 저장/검증 ---
            if args.byteswap32:
                chunk0 = byteswap32_fast(raw0)
                chunk1 = byteswap32_fast(raw1)
            else:
                chunk0 = raw0
                chunk1 = raw1

            frame = chunk0 + chunk1
            if len(frame) != args.frame_size:
                print(f"[WARN] frame_size mismatch: got {len(frame)} expected {args.frame_size}")

            # --- verification ---
            v = verify_frame(frame, args.byteswap32, expected_cnt, sample_checks=args.sample_checks)
            cnt = v["cnt"]
            if expected_cnt is None and cnt is not None:
                expected_cnt = cnt
            else:
                if expected_cnt is not None and cnt is not None and cnt != expected_cnt:
                    seq_fail += 1
                    gap += 1
                if expected_cnt is not None:
                    expected_cnt = (expected_cnt + 1) & 0xFFFFFFFF

            # chunk1 signature check
            exp_sig = expected_chunk1_signature(args.byteswap32)
            chunk1_ok = (chunk1[:4] == exp_sig)

            # save frame (128KB per file)
            out_path = os.path.join(out_dir, f"adc_data_{frame_idx:06d}.bin")
            with open(out_path, "wb") as f:
                f.write(frame)

            elapsed = time.perf_counter() - start
            fps = (frame_idx / elapsed) if elapsed > 0 else 0.0
            mbps = (total_bytes / (1024 * 1024)) / elapsed if elapsed > 0 else 0.0

            if args.preview_every > 0 and (frame_idx % args.preview_every) == 0:
                swp_hex = " ".join(f"{b:02X}" for b in chunk0[:min(args.preview_len, 32)])
                print(f"[PREVIEW][SWAP0] {swp_hex}")

            frame_no_hex = f"0x{cnt:08X}" if cnt is not None else "?"
            print(
                f"Frame {frame_idx}: saved {os.path.basename(out_path)} | "
                f"FrameNo={frame_no_hex} | "
                f"hdr_seq_ok={int(v['seq_ok'])} chunk1_ok={int(chunk1_ok)} "
                f"sample_mis={v['sample_mismatch']} | "
                f"period_fail={period_fail} seq_fail={seq_fail} gap={gap} timeout={timeout_cnt} | "
                f"{mbps:.2f} MB/s, {fps:.2f} fps"
            )

            if args.stop_on_error:
                if (not v["seq_ok"]) or (v["sample_mismatch"] > 0) or (not chunk1_ok):
                    print("[STOP] Pattern mismatch detected (stop_on_error).")
                    break

            if args.frames == 0:
                continue

        # --- 캡처 완료: 통계 ---
        capture_elapsed = time.perf_counter() - start
        capture_fps = (frame_idx / capture_elapsed) if capture_elapsed > 0 else 0.0
        capture_mbps = (total_bytes / (1024 * 1024)) / capture_elapsed if capture_elapsed > 0 else 0.0

        # --- fast_mode 후처리: 파일 저장 + seq 검증 ---
        if args.fast_mode and frame_buffer:
            print(f"\n\n[FAST] Capture done. Processing {len(frame_buffer)} frames...")
            print(f"[FAST] Capture stats: {capture_mbps:.2f} MB/s, {capture_fps:.2f} fps")

            # byteswap + 저장
            save_start = time.perf_counter()
            for i, raw_frame in enumerate(frame_buffer, start=1):
                if args.byteswap32:
                    frame = byteswap32_fast(raw_frame)
                else:
                    frame = raw_frame

                out_path = os.path.join(out_dir, f"adc_data_{i:06d}.bin")
                with open(out_path, "wb") as f:
                    f.write(frame)

                if i % 20 == 0:
                    print(f"\r[FAST] Saved {i}/{len(frame_buffer)} files...", end="", flush=True)

            save_elapsed = time.perf_counter() - save_start
            print(f"\n[FAST] Files saved in {save_elapsed:.2f}s ({len(frame_buffer) / save_elapsed:.1f} files/s)")

            # seq 검증(패턴 FW의 cnt가 연속인지)
            print("[FAST] Verifying sequence...")
            seq_errors = 0
            prev_cnt = None
            gap_sum = 0

            for i, raw_frame in enumerate(frame_buffer, start=1):
                if args.byteswap32:
                    frame = byteswap32_fast(raw_frame)
                else:
                    frame = raw_frame

                if len(frame) >= 4:
                    cnt = int.from_bytes(frame[0:4], "big")
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
            gap = gap_sum

        print("\n" + "=" * 60)
        print("                    CAPTURE SUMMARY")
        print("=" * 60)
        print(f"Frames captured : {frame_idx}")
        print(f"Total bytes     : {total_bytes:,} ({total_bytes / (1024*1024):.2f} MB)")
        print(f"Capture time    : {capture_elapsed:.2f} s")
        print("-" * 60)
        print(f"Throughput      : {capture_mbps:.2f} MB/s")
        print(f"Frame rate      : {capture_fps:.2f} fps")
        print(f"Avg frame time  : {1000.0/capture_fps:.2f} ms" if capture_fps > 0 else "Avg frame time  : N/A")
        print("-" * 60)
        print(f"Period failures : {period_fail}")
        print(f"Sequence errors : {seq_fail}")
        print(f"Gap sum         : {gap}")
        print(f"Timeouts        : {timeout_cnt}")
        print("-" * 60)
        print(f"Output dir      : {out_dir}")
        print("=" * 60)

    finally:
        try:
            if hasattr(dev_spi, "spiMaster_EndTransaction"):
                dev_spi.spiMaster_EndTransaction()
        except Exception:
            pass
        try:
            dev_spi.close()
        except Exception:
            pass
        try:
            dev_gpio.close()
        except Exception:
            pass


def build_argparser():
    p = argparse.ArgumentParser(description="FT4222H(UMFT4222EV) CNFMODE0 SPI capture tool (frame-split + verify)")

    p.add_argument("--list-devices", action="store_true", help="Print FT4222 device list and exit")

    p.add_argument("--spi-desc", default="FT4222 A", help="SPI device description (default: FT4222 A)")
    p.add_argument("--gpio-desc", default="FT4222 B", help="GPIO device description (default: FT4222 B)")
    p.add_argument("--spi-index", type=int, default=0, help="fallback SPI device index")
    p.add_argument("--gpio-index", type=int, default=1, help="fallback GPIO device index")

    p.add_argument("--spi-mode", type=int, default=0, help="SPI mode 0..3 (default: 0)")
    p.add_argument("--spi-clock-hz", type=int, default=24_000_000, help="target SCK Hz (default: 24MHz)")
    p.add_argument("--ss", type=int, default=0, help="SlaveSelect SS0..SS3 (default: 0)")

    p.add_argument("--host-intr-port", type=int, default=3, help="HOST_INTR GPIO port 0..3 (default: 3=P3)")
    p.add_argument("--intr-timeout-s", type=float, default=5.0, help="HOST_INTR LOW wait timeout sec (default: 5)")
    p.add_argument("--post-intr-delay-us", type=int, default=0, help="Delay after INTR LOW in microseconds (default: 0)")

    # GPIO stable read options (for max performance, use --intr-stable-reads 0)
    p.add_argument("--intr-stable-reads", type=int, default=3, help="after LOW, require N stable LOW reads (default: 3, use 0 for max speed)")
    p.add_argument("--intr-stable-sleep-us", type=int, default=20, help="sleep between stable reads (default: 20us)")

    p.add_argument("--frame-size", type=int, default=131072, help="bytes per frame (default: 131072)")
    p.add_argument("--chunk-size", type=int, default=65536, help="bytes per chunk (default: 65536)")
    p.add_argument("--frames", type=int, default=100, help="frames to capture (0=infinite, default: 100)")

    p.add_argument("--byteswap32", action="store_true", help="enable 32-bit byteswap")
    p.add_argument("--no-byteswap32", dest="byteswap32", action="store_false", help="disable 32-bit byteswap")
    p.set_defaults(byteswap32=True)

    # Fast mode: buffer frames in memory, write after capture (reduces disk IO bottleneck)
    p.add_argument("--fast-mode", action="store_true", help="Buffer frames in memory, write after capture (max performance)")
    p.add_argument("--no-fast-mode", dest="fast_mode", action="store_false", help="Disable fast mode (write files during capture)")
    p.set_defaults(fast_mode=True)

    # Output options
    p.add_argument("--out-dir", default="capture_out", help="output directory (default: capture_out)")
    p.add_argument("--clean", action="store_true", help="delete previous adc_data_*.bin in out-dir at start")
    p.add_argument("--no-clean", dest="clean", action="store_false", help="do not delete previous files")
    p.set_defaults(clean=True)

    # Preview and logging options
    p.add_argument("--preview-every", type=int, default=10, help="print preview every N frames (0=off, default: 10)")
    p.add_argument("--preview-len", type=int, default=64, help="preview bytes length (default: 64)")
    p.add_argument("--log-every", type=int, default=10, help="print statistics every N frames in fast mode (default: 10)")
    p.add_argument("--sample-checks", type=int, default=256, help="sample checks across frame (default: 256)")
    p.add_argument("--stop-on-error", action="store_true", help="stop capture if pattern mismatch detected")

    # Period tracking
    p.add_argument("--period-ms", type=float, default=64.0, help="expected frame period in ms (default: 64.0)")
    p.add_argument("--period-tol-ms", type=float, default=10.0, help="period tolerance in ms (default: 10.0)")

    return p


if __name__ == "__main__":
    args = build_argparser().parse_args()
    capture(args)
