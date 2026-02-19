#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ft4222h_spi_data_capture_tool_cl_slave.py  (SPI SLAVE - frame-based read)

FT4222H SPI SLAVE Capture Tool – Single‑block mode for 64KB frames
==================================================================
Master: AWRL6844 (SPI Master, 20MHz, Mode0)
        - Sends 64 KiB per frame (32 chirps × 256 samples × 4 Rx × 2 bytes)
        - Toggles HOST_INTR LOW/HIGH once per frame
Slave : FT4222H (SPI Slave, this script)
        - Waits for LOW, reads exactly 64 KiB, waits for HIGH (optional)
"""

import os
import time
import struct
import argparse
import glob
import array
import sys
from typing import Optional, List, Tuple, Any

import ft4222
from ft4222.SPI import Cpha, Cpol
from ft4222.GPIO import Dir, Port

# ----------------------------------------------------------------------
# Constants & compatibility
# ----------------------------------------------------------------------
API_SAFE_MAX_READ = 65535
DEFAULT_CHUNK_SIZE = 64 * 1024          # 65536, MAXSPISIZEFTDI
DEFAULT_FRAME_SIZE = 32 * 1024          # 65536 (32 chirps * 256 samples * 4 Rx * 2 bytes)


def list_ft4222_devices() -> List[Tuple[int, Any]]:
    out = []
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
            info = f"<error: {e}>"
        out.append((i, info))
    return out


def open_by_index_compat(index: int):
    """Fallback for environments where ft4222.open() does not accept index."""
    if hasattr(ft4222, "open"):
        return ft4222.open(index)
    try:
        try:
            info = ft4222.getDeviceInfoDetail(index, update=False)
        except TypeError:
            info = ft4222.getDeviceInfoDetail(index)
        loc = info.get("location") if isinstance(info, dict) else getattr(info, "location", None)
        if loc is None:
            raise RuntimeError(f"location not found for device index={index}")
        return ft4222.openByLocation(loc)
    except Exception as e:
        raise RuntimeError(f"open device by index={index} failed: {e}")


def open_by_desc_or_index(desc: str, index: Optional[int]):
    try:
        return ft4222.openByDescription(desc)
    except TypeError:
        return ft4222.openByDescription(desc.encode("utf-8"))
    except Exception:
        if index is None:
            raise
        return open_by_index_compat(index)


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


def spi_mode_to_cpol_cpha(mode: int):
    if mode == 0:
        return Cpol.IDLE_LOW, Cpha.CLK_LEADING
    if mode == 1:
        return Cpol.IDLE_LOW, Cpha.CLK_TRAILING
    if mode == 2:
        return Cpol.IDLE_HIGH, Cpha.CLK_LEADING
    if mode == 3:
        return Cpol.IDLE_HIGH, Cpha.CLK_TRAILING
    raise ValueError("SPI mode must be 0..3")


# ----------------------------------------------------------------------
# Endian‑aware byteswap (for SPI big‑endian data on little‑endian hosts)
# ----------------------------------------------------------------------
def byteswap32_if_little_endian(data: bytes) -> bytes:
    """Swap 32‑bit words only if host is little‑endian, otherwise return as is."""
    if sys.byteorder == 'little':
        if len(data) % 4 != 0:
            data = data + b"\x00" * (4 - (len(data) % 4))
        a = array.array('I')
        if a.itemsize != 4:
            b = bytearray(data)
            for i in range(0, len(b), 4):
                b[i:i+4] = b[i:i+4][::-1]
            return bytes(b)
        a.frombytes(data)
        a.byteswap()
        return a.tobytes()
    else:
        return data


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


# ----------------------------------------------------------------------
# GPIO helpers with stability options
# ----------------------------------------------------------------------
def gpio_init_input_all(dev_gpio):
    try:
        dev_gpio.gpio_Init(gpio0=Dir.INPUT, gpio1=Dir.INPUT,
                           gpio2=Dir.INPUT, gpio3=Dir.INPUT)
    except TypeError:
        dev_gpio.gpio_Init(Dir.INPUT, Dir.INPUT, Dir.INPUT, Dir.INPUT)


def gpio_wait_level(dev_gpio, port, level_low, timeout_s,
                    stable_reads=0, stable_sleep_us=20):
    """
    GPIO 핀이 특정 레벨(LOW/HIGH)이 될 때까지 대기.
    - stable_reads: 연속으로 같은 레벨을 읽어야 성공으로 간주하는 횟수
    - stable_sleep_us: 안정화 읽기 사이의 지연 시간 (마이크로초)
    """
    timeout_ms = int(timeout_s * 1000)
    target_level = level_low  # True = LOW, False = HIGH

    def is_level():
        v = dev_gpio.gpio_Read(port)
        is_low = (v is False) or (v == 0)
        return is_low == target_level

    # ------------------------------------------------------------------
    # 하드웨어 GPIO 웨이트 기능이 있는 경우 (gpio_Wait)
    # ------------------------------------------------------------------
    if hasattr(dev_gpio, "gpio_Wait"):
        deadline = time.perf_counter() + timeout_s
        while True:
            # 1차: gpio_Wait로 레벨 도달까지 대기
            try:
                if hasattr(dev_gpio.gpio_Wait, '__call__'):
                    dev_gpio.gpio_Wait(port, target_level, timeout=timeout_ms, sleep=0)
                else:
                    # 구형 API 대응
                    dev_gpio.gpio_Wait(port, target_level, timeout_ms)
            except Exception:
                # 타임아웃 예외 발생 시 재시도 (루프에서 시간 초과 검사)
                pass

            # 2차: 안정화 검증
            if stable_reads > 0:
                all_match = True
                for _ in range(stable_reads):
                    if not is_level():
                        all_match = False
                        break
                    if stable_sleep_us > 0:
                        time.sleep(stable_sleep_us / 1_000_000.0)
                if all_match:
                    return
                # 안정화 실패 → 남은 시간 내에 다시 gpio_Wait로 대기
                remaining = deadline - time.perf_counter()
                if remaining <= 0:
                    raise TimeoutError(f"HOST_INTR {'LOW' if target_level else 'HIGH'} timeout (port={port})")
                timeout_ms = int(remaining * 1000)
                continue  # 다시 gpio_Wait부터 재시도
            else:
                # 안정화 검증 불필요 시 즉시 반환
                return

    # ------------------------------------------------------------------
    # 소프트웨어 폴링 방식 (gpio_Wait 없음)
    # ------------------------------------------------------------------
    else:
        t0 = time.perf_counter()
        while True:
            if is_level():
                if stable_reads > 0:
                    ok = True
                    for _ in range(stable_reads):
                        if stable_sleep_us > 0:
                            time.sleep(stable_sleep_us / 1_000_000.0)
                        if not is_level():
                            ok = False
                            break
                    if ok:
                        return
                    # 안정화 실패 → while 루프 계속
                else:
                    return
            if (time.perf_counter() - t0) > timeout_s:
                state = "LOW" if target_level else "HIGH"
                raise TimeoutError(f"HOST_INTR {state} timeout (port={port})")
            time.sleep(0.0001)  # 100µs 폴링 간격


def gpio_wait_low(dev_gpio, port: Port, timeout_s: float,
                  stable_reads: int = 0, stable_sleep_us: int = 10):
    return gpio_wait_level(dev_gpio, port, True, timeout_s,
                           stable_reads, stable_sleep_us)


def gpio_wait_high(dev_gpio, port: Port, timeout_s: float,
                   stable_reads: int = 0, stable_sleep_us: int = 10):
    return gpio_wait_level(dev_gpio, port, False, timeout_s,
                           stable_reads, stable_sleep_us)


# ----------------------------------------------------------------------
# SPI Slave low‑level access (works with multiple API flavours)
# ----------------------------------------------------------------------
def _find_spi_obj(dev_spi):
    for name in ("spiSlave", "spislave", "SPI", "spi"):
        if hasattr(dev_spi, name):
            obj = getattr(dev_spi, name)
            if any(hasattr(obj, m) for m in ("spiSlave_InitEx", "spiSlave_Init",
                                              "spiSlave_Read", "spiSlave_GetRxStatus")):
                return obj
    return dev_spi


def spi_slave_get_rx_available(spi) -> Optional[int]:
    for name in ("spiSlave_GetRxStatus", "spiSlave_GetRxSize",
                 "spiSlave_GetRxBytes", "spiSlave_RxBytes"):
        if hasattr(spi, name):
            try:
                v = getattr(spi, name)()
                if isinstance(v, tuple):
                    for item in v[::-1]:
                        if isinstance(item, int):
                            return item
                    return None
                if isinstance(v, int):
                    return v
            except Exception:
                return None
    return None


def spi_slave_read_once(spi, nbytes: int) -> bytes:
    candidates = ("spiSlave_Read", "spiSlave_ReadEx",
                  "spiSlave_ReadBytes", "spiSlave_Receive")
    last_err = None
    for name in candidates:
        if hasattr(spi, name):
            fn = getattr(spi, name)
            try:
                r = fn(nbytes)
                if isinstance(r, tuple):
                    for item in r:
                        if isinstance(item, (bytes, bytearray)):
                            return bytes(item)
                    if len(r) >= 2 and isinstance(r[-1], (bytes, bytearray)):
                        return bytes(r[-1])
                    return b""
                return bytes(r)
            except TypeError:
                try:
                    r = fn(nbytes, 0)
                    if isinstance(r, tuple):
                        for item in r:
                            if isinstance(item, (bytes, bytearray)):
                                return bytes(item)
                        if len(r) >= 2 and isinstance(r[-1], (bytes, bytearray)):
                            return bytes(r[-1])
                        return b""
                    return bytes(r)
                except Exception as e:
                    last_err = e
            except Exception as e:
                last_err = e
    if last_err:
        raise RuntimeError(f"spi slave read failed: {last_err}")
    raise RuntimeError("no working spi slave read API found")


def spi_slave_read_exact(spi, nbytes: int, timeout_s: float) -> bytes:
    out = bytearray()
    t0 = time.perf_counter()
    while len(out) < nbytes:
        if (time.perf_counter() - t0) > timeout_s:
            raise TimeoutError(f"read_exact: got {len(out)}/{nbytes} bytes")
        remain = nbytes - len(out)
        want = min(remain, 4096, API_SAFE_MAX_READ)
        b = spi_slave_read_once(spi, want)
        if not b:
            time.sleep(0.0001)
            continue
        out.extend(b)
    return bytes(out)


def spi_slave_flush(spi, max_bytes: int = 2_000_000) -> int:
    flushed = 0
    while flushed < max_bytes:
        avail = spi_slave_get_rx_available(spi)
        if avail is None or avail <= 0:
            break
        take = min(avail, 4096)
        b = spi_slave_read_once(spi, take)
        if not b:
            break
        flushed += len(b)
    return flushed


def spi_slave_init_stable(dev_spi, cpol, cpha, initex_override: Optional[int] = None):
    spi = _find_spi_obj(dev_spi)

    for rn in ("spi_Reset", "spi_ResetTransaction"):
        if hasattr(spi, rn):
            try:
                getattr(spi, rn)()
            except Exception:
                pass

    if hasattr(spi, "spiSlave_InitEx"):
        if initex_override is not None:
            r = spi.spiSlave_InitEx(int(initex_override))
            print(f"[SPI] spiSlave_InitEx({initex_override}) OK ret={r}")
        else:
            ok = False
            for cand in (1, 2, 0):
                try:
                    r = spi.spiSlave_InitEx(int(cand))
                    print(f"[SPI] spiSlave_InitEx({cand}) OK ret={r}")
                    ok = True
                    break
                except Exception as e:
                    print(f"[SPI] spiSlave_InitEx({cand}) failed: {e}")
            if not ok:
                raise RuntimeError("spiSlave_InitEx failed for 1,2,0")
    elif hasattr(spi, "spiSlave_Init"):
        try:
            spi.spiSlave_Init()
            print("[SPI] spiSlave_Init() OK")
        except TypeError:
            spi.spiSlave_Init(0)
            print("[SPI] spiSlave_Init(mode=0) OK")
    else:
        raise RuntimeError("No SPI slave init API found")

    if hasattr(spi, "spiSlave_SetMode"):
        r = spi.spiSlave_SetMode(cpol, cpha)
        print(f"[SPI] spiSlave_SetMode(CPOL={0 if cpol==Cpol.IDLE_LOW else 1}, "
              f"CPHA={0 if cpha==Cpha.CLK_LEADING else 1}) OK ret={r}")

    return spi


# ----------------------------------------------------------------------
# Frame verification
# ----------------------------------------------------------------------
def verify_frame(frame: bytes, expected_cnt: Optional[int], sample_checks: int = 256) -> dict:
    res = {
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


# ----------------------------------------------------------------------
# Main capture routine (single-block or split, based on args)
# ----------------------------------------------------------------------
def capture(args):
    if args.list_devices:
        devs = list_ft4222_devices()
        if not devs:
            print("No FT4222 devices found.")
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

    # ------------------------------------------------------------------
    # 1. 장치 열기
    # ------------------------------------------------------------------
    dev_spi = open_by_desc_or_index(args.spi_desc, args.spi_index)
    dev_gpio = open_by_desc_or_index(args.gpio_desc, args.gpio_index)

    spi = None
    try:
        # ------------------------------------------------------------------
        # 2. FT4222 완전 초기화 (단순 close/reopen 방식)
        #    - _d_slave.py와 동일한 방식으로 FIFO 상태 초기화
        # ------------------------------------------------------------------
        if args.full_reset:
            print("[RESET] Full device close/reopen...")
            try:
                dev_spi.close()
            except Exception:
                pass
            try:
                dev_gpio.close()
            except Exception:
                pass
            time.sleep(0.5)  # 500ms 대기 (USB 리셋 안정화)

            dev_spi = open_by_desc_or_index(args.spi_desc, args.spi_index)
            dev_gpio = open_by_desc_or_index(args.gpio_desc, args.gpio_index)
            print("[RESET] Devices reopened.")

        # ------------------------------------------------------------------
        # 3. 클럭 및 GPIO 초기화
        # ------------------------------------------------------------------
        if hasattr(dev_spi, "setClock") and hasattr(ft4222, "SysClock"):
            try:
                dev_spi.setClock(ft4222.SysClock.CLK_80)
                print("[CLK] setClock(SysClock.CLK_80) OK")
            except Exception as e:
                print(f"[CLK] setClock failed (ignored): {e}")

        gpio_init_input_all(dev_gpio)
        intr_port = port_enum(args.host_intr_port)

        # ------------------------------------------------------------------
        # 4. SPI 슬레이브 모드 초기화
        # ------------------------------------------------------------------
        cpol, cpha = spi_mode_to_cpol_cpha(args.spi_mode)
        spi = spi_slave_init_stable(dev_spi, cpol, cpha, initex_override=args.initex)

        # ------------------------------------------------------------------
        # 5. [필수] 시작 전 HOST_INTR = HIGH 상태 확인
        #    - 타임아웃 발생 시 즉시 종료 (동기화 실패 방지)
        # ------------------------------------------------------------------
        if args.sync_idle_high:
            try:
                gpio_wait_high(dev_gpio, intr_port, args.intr_timeout_s)
                print("[SYNC] HOST_INTR is HIGH (idle) – OK")
            except TimeoutError:
                sys.exit("[ERROR] HOST_INTR is not HIGH at start. "
                         "Please ensure master is idle (CS=HIGH) before running.\n"
                         "Reset the master device or unplug/replug FT4222.")
        else:
            print("[WARNING] --no-sync-idle-high: startup synchronization disabled. "
                  "Data corruption is likely if master CS is already LOW.")

        # ------------------------------------------------------------------
        # 6. 안정화 지연 및 SPI RX FIFO 플러시
        # ------------------------------------------------------------------
        time.sleep(0.001)  # 1ms 안정화 시간
        if args.flush_before_start:
            flushed = spi_slave_flush(spi)
            if flushed > 0:
                print(f"[SPI] RX flush before start: {flushed} bytes discarded")

        # ------------------------------------------------------------------
        # 7. 설정 정보 출력
        # ------------------------------------------------------------------
        print("\n" + "=" * 60)
        print("    FT4222H SPI SLAVE Capture (64KB Frame Mode)")
        print("=" * 60)
        print("Mode           : SPI SLAVE")
        print("                 AWRL6844 = Master (drives SCLK/CS, sends 64KB frames)")
        print("                 FT4222H  = Slave  (receives one block per frame)")
        print("-" * 60)
        print(f"SPI desc       : {args.spi_desc} (fallback index={args.spi_index})")
        print(f"GPIO desc      : {args.gpio_desc} (fallback index={args.gpio_index})")
        print(f"SPI mode       : {args.spi_mode} (CPOL/CPHA)")
        print(f"HOST_INTR      : P{args.host_intr_port} (LOW = data ready)")
        print(f"Wait INTR HIGH : {'YES' if args.wait_intr_high else 'NO'} (per frame)")
        print(f"Frame size     : {args.frame_size:,} bytes")
        print(f"Chunk size     : {args.chunk_size:,} bytes "
              f"{'(≥ frame size → no splitting)' if args.chunk_size >= args.frame_size else '(split into chunks)'}")
        print(f"Byteswap32     : {'ON' if args.byteswap32 else 'OFF'} "
              f"(host={sys.byteorder}-endian)")
        print(f"Fast mode      : {'ON (buffer in memory)' if args.fast_mode else 'OFF'}")
        print(f"Full reset     : {'ON' if args.full_reset else 'OFF'}")
        print(f"OUT dir        : {out_dir}")
        print("=" * 60)
        print("\nWaiting for AWRL6844 (Master) to send data...\n")

        # ------------------------------------------------------------------
        # 8. 메인 캡처 루프
        # ------------------------------------------------------------------
        total_bytes = 0
        start_time = time.perf_counter()
        frame_timeout_cnt = 0
        chunk_timeout_cnt = 0
        expected_cnt = None

        frame_buffer = [] if args.fast_mode else None

        last_stat_time = start_time
        last_stat_bytes = 0
        last_stat_frames = 0

        frame_idx = 0
        max_frames = args.frames if args.frames > 0 else 10**12

        while frame_idx < max_frames:
            if args.flush_each_frame:
                try:
                    gpio_wait_high(dev_gpio, intr_port, args.intr_timeout_s)
                except TimeoutError:
                    pass
                spi_slave_flush(spi)

            remaining = args.frame_size
            frame_chunks = []
            frame_ok = True

            while remaining > 0:
                # [중요] 하강 에지 대기 (프레임 시작)
                try:
                    gpio_wait_low(dev_gpio, intr_port, args.intr_timeout_s,
                                  stable_reads=args.intr_stable_reads,
                                  stable_sleep_us=args.intr_stable_sleep_us)
                except TimeoutError:
                    print(f"[FRAME {frame_idx+1}] Timeout waiting INTR LOW")
                    frame_timeout_cnt += 1
                    frame_ok = False
                    break

                to_read = min(remaining, args.chunk_size)
                try:
                    chunk = spi_slave_read_exact(spi, to_read, args.spi_read_timeout_s)
                except TimeoutError as e:
                    print(f"[FRAME {frame_idx+1}] Read timeout: {e}")
                    chunk_timeout_cnt += 1
                    frame_ok = False
                    break

                frame_chunks.append(chunk)
                remaining -= len(chunk)

                # 프레임/청크 종료 후 HIGH 대기 (동기화 유지)
                if args.wait_intr_high:
                    try:
                        gpio_wait_high(dev_gpio, intr_port, args.intr_timeout_s,
                                       stable_reads=args.intr_stable_reads,
                                       stable_sleep_us=args.intr_stable_sleep_us)
                    except TimeoutError:
                        print(f"[FRAME {frame_idx+1}] Warning: INTR HIGH timeout")

                if args.post_chunk_delay_us > 0:
                    time.sleep(args.post_chunk_delay_us / 1_000_000.0)

            if not frame_ok:
                if args.stop_on_error:
                    print("[STOP] stop_on_error triggered")
                    break
                continue

            raw_frame = b''.join(frame_chunks)
            frame_idx += 1
            total_bytes += len(raw_frame)

            if args.byteswap32:
                frame = byteswap32_if_little_endian(raw_frame)
            else:
                frame = raw_frame

            if args.fast_mode:
                frame_buffer.append(frame)
            else:
                out_path = os.path.join(out_dir, f"adc_data_{frame_idx:06d}.bin")
                with open(out_path, "wb") as f:
                    f.write(frame)

            # ------------------------------------------------------------------
            # 9. 통계 및 프리뷰 출력
            # ------------------------------------------------------------------
            if args.log_every > 0 and (frame_idx % args.log_every) == 0:
                now = time.perf_counter()
                elapsed_interval = now - last_stat_time
                interval_bytes = total_bytes - last_stat_bytes
                interval_frames = frame_idx - last_stat_frames

                interval_mbps = (interval_bytes / (1024 * 1024)) / elapsed_interval if elapsed_interval > 0 else 0.0
                interval_fps = interval_frames / elapsed_interval if elapsed_interval > 0 else 0.0

                total_elapsed = now - start_time
                total_mbps = (total_bytes / (1024 * 1024)) / total_elapsed if total_elapsed > 0 else 0.0
                total_fps = frame_idx / total_elapsed if total_elapsed > 0 else 0.0

                print(f"[SLAVE] Frame {frame_idx}: "
                      f"Interval({interval_mbps:.2f} MB/s, {interval_fps:.1f} fps) | "
                      f"Total({total_mbps:.2f} MB/s, {total_fps:.1f} fps, {total_bytes/(1024*1024):.1f} MB) | "
                      f"TO_frm={frame_timeout_cnt} TO_chk={chunk_timeout_cnt}")

                last_stat_time = now
                last_stat_bytes = total_bytes
                last_stat_frames = frame_idx

            if args.preview_every > 0 and (frame_idx % args.preview_every) == 0:
                pr_raw = raw_frame[:32]
                pr_sw = frame[:32]
                if args.preview_raw:
                    print("[SLAVE][PREVIEW][RAW ] " + " ".join(f"{b:02X}" for b in pr_raw))
                print("[SLAVE][PREVIEW][SWAP] " + " ".join(f"{b:02X}" for b in pr_sw))

                if args.verify_preview:
                    v = verify_frame(frame, expected_cnt, sample_checks=64)
                    cnt = v["cnt"]
                    if expected_cnt is None and cnt is not None:
                        expected_cnt = cnt
                    elif expected_cnt is not None:
                        expected_cnt = (expected_cnt + 1) & 0xFFFFFFFF
                    if (not v["seq_ok"]) or (v["sample_mismatch"] > 0):
                        print(f"[WARN] Pattern mismatch: seq_ok={v['seq_ok']} sample_mis={v['sample_mismatch']} cnt={cnt}")
                        if not args.preview_raw:
                            print("[SLAVE][PREVIEW][RAW ] " + " ".join(f"{b:02X}" for b in pr_raw))
                        if args.stop_on_error:
                            print("[STOP] stop_on_error triggered.")
                            break

        # ------------------------------------------------------------------
        # 10. Fast mode 일괄 저장
        # ------------------------------------------------------------------
        if args.fast_mode and frame_buffer:
            print(f"\n[SLAVE] Capture finished. Saving {len(frame_buffer)} frames...")
            save_start = time.perf_counter()
            for i, frame in enumerate(frame_buffer, start=1):
                out_path = os.path.join(out_dir, f"adc_data_{i:06d}.bin")
                with open(out_path, "wb") as f:
                    f.write(frame)
                if i % 20 == 0:
                    print(f"\r[SLAVE] Saved {i}/{len(frame_buffer)}...", end="", flush=True)
            print()
            print(f"[SLAVE] Save elapsed: {time.perf_counter() - save_start:.2f}s")

        elapsed = time.perf_counter() - start_time
        fps = (frame_idx / elapsed) if elapsed > 0 else 0.0
        mbps = (total_bytes / (1024 * 1024)) / elapsed if elapsed > 0 else 0.0

        print("\n" + "=" * 60)
        print("           CAPTURE SUMMARY (SPI SLAVE MODE)")
        print("=" * 60)
        print(f"Frames captured : {frame_idx}")
        print(f"Total bytes     : {total_bytes:,} ({total_bytes/(1024*1024):.2f} MB)")
        print(f"Capture time    : {elapsed:.2f} s")
        print(f"Throughput      : {mbps:.2f} MB/s")
        print(f"Frame rate      : {fps:.2f} fps")
        print(f"Frame timeouts  : {frame_timeout_cnt}")
        print(f"Chunk timeouts  : {chunk_timeout_cnt}")
        print(f"Output dir      : {out_dir}")
        print("=" * 60)

    finally:
        # ------------------------------------------------------------------
        # 11. 장치 정리
        # ------------------------------------------------------------------
        try:
            dev_spi.close()
        except Exception:
            pass
        try:
            dev_gpio.close()
        except Exception:
            pass

def build_argparser():
    p = argparse.ArgumentParser(
        description="FT4222H SPI SLAVE capture – 64KB frame mode (AWRL6844 master)")

    p.add_argument("--list-devices", action="store_true")

    p.add_argument("--spi-desc", default="FT4222 A")
    p.add_argument("--gpio-desc", default="FT4222 B")
    p.add_argument("--spi-index", type=int, default=0)
    p.add_argument("--gpio-index", type=int, default=1)

    p.add_argument("--spi-mode", type=int, default=0, choices=[0, 1, 2, 3])

    p.add_argument("--initex", type=int, default=None,
                   help="Force spiSlave_InitEx(value). If not set, tries 1→2→0")

    p.add_argument("--host-intr-port", type=int, default=3, choices=[0, 1, 2, 3])
    p.add_argument("--intr-timeout-s", type=float, default=5.0)
    p.add_argument("--intr-stable-reads", type=int, default=0)
    p.add_argument("--intr-stable-sleep-us", type=int, default=10)

    p.add_argument("--wait-intr-high", action="store_true", default=True,
                   help="Wait for HOST_INTR HIGH after each frame/chunk (recommended)")
    p.add_argument("--no-wait-intr-high", dest="wait_intr_high", action="store_false")

    p.add_argument("--frame-size", type=int, default=DEFAULT_FRAME_SIZE,
                   help="Total bytes per frame (default: 65536 for 32 chirps)")
    p.add_argument("--chunk-size", type=int, default=DEFAULT_CHUNK_SIZE,
                   help="Max bytes per SPI transaction (default: 65536)")
    p.add_argument("--frames", type=int, default=100,
                   help="Number of frames to capture (0 = infinite)")

    p.add_argument("--spi-read-timeout-s", type=float, default=10.0,
                   help="Timeout for reading a frame/chunk")

    p.add_argument("--post-chunk-delay-us", type=int, default=0,
                   help="Delay after each chunk before waiting for next LOW")

    p.add_argument("--byteswap32", action="store_true", default=True,
                   help="Swap 32‑bit words (for little‑endian hosts)")
    p.add_argument("--no-byteswap32", dest="byteswap32", action="store_false")

    p.add_argument("--fast-mode", action="store_true", default=True,
                   help="Keep frames in memory, write all at the end")
    p.add_argument("--no-fast-mode", dest="fast_mode", action="store_false")

    p.add_argument("--out-dir", default="capture_out")
    p.add_argument("--clean", action="store_true", default=True)
    p.add_argument("--no-clean", dest="clean", action="store_false")

    p.add_argument("--log-every", type=int, default=10)
    p.add_argument("--preview-every", type=int, default=1)
    p.add_argument("--preview-raw", action="store_true")
    p.add_argument("--verify-preview", action="store_true")
    p.add_argument("--stop-on-error", action="store_true")

    p.add_argument("--sync-idle-high", action="store_true", default=True)
    p.add_argument("--no-sync-idle-high", dest="sync_idle_high", action="store_false")

    p.add_argument("--flush-before-start", action="store_true", default=True)
    p.add_argument("--no-flush-before-start", dest="flush_before_start", action="store_false")

    p.add_argument("--flush-each-frame", action="store_true", default=False)

    p.add_argument("--full-reset", action="store_true", default=True,
                   help="Close and reopen devices before init (clean FIFO state)")
    p.add_argument("--no-full-reset", dest="full_reset", action="store_false")

    return p


if __name__ == "__main__":
    parser = build_argparser()
    args = parser.parse_args()
    capture(args)