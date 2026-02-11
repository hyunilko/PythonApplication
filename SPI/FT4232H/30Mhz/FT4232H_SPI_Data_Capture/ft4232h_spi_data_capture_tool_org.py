  #!/usr/bin/env python3
"""
file: ft4232h_spi_data_capture_tool.py  (CLI / 1 frame per file)

Based on the working GUI reference (ft4232h_cl_usb_gui.py)

Firmware behavior assumed:
- Each frame = frame_size bytes (default 131072)
- Sent in chunks of 65536 bytes (MAXSPISIZEFTDI)
- For each chunk:
    HOST_INTR goes LOW  -> FW runs SPI transfer -> HOST_INTR goes HIGH
- Frame header (test pattern FW):
    byte[0..3] = u32_to_be(cnt)   (big-endian frame counter)
    byte[4.. ] = (i-4) & 0xFF

Host behavior:
- Wait HOST_INTR LOW before EACH chunk, then SPI read exactly that chunk size
- byteswap32 reconstructs original buffer order when FW sends uint32_t* with dataSize=32 and MSB-first

Fixes included:
1) spi_read_exact(): loops until full length is received (avoids partial-read misalignment)
2) resync_on_start (default ON): discards chunks until chunk0 is found (preview always aligned)
3) HOST_INTR mask is ALWAYS the user-selected mask (no hardcode)
4) settle_us delay after detecting LOW (helps at high speed)
5) 1 frame per file output: frame_0000.bin, frame_0001.bin, ...
6) Optional 3-phase clocking (stable at high speed in some setups)

Run:
- python ft4232h_spi_data_capture_tool.py
"""

import os
import sys
import time
import glob
import struct
import array
from datetime import datetime

try:
    import ftd2xx as ftd
except ImportError:
    print("ERROR: ftd2xx library not found. Install with: pip install ftd2xx")
    sys.exit(1)

# ======================== FTDI MPSSE Configuration ========================
FTDI_CFG_60MHZ_SYSCLK = b"\x8A"         # Disable Clk Divide by 5 -> 60MHz
FTDI_CFG_NO_ADAPT_CLK = b"\x97"         # Turn Off Adaptive clocking
FTDI_CFG_3PHAS_CLK    = b"\x8C"         # Enable 3-Phase Data Clocking
FTDI_CFG_NO_3PHAS_CLK = b"\x8D"         # Disable 3-Phase Data Clocking
FTDI_CFG_NO_LOOPBACK  = b"\x85"         # Disable loopback
FTDI_CFG_SPI_4PIN_CFG = b"\x80\x08\x0B" # Value=0x08, Dir=0x0B (SCK/MOSI/CS outputs)

# MPSSE Commands
FTDI_CMD_CS_LOW     = b"\x80\x00\x0B"
FTDI_CMD_CS_HIGH    = b"\x80\x08\x0B"
FTDI_CMD_READ_BYTES = b"\x20"           # Clock Data Bytes In on +ve edge MSB first
FTDI_CMD_READ_GPIO  = b"\x81"           # Read Data bits LowByte

FTDI_MAX_CHUNK = 65536  # MAXSPISIZEFTDI

# Test-pattern heuristic (byteswap32 ON 기준)
# i=65536인 위치부터 (i-4)&0xFF => 0xFC,0xFD,0xFE,0xFF
CHUNK1_HEAD_BSWAP_ON  = b"\xFC\xFD\xFE\xFF"
CHUNK1_HEAD_BSWAP_OFF = b"\xFF\xFE\xFD\xFC"


# ======================== Low-level helpers ========================

def list_ftdi_devices() -> list[tuple[int, str, str]]:
    """List available FTDI devices with (index, description, serial)."""
    devices = []
    try:
        n = None
        try:
            n = ftd.createDeviceInfoList()
        except Exception:
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
                pass
    except Exception:
        pass
    return devices


def set_clk(handle, hz_req: int) -> int:
    """
    Set SPI clock frequency from 60MHz system clock.
    actual_clk = 60MHz / (2 * (div + 1))

    div = ceil(60MHz / (2*hz_req)) - 1  => actual <= requested
    Returns actual clock (Hz).
    """
    if hz_req > 30_000_000:
        raise ValueError("Max SCK rate is 30MHz")

    div = max(0, (60_000_000 + 2 * hz_req - 1) // (2 * hz_req) - 1)
    actual_hz = 60_000_000 // (2 * (div + 1))

    cmd = bytes((0x86, div & 0xFF, (div >> 8) & 0xFF))
    handle.write(cmd)

    return actual_hz


def set_device(handle, clk_speed: int = 15_000_000, latency_timer: int = 1,
               rw_timeout_ms: int = 5000, use_3phase_clk: bool = False) -> int:
    """Initialize FTDI device for MPSSE SPI + GPIO reads. Returns actual clock."""
    handle.resetDevice()

    # Purge stale RX bytes
    try:
        rx_bytes, _, _ = handle.getStatus()
        if rx_bytes and rx_bytes > 0:
            handle.read(rx_bytes)
    except Exception:
        pass

    handle.setUSBParameters(65535, 65535)
    handle.setChars(False, 0, False, 0)
    handle.setTimeouts(rw_timeout_ms, rw_timeout_ms)
    handle.setLatencyTimer(latency_timer)

    handle.setBitMode(0, 0)  # Reset MPSSE
    handle.setBitMode(0, 2)  # Enable MPSSE
    time.sleep(0.050)

    handle.write(FTDI_CFG_60MHZ_SYSCLK)
    handle.write(FTDI_CFG_NO_ADAPT_CLK)

    if use_3phase_clk:
        handle.write(FTDI_CFG_3PHAS_CLK)
    else:
        handle.write(FTDI_CFG_NO_3PHAS_CLK)

    handle.write(FTDI_CFG_SPI_4PIN_CFG)
    actual_clk = set_clk(handle, clk_speed)
    time.sleep(0.010)
    handle.write(FTDI_CFG_NO_LOOPBACK)
    time.sleep(0.010)

    return actual_clk


def read_gpio(handle) -> int:
    """Read GPIO pins state (returns 8-bit value)."""
    handle.write(FTDI_CMD_READ_GPIO)
    res = handle.read(1)
    if not res:
        raise TimeoutError("GPIO read timeout/empty")
    return int.from_bytes(res, "big")


def spi_read_exact(handle, length: int) -> bytes:
    """
    Read EXACTLY 'length' bytes via SPI.
    IMPORTANT: ftd2xx.read(length) can return fewer bytes; we must loop until full.
    """
    if length < 1 or length > 0x10000:
        raise ValueError("Length must be between 1 and 65536")

    len_bytes = int(length - 1).to_bytes(2, "little")
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
    """Fast byteswap per 32-bit word using array('I').byteswap()."""
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


def intr_active_low(gpio_val: int, mask: int) -> bool:
    """HOST_INTR is considered 'ready' when (GPIO & mask) == 0."""
    return (gpio_val & mask) == 0


def wait_intr_low(dev_gpio, mask: int, timeout_s: float = 5.0,
                  poll_sleep_us: int = 10, settle_us: int = 100) -> int:
    """
    Wait until HOST_INTR becomes active-low (GPIO&mask==0).
    settle_us: small delay after detecting LOW (helps at 24MHz+ in some FW timing).
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


def chunk1_head(byteswapped: bool) -> bytes:
    return CHUNK1_HEAD_BSWAP_ON if byteswapped else CHUNK1_HEAD_BSWAP_OFF


def looks_like_chunk1(chunk: bytes, byteswapped: bool) -> bool:
    return len(chunk) >= 4 and chunk[:4] == chunk1_head(byteswapped)


def looks_like_chunk0(chunk: bytes, byteswapped: bool) -> bool:
    """
    Simple heuristic for current test FW:
    - chunk1 starts with FC FD FE FF (byteswap ON)
    - chunk0 starts with frame counter and should not equal chunk1 head
    """
    return len(chunk) >= 4 and chunk[:4] != chunk1_head(byteswapped)


# ======================== CLI Helpers ========================

def input_with_default(prompt: str, default, cast=None):
    s = input(f"{prompt} [{default}]: ").strip()
    if s == "":
        return default
    return cast(s) if cast else type(default)(s)

def input_yesno(prompt: str, default: bool) -> bool:
    d = "y" if default else "n"
    s = input(f"{prompt} [y/n, default={d}]: ").strip().lower()
    if s == "":
        return default
    return s.startswith("y")


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
):
    frame_size = chirps_per_burst * bursts_per_frame * adc_samples * rx_antennas * 2
    chunks_per_frame = (frame_size + FTDI_MAX_CHUNK - 1) // FTDI_MAX_CHUNK
    if chunks_per_frame < 1:
        chunks_per_frame = 1

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
        print("")

        # output folder
        os.makedirs(out_folder, exist_ok=True)
        if clean_old_bins:
            old = glob.glob(os.path.join(out_folder, "*.bin"))
            for f in old:
                try:
                    os.remove(f)
                except Exception:
                    pass
            print(f"Output folder   : {out_folder} (cleaned {len(old)} old .bin)")
        else:
            print(f"Output folder   : {out_folder}")

        # open devices
        dev_spi = ftd.open(spi_index)
        actual_clk = set_device(dev_spi, clk_speed=clock_req_hz, latency_timer=1,
                                rw_timeout_ms=5000, use_3phase_clk=use_3phase_clk)

        if gpio_index != spi_index:
            dev_gpio = ftd.open(gpio_index)
            set_device(dev_gpio, clk_speed=1_000_000, latency_timer=1, rw_timeout_ms=5000, use_3phase_clk=False)
        else:
            dev_gpio = dev_spi

        if actual_clk != clock_req_hz:
            print(f"SPI clock       : requested {clock_req_hz/1e6:.1f} MHz -> actual {actual_clk/1e6:.1f} MHz")
        else:
            print(f"SPI clock       : {actual_clk/1e6:.1f} MHz")
        print("\nWaiting for sensor streaming...\n")

        # --------- RESYNC ON START ---------
        pending_chunks: list[bytes] = []
        if resync_on_start:
            discards = 0
            print("Syncing to frame boundary (searching chunk0)...")
            while True:
                try:
                    wait_intr_low(dev_gpio, host_intr_mask, timeout_s=0.25,
                                  poll_sleep_us=poll_sleep_us, settle_us=settle_us)
                except TimeoutError:
                    # keep polling; do not block the UI/CLI
                    continue

                raw0 = spi_read_exact(dev_spi, min(FTDI_MAX_CHUNK, frame_size))
                if byteswap32:
                    raw0 = byteswap32_fast(raw0)

                if not looks_like_chunk0(raw0, byteswap32):
                    discards += 1
                    if discards % 5 == 0:
                        head8 = " ".join(f"{b:02X}" for b in raw0[:8])
                        print(f"  resync: discarded {discards} chunks (head={head8})")
                    continue

                if chunks_per_frame == 1:
                    pending_chunks = [raw0]
                    break

                # read next chunk and confirm chunk1 head (reduces false alignment)
                try:
                    wait_intr_low(dev_gpio, host_intr_mask, timeout_s=0.25,
                                  poll_sleep_us=poll_sleep_us, settle_us=settle_us)
                except TimeoutError:
                    discards += 1
                    continue

                raw1 = spi_read_exact(dev_spi, min(FTDI_MAX_CHUNK, frame_size - len(raw0)))
                if byteswap32:
                    raw1 = byteswap32_fast(raw1)

                if looks_like_chunk1(raw1, byteswap32):
                    pending_chunks = [raw0, raw1]
                    break

                discards += 1
                if discards % 5 == 0:
                    head8 = " ".join(f"{b:02X}" for b in raw0[:8])
                    print(f"  resync: discarded {discards} chunks (head={head8})")

            print("Sync OK.\n")

        # --------- CAPTURE LOOP ---------
        frame_count = 0
        total_bytes = 0
        t0 = time.perf_counter()

        target = num_frames if num_frames > 0 else float("inf")

        while frame_count < target:
            remain = frame_size
            chunks_to_use = pending_chunks
            pending_chunks = []

            # open per-frame file
            out_file = os.path.join(out_folder, f"frame_{frame_count:04d}.bin")
            first64 = None
            frame_no_hex = "????????"

            with open(out_file, "wb") as f:
                # ---- chunk0 ----
                if chunks_to_use:
                    chunk0 = chunks_to_use.pop(0)
                else:
                    while True:
                        try:
                            wait_intr_low(dev_gpio, host_intr_mask, timeout_s=0.25,
                                          poll_sleep_us=poll_sleep_us, settle_us=settle_us)
                            break
                        except TimeoutError:
                            continue

                    chunk0 = spi_read_exact(dev_spi, min(FTDI_MAX_CHUNK, remain))
                    if byteswap32:
                        chunk0 = byteswap32_fast(chunk0)

                # store preview/frameno
                first64 = chunk0[:64]
                if len(chunk0) >= 4:
                    frame_no_u32 = int.from_bytes(chunk0[0:4], "big")  # after byteswap32, BE is valid
                    frame_no_hex = f"{frame_no_u32:08X}"

                f.write(chunk0)
                remain -= len(chunk0)

                # ---- remaining chunks ----
                while remain > 0:
                    if chunks_to_use:
                        chunk = chunks_to_use.pop(0)
                    else:
                        while True:
                            try:
                                wait_intr_low(dev_gpio, host_intr_mask, timeout_s=0.25,
                                              poll_sleep_us=poll_sleep_us, settle_us=settle_us)
                                break
                            except TimeoutError:
                                continue

                        chunk = spi_read_exact(dev_spi, min(FTDI_MAX_CHUNK, remain))
                        if byteswap32:
                            chunk = byteswap32_fast(chunk)

                    f.write(chunk)
                    remain -= len(chunk)

            frame_count += 1
            total_bytes += frame_size

            # preview
            if preview_every > 0 and (frame_count % preview_every) == 0 and first64 is not None:
                print("[PREVIEW] " + " ".join(f"{b:02X}" for b in first64))

            # log
            if log_every > 0 and (frame_count % log_every) == 0:
                elapsed = time.perf_counter() - t0
                mbps = (total_bytes / (1024 * 1024)) / elapsed if elapsed > 0 else 0.0
                print(f"Frame {frame_count}: {frame_size} bytes | Total {total_bytes/1024:.1f} KB | {mbps:.2f} MB/s | FrameNo={frame_no_hex}")

        elapsed = time.perf_counter() - t0
        mbps = (total_bytes / (1024 * 1024)) / elapsed if elapsed > 0 else 0.0
        print("\n===== Capture Done =====")
        print(f"Frames captured : {frame_count}")
        print(f"Total bytes     : {total_bytes:,}")
        print(f"Avg speed       : {mbps:.2f} MB/s")
        print(f"Saved to        : {out_folder}")

    finally:
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


# ======================== Main ========================

def main():
    print("========== FT4232H SPI Data Capture Tool (CLI / 1 frame per file) ==========\n")

    devs = list_ftdi_devices()
    if not devs:
        print("No FTDI devices found. Check D2XX driver / cable connection.")
        sys.exit(1)

    print("Detected FTDI devices:")
    for i, desc, sn in devs:
        print(f"  [{i}] {desc} (SN:{sn})")
    print("")

    spi_index = input_with_default("SPI device index", 0, int)
    gpio_index = input_with_default("GPIO device index (same if shared)", 1, int)

    host_intr_mask_str = input_with_default("HOST_INTR mask hex (e.g. 0xA0)", "0xA0", str)
    try:
        host_intr_mask = int(host_intr_mask_str, 16)
    except Exception:
        host_intr_mask = 0xA0

    # clock
    clock_req_hz = input_with_default("SPI clock request (Hz) (e.g. 24000000/30000000/15000000)", 30000000, int)
    use_3phase_clk = input_yesno("Enable 3-phase clocking? (recommended ON for high speed stability)", True)

    # radar config (frame size)
    print("\n--- Frame size parameters (must match firmware) ---")
    adc_samples      = input_with_default("ADC samples", 256, int)
    chirps_per_burst = input_with_default("Chirps per burst", 64, int)
    bursts_per_frame = input_with_default("Bursts per frame", 1, int)
    rx_antennas      = input_with_default("RX antennas", 4, int)

    # capture options
    num_frames   = input_with_default("Number of frames (0=infinite)", 100, int)
    byteswap32   = input_yesno("Apply byteswap32? (recommended ON)", True)
    resync       = input_yesno("Resync on start? (recommended ON)", True)
    poll_us      = input_with_default("GPIO poll sleep (us)", 10, int)
    settle_us    = input_with_default("Settle time after LOW detect (us) (try 100~500 for 24MHz+)", 100, int)

    preview_every = input_with_default("Preview every N frames (0=off)", 1, int)
    log_every     = input_with_default("Log every N frames", 1, int)

    default_out_folder = os.path.join(os.getcwd(), f"capture_out_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
    out_folder = input_with_default("Output folder (1 file per frame)", default_out_folder, str)
    clean_old = input_yesno("Remove old .bin files in output folder?", True)

    capture_1frame_1file(
        spi_index=spi_index,
        gpio_index=gpio_index,
        clock_req_hz=clock_req_hz,
        host_intr_mask=host_intr_mask,
        num_frames=num_frames,
        out_folder=out_folder,
        byteswap32=byteswap32,
        resync_on_start=resync,
        poll_sleep_us=poll_us,
        settle_us=settle_us,
        use_3phase_clk=use_3phase_clk,
        adc_samples=adc_samples,
        chirps_per_burst=chirps_per_burst,
        bursts_per_frame=bursts_per_frame,
        rx_antennas=rx_antennas,
        preview_every=preview_every,
        log_every=log_every,
        clean_old_bins=clean_old,
    )

    input("\nPress Enter to exit...")

if __name__ == "__main__":
    main()
