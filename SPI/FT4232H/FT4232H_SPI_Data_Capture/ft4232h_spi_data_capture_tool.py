#!/usr/bin/env python3
"""
spi_data_capture_tool.py (Improved for AWRL6844 + FTDI(MPSSE) RX-only capture)

- Works with firmware pattern:
  - adcDataPerFrame = 131072 bytes (default)
  - transferred in chunks (MAXSPISIZEFTDI=65536) => 2 chunks per frame
  - Before each chunk: HOST_INTR -> LOW
  - After chunk: HOST_INTR -> HIGH
  - Host should wait LOW, then clock out data immediately.

Key improvements vs original:
- Fast path: read bytes and write to .bin directly (no list conversion, no parser heavy work)
- AOP GPIO is read from GPIO device consistently
- Wait only HOST_INTR LOW (no HIGH wait -> avoids timeout you saw)
- Optional byteswap32 to match your expected preview
- FrameNo printed as HEX-without-0x (so 0x311 prints "311", not 785)

===== Capture Start =====
Device Type     : AOP
SPI index       : 0
GPIO index      : 1 (separate)
SPI Clock       : 15.0 MHz
Frame Size      : 131,072 bytes
Chunks/Frame    : 2
HOST_INTR mask  : 0xA0  (ready when GPIO&mask==0)
Byteswap32      : ON
Output          : adc_data_20260205_145325.bin
"""

import os
import sys
import time
import struct
from datetime import datetime

try:
    import ftd2xx as ftd
except ImportError:
    print("ERROR: ftd2xx library not found. Install with: pip install ftd2xx")
    sys.exit(1)

######################## Configuration bytestrings ######################################################
FTDI_CFG_60MHZ_SYSCLK = b'\x8A'         # AN108 6.1:   Disable Clk Divide by 5, resulting in 60MHz system clock
FTDI_CFG_NO_ADAPT_CLK = b'\x97'         # AN108 6.10:  Turn Off Adaptive clocking
FTDI_CFG_NO_3PHAS_CLK = b'\x8D'         # AN108 6.4:   Disable 3 Phase Data Clocking
FTDI_CFG_SET_CLK_DIV  = b'\x86'         # AN108 3.8.2: Set clk divisor, [0x86,0xValueL,0xValueH]
FTDI_CFG_NO_LOOPBACK  = b'\x85'         # AN108 3.7.2: Disconnect TDI to TDO for Loopback
FTDI_CFG_SPI_4PIN_CFG = b'\x80\x08\x0B' # AN108 3.6.1: Set Data bits LowByte [0x80,0xValue,0xDirection] (value and direction are bitmasks for FTDI pins)
FTDI_CFG_SPI_WITH_GPIO = b'\x80\x08\x09'# AN108 3.6.1: Set Data bits LowByte [0x80,0xValue,0xDirection] (value and direction are bitmasks for FTDI pins)

######################### Command bytestrings ############################################################
FTDI_CMD_CS_LOW       = b'\x80\x00\x0B' # AN108 3.6.1: Set Data bits LowByte [0x80,0xValue,0xDirection] (value and direction are bitmasks for FTDI pins)
FTDI_CMD_CS_HIGH      = b'\x80\x08\x0B' # AN108 3.6.1: Set Data bits LowByte [0x80,0xValue,0xDirection] (value and direction are bitmasks for FTDI pins)
FTDI_CMD_WRITE_BYTES  = b'\x11'         # AN108 3.3.2: Clock Data Bytes Out on -ve clock edge MSB first (no read) [0x11,LengthL,LengthH,byte0,...,byteN]
FTDI_CMD_READ_BYTES   = b'\x20'         # AN108 3.3.5: Clock Data Bytes In on +ve clock edge MSB first (no write) [0x20,LengthL,LengthH]
FTDI_CMD_RW_BYTES     = b'\x31'         # AN108 3.3.9: Clock Data Bytes In on +ve and Out on -ve MSB first [0x20,LengthL,LengthH,byte0,...,byteN]
FTDI_CMD_READ_BITS    = b'\x81'         # AN108 3.6.3: Read Data bits LowByte, read the current state of the first 8 pins and send back 1 byte

FTDI_MAX_CHUNK = 65536  # TI uses MAXSPISIZEFTDI=65536

# ======================== Helpers ========================

def list_ftdi_devices():
    """Return list of (index, desc, serial)."""
    out = []
    try:
        devs = ftd.listDevices()
        if not devs:
            return out
        for i in range(len(devs)):
            try:
                h = ftd.open(i)
                info = h.getDeviceInfo()
                desc = info.get("description", b"").decode(errors="ignore")
                serial = info.get("serial", b"").decode(errors="ignore")
                out.append((i, desc, serial))
                h.close()
            except Exception:
                pass
    except Exception:
        pass
    return out

def set_clk(handle, hz: int):
    """Set SPI clock from 60MHz system clock."""
    if hz > 30_000_000:
        raise ValueError("Max SCK rate is 30MHz")
    div = int((60_000_000 / (hz * 2)) - 1)
    cmd = bytes((0x86, div & 0xFF, (div >> 8) & 0xFF))
    handle.write(cmd)

def set_device(handle, clk_speed: int = 15_000_000, latency_timer: int = 1):
    """Initialize FTDI device for MPSSE SPI read."""
    handle.resetDevice()

    # Purge RX buffer
    rx_bytes, tx_bytes, event_status = handle.getStatus()
    if rx_bytes > 0:
        handle.read(rx_bytes)

    handle.setUSBParameters(65535, 65535)
    handle.setChars(False, 0, False, 0)
    handle.setTimeouts(200000, 200000)
    handle.setLatencyTimer(latency_timer)

    handle.setBitMode(0, 0)   # Reset MPSSE
    handle.setBitMode(0, 2)   # Enable MPSSE
    time.sleep(0.050)

    handle.write(FTDI_CFG_60MHZ_SYSCLK)
    handle.write(FTDI_CFG_NO_ADAPT_CLK)
    handle.write(FTDI_CFG_NO_3PHAS_CLK)
    handle.write(FTDI_CFG_SPI_4PIN_CFG)
    set_clk(handle, clk_speed)
    time.sleep(0.020)
    handle.write(FTDI_CFG_NO_LOOPBACK)
    time.sleep(0.030)

def read_gpio(handle) -> int:
    """Read low byte GPIO state (8-bit)."""
    handle.write(FTDI_CMD_READ_BITS)
    res = handle.read(1)
    return int.from_bytes(res, "big")

def spi_read_bin(handle, length: int) -> bytes:
    """Read 'length' bytes using MPSSE read command with CS low/high."""
    if length < 1 or length > 0x10000:
        raise ValueError("Length must be 1..65536")
    ln = (length - 1).to_bytes(2, "little")
    cmd = FTDI_CMD_CS_LOW + FTDI_CMD_READ_BYTES + ln + FTDI_CMD_CS_HIGH
    handle.write(cmd)
    return handle.read(length)

def swap_bytes_32bit(data: bytes) -> bytes:
    """Reverse bytes within each 32-bit word."""
    # Expect multiples of 4 for your case (64KB chunks are multiple of 4)
    if len(data) % 4 != 0:
        # pad (shouldn't happen in your 131072B case)
        pad = 4 - (len(data) % 4)
        data = data + b"\x00" * pad
    b = bytearray(data)
    for i in range(0, len(b), 4):
        b[i], b[i+1], b[i+2], b[i+3] = b[i+3], b[i+2], b[i+1], b[i]
    return bytes(b)

def wait_intr_low(gpio_handle, mask: int, timeout_s: float = 5.0):
    """
    Wait until (GPIO & mask) == 0 (active-low ready).
    This matches TI reference behavior and your firmware (LOW before transfer).
    """
    t0 = time.perf_counter()
    last = 0
    spin = 0
    while True:
        last = read_gpio(gpio_handle)
        if (last & mask) == 0:
            return last
        if (time.perf_counter() - t0) > timeout_s:
            raise TimeoutError(f"HOST_INTR LOW timeout (GPIO=0x{last:02X}, mask=0x{mask:02X})")
        # Fast spin first, then yield a bit (Windows sleep resolution is coarse)
        spin += 1
        if spin > 2000:
            time.sleep(0.00001)

def input_with_default(prompt: str, default, cast=None):
    s = input(f"{prompt} [{default}]: ").strip()
    if s == "":
        return default
    if cast:
        return cast(s)
    return type(default)(s)

def input_yesno(prompt: str, default: bool) -> bool:
    d = "y" if default else "n"
    s = input(f"{prompt} [y/n, default={d}]: ").strip().lower()
    if s == "":
        return default
    return s.startswith("y")

# ======================== Capture Core ========================

def capture_frames(
    device_type: str,
    spi_index: int,
    gpio_index: int,
    clock_hz: int,
    frames: int,
    frame_period_ms: int,
    frame_size: int,
    host_intr_mask: int,
    out_path: str,
    byteswap32: bool,
    preview_every: int,
    log_every: int,
):
    dev_spi = None
    dev_gpio = None
    fout = None

    # For your firmware: 131072 bytes => 2 x 65536
    if frame_size <= 0:
        raise ValueError("frame_size must be > 0")

    try:
        dev_spi = ftd.open(spi_index)
        set_device(dev_spi, clock_hz, latency_timer=1)

        if device_type.upper() == "AOP":
            dev_gpio = ftd.open(gpio_index)
            # GPIO read does not require high clock; keep it low to reduce any overhead
            set_device(dev_gpio, 1_000_000, latency_timer=1)
        else:
            dev_gpio = dev_spi

        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
        fout = open(out_path, "wb")

        print("\n===== Capture Start =====")
        print(f"Device Type     : {device_type}")
        print(f"SPI index       : {spi_index}")
        print(f"GPIO index      : {gpio_index if dev_gpio != dev_spi else spi_index} ({'separate' if dev_gpio != dev_spi else 'same'})")
        print(f"SPI Clock       : {clock_hz/1e6:.1f} MHz")
        print(f"Frame Size      : {frame_size:,} bytes")
        print(f"Chunks/Frame    : {((frame_size + FTDI_MAX_CHUNK - 1)//FTDI_MAX_CHUNK)}")
        print(f"HOST_INTR mask  : 0x{host_intr_mask:02X}  (ready when GPIO&mask==0)")
        print(f"Byteswap32      : {'ON' if byteswap32 else 'OFF'}")
        print(f"Output          : {out_path}")
        print("Waiting for sensor streaming...\n")

        total_expected_time = None
        if frames > 0 and frame_period_ms > 0:
            total_expected_time = frames * (frame_period_ms / 1000.0)

        total_bytes = 0
        start_time = time.perf_counter()

        frame_idx = 0
        while True:
            if frames > 0 and frame_idx >= frames:
                break

            # Stream-read one frame (write chunks directly to file)
            bytes_left = frame_size
            preview_buf = bytearray()
            frame_id_u32 = None

            while bytes_left > 0:
                chunk_size = FTDI_MAX_CHUNK if bytes_left > FTDI_MAX_CHUNK else bytes_left

                # Wait HOST_INTR LOW (ready) for each chunk
                wait_intr_low(dev_gpio, host_intr_mask, timeout_s=5.0)

                chunk = spi_read_bin(dev_spi, chunk_size)

                # Apply byteswap if needed (to match your expected byte order)
                if byteswap32:
                    chunk = swap_bytes_32bit(chunk)

                # Save to file immediately (fast)
                fout.write(chunk)

                # Capture preview bytes (first 64B)
                if len(preview_buf) < 64:
                    need = 64 - len(preview_buf)
                    preview_buf.extend(chunk[:need])

                bytes_left -= len(chunk)

            fout.flush()

            frame_idx += 1
            total_bytes += frame_size

            # Extract frame number from first 4 bytes (firmware writes BE at byte[0..3])
            if len(preview_buf) >= 4:
                frame_id_u32 = struct.unpack(">I", preview_buf[:4])[0]
                # You want "311" (hex digits) instead of 785(dec)
                frame_no_hex = format(frame_id_u32, "X")  # no 0x
            else:
                frame_no_hex = "?"

            now = time.perf_counter()
            elapsed = now - start_time
            mbps = (total_bytes / (1024 * 1024)) / elapsed if elapsed > 0 else 0.0

            # Print preview sometimes
            if preview_every > 0 and (frame_idx % preview_every) == 0:
                hex_str = " ".join(f"{b:02X}" for b in preview_buf[:64])
                print(f"[PREVIEW] {hex_str}")

            # Print log
            if log_every > 0 and (frame_idx % log_every) == 0:
                if total_expected_time is not None:
                    remaining = total_expected_time - (frame_idx * (frame_period_ms / 1000.0))
                    if remaining < 0:
                        remaining = 0
                    print(f"Frame {frame_idx}: {frame_size} bytes | Total {total_bytes/1024:.1f} KB | {mbps:.2f} MB/s | FrameNo={frame_no_hex} | ETA {int(remaining)}s")
                else:
                    print(f"Frame {frame_idx}: {frame_size} bytes | Total {total_bytes/1024:.1f} KB | {mbps:.2f} MB/s | FrameNo={frame_no_hex}")

        print("\n===== Capture Done =====")
        print(f"Frames captured : {frame_idx}")
        print(f"Total bytes     : {total_bytes:,}")
        print(f"Avg speed       : {(total_bytes/(1024*1024))/max((time.perf_counter()-start_time),1e-9):.2f} MB/s")

    finally:
        try:
            if fout:
                fout.close()
        except Exception:
            pass
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

# ======================== Main (Interactive) ========================

def main():
    print("========== SPI Data Capture Tool (Improved / FAST BIN) ==========\n")

    # Show devices
    devs = list_ftdi_devices()
    if not devs:
        print("No FTDI devices found. Check D2XX driver / cable connection.")
        sys.exit(1)

    print("Detected FTDI devices:")
    for i, desc, sn in devs:
        print(f"  [{i}] {desc} (SN:{sn})")
    print("")

    # Defaults for your setup
    device_sel = input_with_default("Device type: 1=AOP, 2=FCCSP", 1, int)
    device_type = "AOP" if device_sel == 1 else "FCCSP"

    spi_index = input_with_default("SPI device index", 0, int)
    if device_type == "AOP":
        gpio_index = input_with_default("GPIO device index (AOP uses separate)", 1, int)
        host_intr_mask = 0xA0
    else:
        gpio_index = spi_index
        host_intr_mask = 0x10

    # Allow override mask if needed
    mask_override = input_with_default(f"HOST_INTR mask hex (0x..), default 0x{host_intr_mask:02X}", f"0x{host_intr_mask:02X}", str)
    try:
        host_intr_mask = int(mask_override, 16)
    except Exception:
        pass

    clock_hz = input_with_default("SPI clock (Hz)", 15000000, int)

    frame_size = input_with_default("Frame size (bytes) (firmware sends 131072)", 131072, int)

    frames = input_with_default("Number of frames (0=infinite)", 100, int)
    frame_period_ms = input_with_default("Frame periodicity (ms) (ETA only)", 200, int)

    byteswap32 = input_yesno("Apply 32-bit byteswap? (recommended ON for your preview)", True)
    preview_every = input_with_default("Preview every N frames (0=off)", 1, int)
    log_every = input_with_default("Log every N frames (0=off)", 1, int)

    default_out = f"adc_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.bin"
    out_path = input_with_default("Output .bin file path", default_out, str)

    capture_frames(
        device_type=device_type,
        spi_index=spi_index,
        gpio_index=gpio_index,
        clock_hz=clock_hz,
        frames=frames,
        frame_period_ms=frame_period_ms,
        frame_size=frame_size,
        host_intr_mask=host_intr_mask,
        out_path=out_path,
        byteswap32=byteswap32,
        preview_every=preview_every,
        log_every=log_every,
    )

    input("\nPress Enter to exit...")

if __name__ == "__main__":
    main()
