  #!/usr/bin/env python3
  # -*- coding: utf-8 -*-
"""
can_isotp_send_profilecfg.py
- One-click friendly: all inputs have sensible defaults so you can run directly in PyCharm.
- Sends an Application PDU: [ MsgID | Payload ]
  * MsgID defaults to 0x24
  * Payload is MMWave_ProfileComCfg (packed LE, 11 bytes)

Requirements:
    pip install python-can can-isotp
    (and your PCAN runtime, as already used in can_isotp_sender.py)
"""
from __future__ import annotations
import argparse
import logging
import time

import can_isotp_sender
from mmwave_profile_cfg import MMWave_ProfileComCfg, hexdump, WIRE_SIZE

log = logging.getLogger("can_isotp_send_profilecfg")

def build_args():
    p = argparse.ArgumentParser(description="Send MMWave_ProfileComCfg as Application PDU over ISO-TP (PCAN)")
    # App-level
    p.add_argument("--msg-id", type=lambda x: int(x, 0), default=0x24, help="Application Msg ID (default 0x24)")

    # -------- Profile fields (ALL DEFAULTED) --------
    p.add_argument("--samp",  type=int,   default=1,    help="digOutputSampRate (uint8, default 1)")
    p.add_argument("--bits",  type=int,   default=2,    help="digOutputBitsSel  (uint8, default 2)")
    p.add_argument("--fir",   type=int,   default=3,    help="dfeFirSel         (uint8, default 3)")
    p.add_argument("--adc",   type=int,   default=4,    help="numOfAdcSamples   (uint16, default 4)")
    p.add_argument("--ramp",  type=float, default=5.0,  help="chirpRampEndTimeus (float, default 5.0)")
    p.add_argument("--rxhpf", type=int,   default=6,    help="chirpRxHpfSel     (uint8, default 6)")
    p.add_argument("--mimo",  type=int,   default=7,    help="chirpTxMimoPatSel (uint8, default 7)")

    # -------- Transport/Link params (defaults match your environment) --------
    p.add_argument("--channel", default="PCAN_USBBUS1")
    p.add_argument("--bitrate", type=int, default=500000)
    # FD on/off
    p.add_argument("--fd", dest="fd", action="store_true", default=True)
    p.add_argument("--no-fd", dest="fd", action="store_false")
    # 11-bit IDs by default
    p.add_argument("--txid", type=lambda x: int(x, 0), default=0xC0)
    p.add_argument("--rxid", type=lambda x: int(x, 0), default=0xC8)
    p.add_argument("--extended", action="store_true", help="Use 29-bit IDs")
    # ISO-TP params
    p.add_argument("--stmin", type=lambda x: int(x, 0), default=0)
    p.add_argument("--bs", type=int, default=8)
    p.add_argument("--tx_data_length", type=int, default=8)
    p.add_argument("--tx_data_min_length", type=int, default=None)
    p.add_argument("--override_receiver_stmin", type=float, default=None)
    p.add_argument("--rx_fc_timeout", type=int, default=1000)
    p.add_argument("--rx_cf_timeout", type=int, default=5000)
    p.add_argument("--tx_padding", type=lambda x: int(x, 0), default=0x00)
    p.add_argument("--can_fd_brs", action="store_true")
    # App behavior
    p.add_argument("--timeout", type=float, default=2.0)
    p.add_argument("--verbose", action="store_true")
    p.add_argument("--ifg-us", dest="ifg_us", type=int, default=3000)
    return p

def send_pdu(sender: "can_isotp_sender.CanIsoTpSender", msg_id: int, payload: bytes, timeout: float):
    if sender.stack is None:
        raise RuntimeError("sender.connect() must be called first")
    app = bytes([msg_id & 0xFF]) + (payload or b"")
    # Non-blocking send with a small wait loop (matches sender.send_bytes behaviour)
    sender.stack.send(app)
    log.debug("TX[APP %dB]: %s", len(app), hexdump(app))
    t0 = time.monotonic()
    while sender.stack.transmitting() and (time.monotonic() - t0) < timeout:
        time.sleep(0.002)

def main() -> int:
    args = build_args().parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    # Build payload from ARGS (defaults work out-of-the-box)
    cfg = MMWave_ProfileComCfg(
        digOutputSampRate=args.samp,
        digOutputBitsSel=args.bits,
        dfeFirSel=args.fir,
        numOfAdcSamples=args.adc,
        chirpRampEndTimeus=args.ramp,
        chirpRxHpfSel=args.rxhpf,
        chirpTxMimoPatSel=args.mimo,
    )
    payload = cfg.to_bytes()
    if len(payload) != WIRE_SIZE:
        log.error("Unexpected payload size: %d (expected %d)", len(payload), WIRE_SIZE)
        return 2

    log.info("MsgID=0x%02X Payload(%dB)=%s", args.msg_id, len(payload), hexdump(payload))

    # Create and configure sender
    sender = can_isotp_sender.CanIsoTpSender(
        channel=args.channel,
        bitrate=args.bitrate,
        fd=args.fd,
        txid=args.txid, rxid=args.rxid, extended=args.extended,
        stmin=args.stmin, blocksize=args.bs,
        tx_data_length=args.tx_data_length,
        tx_data_min_length=args.tx_data_min_length,
        override_receiver_stmin=args.override_receiver_stmin,
        rx_flowcontrol_timeout=args.rx_fc_timeout,
        rx_consecutive_frame_timeout=args.rx_cf_timeout,
        tx_padding=args.tx_padding,
        bitrate_switch=args.can_fd_brs,
        timeout_s=args.timeout,
        ifg_us=args.ifg_us,
    )

    try:
        sender.connect(ifg_us=args.ifg_us)
        log.info("ISO-TP ready: txid=0x%X rxid=0x%X (%s-bit, %s)",
                 args.txid, args.rxid, "29" if args.extended else "11", "FD" if args.fd else "Classic")

        # Send PDU [MsgID | Payload]
        send_pdu(sender, args.msg_id, payload, args.timeout)

        # Optional: wait for a response framed the same way
        rx = sender.read_response(timeout_s=args.timeout)
        if rx is None:
            print("(no response)")
        else:
            msg_id, body = rx
            try:
                txt = body.decode("utf-8").rstrip("\r\n")
                print(f"RX[MsgID={msg_id} {len(body)}B] ASCII: {txt}")
            except UnicodeDecodeError:
                from mmwave_profile_cfg import hexdump as _hd
                print(f"RX[MsgID={msg_id} {len(body)}B] HEX  : {_hd(body)}")

    finally:
        sender.disconnect()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
