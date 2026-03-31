#!/usr/bin/env python3
"""
sent_test.py — SLCAN test tool for SENTToUSB firmware.

Usage:
  python sent_test.py COM8
  python sent_test.py COM8 --nibbles 6 --crc-mode 0 --seed 0x03 --can-id 0x510

Config frame (CAN ID 0x001, DLC 7) — new layout:
  B0 : data nibbles    (4, 6, or 8)
  B1 : CRC mode        (0=DATA_ONLY, 1=STATUS_AND_DATA)
  B2 : CRC seed        (0x03=SAE APR2016, 0x05=legacy/GM)
  B3 : min tick        (units of 0.5 µs; e.g. 5 = 2.5 µs)
  B4 : max tick        (units of 1 µs;   e.g. 5 = 5.0 µs)
  B5 : output CAN ID high byte  (big-endian)
  B6 : output CAN ID low byte
"""
import argparse
import re
import sys
import time
import threading
import serial


SLCAN_CONFIG_ID  = 0x001
SLCAN_CONTROL_ID = 0x600


def build_slcan_frame(can_id: int, data: bytes) -> str:
    return f"t{can_id:03X}{len(data):X}" + data.hex().upper()


def send_line(port: serial.Serial, line: str) -> str:
    port.write((line + "\r").encode())
    port.flush()
    resp = b""
    deadline = time.time() + 1.5
    while time.time() < deadline:
        ch = port.read(1)
        if not ch:
            continue
        resp += ch
        if ch in (b"\r", b"\a"):
            break
    return resp.decode(errors="replace").strip()


def decode_slcan_frame(line: str):
    """Return (can_id, data_bytes) or None."""
    m = re.match(r"t([0-9A-Fa-f]{3})([0-9A-Fa-f])([0-9A-Fa-f]*)", line)
    if not m:
        return None
    can_id = int(m.group(1), 16)
    dlc    = int(m.group(2), 16)
    raw    = m.group(3)
    if len(raw) != dlc * 2:
        return None
    return can_id, bytes.fromhex(raw)


def rx_thread(port: serial.Serial, target_id: int, stop_event: threading.Event,
              counters: dict):
    buf = ""
    while not stop_event.is_set():
        try:
            chunk = port.read(64)
        except Exception:
            break
        if not chunk:
            continue
        buf += chunk.decode(errors="replace")
        while True:
            idx = buf.find("\r")
            if idx == -1:
                break
            line = buf[:idx].strip()
            buf  = buf[idx + 1:]
            if not line:
                continue

            frame = decode_slcan_frame(line)
            if not frame:
                continue
            cid, data = frame

            if cid == target_id and len(data) >= 3:
                counters["rx"] += 1
                nibbles = []
                for b in data:
                    nibbles.append((b >> 4) & 0x0F)
                    nibbles.append(b & 0x0F)
                nib_str = " ".join(f"{n:X}" for n in nibbles)
                hex_str = " ".join(f"{b:02X}" for b in data)
                print(f"  [0x{cid:03X}] {hex_str}   nibbles: {nib_str}")

            elif cid == 0x601:
                # Learn result
                if len(data) >= 4:
                    tick_x10 = data[0] | (data[1] << 8)
                    print(f"  [LEARN RESULT] tick={tick_x10/10:.1f}µs  "
                          f"nibbles={data[2]}  crc_mode={data[3]}")


def main():
    parser = argparse.ArgumentParser(description="SENTToUSB SLCAN test tool")
    parser.add_argument("port",       help="Serial port (e.g. COM8)")
    parser.add_argument("--nibbles",  type=int, default=6, choices=[4, 6, 8])
    parser.add_argument("--crc-mode", type=int, default=0, choices=[0, 1],
                        help="0=DATA_ONLY  1=STATUS_AND_DATA")
    parser.add_argument("--seed",     type=lambda x: int(x, 0), default=0x03,
                        help="CRC init seed: 0x03=APR2016 (default), 0x05=legacy/GM")
    parser.add_argument("--tick-min", type=int, default=5,
                        help="Min tick, units 0.5µs (default 5 ->2.5µs)")
    parser.add_argument("--tick-max", type=int, default=5,
                        help="Max tick, units 1µs   (default 5 ->5.0µs)")
    parser.add_argument("--can-id",   type=lambda x: int(x, 0), default=0x510,
                        help="Output RX CAN ID (default: 0x510)")
    parser.add_argument("--learn",    action="store_true",
                        help="Run learn mode instead of fixed config")
    parser.add_argument("--timeout",  type=float, default=10.0)
    args = parser.parse_args()

    print(f"Opening {args.port} ...")
    try:
        port = serial.Serial(args.port, baudrate=115200, timeout=0.05)
    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)
    time.sleep(0.1)
    port.reset_input_buffer()

    # ── Query device info ─────────────────────────────────────────────────────
    for cmd in ("V", "v", "N"):
        r = send_line(port, cmd)
        print(f"  {cmd} ->{r!r}")

    # ── Open channel ──────────────────────────────────────────────────────────
    r = send_line(port, "O")
    print(f"  O ->{r!r}")
    if r not in ("\r", ""):
        print("  WARNING: unexpected ACK for O command")

    # ── Config or learn ───────────────────────────────────────────────────────
    if args.learn:
        r = send_line(port, build_slcan_frame(SLCAN_CONTROL_ID, bytes([0x04])))
        print(f"\nLearn mode started ->{r!r}")
        print(f"Listening for learn result + frames for {args.timeout:.0f}s ...\n")
        target_id = args.can_id
    else:
        can_id = max(1, min(args.can_id, 0x7FF))
        config_data = bytes([
            args.nibbles,
            args.crc_mode,
            args.seed & 0xFF,
            args.tick_min & 0xFF,
            args.tick_max & 0xFF,
            (can_id >> 8) & 0xFF,
            can_id & 0xFF,
        ])
        frame = build_slcan_frame(SLCAN_CONFIG_ID, config_data)
        r = send_line(port, frame)
        print(f"\nConfig  {frame}")
        print(f"  nibbles={args.nibbles}  crc_mode={args.crc_mode}  "
              f"seed=0x{args.seed:02X}  tick {args.tick_min*0.5:.1f}–{args.tick_max:.1f}µs  "
              f"CAN ID=0x{can_id:03X}  ->{r!r}")
        if "\a" in r or r == chr(7):
            print("  ERROR: device returned NACK — invalid config")
            port.close()
            sys.exit(1)
        target_id = can_id
        print(f"\nListening for {args.timeout:.0f}s on CAN ID 0x{target_id:03X} ...\n")

    # ── RX loop ───────────────────────────────────────────────────────────────
    counters = {"rx": 0}
    stop = threading.Event()
    t = threading.Thread(target=rx_thread,
                         args=(port, target_id, stop, counters), daemon=True)
    t.start()
    try:
        time.sleep(args.timeout)
    except KeyboardInterrupt:
        pass
    stop.set()
    t.join(timeout=1.0)

    send_line(port, "C")
    port.close()
    print(f"\nDone. Received {counters['rx']} frame(s).")
    if counters["rx"] == 0:
        print("  No frames received — check sensor connection and config.")


if __name__ == "__main__":
    main()
