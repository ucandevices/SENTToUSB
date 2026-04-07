"""
verify_sent.py  --  Capture SENT signal via Logic2 automation and verify frame,
                    then check COM8 (SENT RX device) for decoded output.

Signal: Channel 2 (digital), inverted (CSV 0=PA4 HIGH, CSV 1=PA4 LOW)
Firmware config: tick=6us, 6 data nibbles, CRC DATA_ONLY seed=0x03
Expected frame:  status=0, data=[0,1,2,3,4,5], CRC=1
"""

import csv
import os
import sys
import time
import tempfile
import serial
import serial.tools.list_ports

# ── Logic2 automation ─────────────────────────────────────────────────────────
from saleae import automation

LOGIC2_PORT         = 10430
CAPTURE_DURATION_S  = 3.0       # capture 3 seconds to get at least 3 frames
DIGITAL_SAMPLE_RATE = 10_000_000  # 10 MHz -> 0.1 us resolution, fine for 6 us tick
CHANNEL             = 2          # Channel 2

# ── SENT decoder config ───────────────────────────────────────────────────────
NOM_TICK_US      = 6.0
NUM_NIBBLES      = 6
CRC_SEED         = 3
CRC_MODE         = "DATA_ONLY"
SYNC_TICKS       = 56
SYNC_MIN_TICKS   = int(SYNC_TICKS * 0.80)   # allow +-20%
SYNC_MAX_TICKS   = int(SYNC_TICKS * 1.20)
NIBBLE_MIN_TICKS = 12
NIBBLE_MAX_TICKS = 27

_CRC_LUT = [0, 13, 7, 10, 14, 3, 9, 4, 1, 12, 6, 11, 15, 2, 8, 5]

def crc4(nibbles, seed=3):
    crc = seed
    for n in nibbles:
        crc = _CRC_LUT[crc ^ (n & 0xF)]
    return crc


def decode_sent_csv(csv_path, inverted=True):
    """
    Decode SENT frames from a Logic2 edge CSV.
    inverted=True:  CSV 0->1 is PA4 falling edge (SENT interval start).
    inverted=False: CSV 1->0 is PA4 falling edge.
    """
    times = []
    prev_val = None
    with open(csv_path, newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) < 2:
                continue
            try:
                t   = float(row[0])
                val = int(row[1])
            except ValueError:
                continue
            if prev_val is None:
                prev_val = val
                continue
            # Rising edge in CSV = PA4 falling edge when inverted
            if inverted:
                is_falling_pa4 = (prev_val == 0 and val == 1)
            else:
                is_falling_pa4 = (prev_val == 1 and val == 0)

            if is_falling_pa4:
                times.append(t * 1e6)   # us
            prev_val = val

    print(f"  PA4 falling edges found: {len(times)}")
    if len(times) < 2:
        return []

    intervals_us = [times[i+1] - times[i] for i in range(len(times)-1)]
    print(f"  Smallest interval: {min(intervals_us):.1f} us")

    frames = []
    state  = "WAIT_SYNC"
    tick   = NOM_TICK_US
    status = None
    data   = []
    sync_iv = None

    for iv in intervals_us:
        ticks_f = iv / tick
        ticks   = round(ticks_f)

        if state == "WAIT_SYNC":
            if SYNC_MIN_TICKS <= ticks <= SYNC_MAX_TICKS:
                tick    = iv / SYNC_TICKS
                sync_iv = iv
                status  = None
                data    = []
                state   = "STATUS"
            continue

        if state == "STATUS":
            if SYNC_MIN_TICKS <= ticks <= SYNC_MAX_TICKS:
                tick    = iv / SYNC_TICKS
                sync_iv = iv
                data    = []
                status  = None
                continue
            if ticks > SYNC_MAX_TICKS:
                state = "WAIT_SYNC"
                tick  = NOM_TICK_US
                continue
            if not (NIBBLE_MIN_TICKS <= ticks <= NIBBLE_MAX_TICKS):
                state = "WAIT_SYNC"
                tick  = NOM_TICK_US
                continue
            status = ticks - 12
            state  = "DATA"
            continue

        if state == "DATA":
            if SYNC_MIN_TICKS <= ticks <= SYNC_MAX_TICKS:
                tick    = iv / SYNC_TICKS
                sync_iv = iv
                data    = []
                status  = None
                state   = "STATUS"
                continue
            if not (NIBBLE_MIN_TICKS <= ticks <= NIBBLE_MAX_TICKS):
                state = "WAIT_SYNC"
                tick  = NOM_TICK_US
                continue
            data.append(ticks - 12)
            if len(data) >= NUM_NIBBLES:
                state = "CRC"
            continue

        if state == "CRC":
            if not (NIBBLE_MIN_TICKS <= ticks <= NIBBLE_MAX_TICKS):
                state = "WAIT_SYNC"
                tick  = NOM_TICK_US
                continue
            crc_recv = ticks - 12
            crc_calc = crc4(data, CRC_SEED)
            frames.append({
                "sync_us":  sync_iv,
                "tick_us":  tick,
                "status":   status,
                "data":     list(data),
                "crc_recv": crc_recv,
                "crc_calc": crc_calc,
                "crc_ok":   crc_recv == crc_calc,
            })
            state  = "STATUS"
            status = None
            data   = []
            continue

    return frames


def capture_and_decode():
    print("=== Step 1: Logic2 capture ===")
    print(f"Connecting to Logic2 on port {LOGIC2_PORT}...")
    try:
        mgr = automation.Manager.connect(port=LOGIC2_PORT)
    except Exception as e:
        print(f"FAILED to connect to Logic2: {e}")
        print("Make sure Logic2 is running with automation API enabled (port 10430).")
        return None

    devices = mgr.get_devices(include_simulation_devices=False)
    if not devices:
        # fall back to simulation
        devices = mgr.get_devices(include_simulation_devices=True)
        if not devices:
            print("No Logic2 devices found (real or simulation).")
            mgr.close()
            return None
        print(f"No real device, using simulation: {devices[0].device_id}")
    else:
        print(f"Device: {devices[0].device_id}  type={devices[0].device_type}")

    device_id = devices[0].device_id

    dev_cfg = automation.LogicDeviceConfiguration(
        enabled_digital_channels=[CHANNEL],
        digital_sample_rate=DIGITAL_SAMPLE_RATE,
    )
    cap_cfg = automation.CaptureConfiguration(
        capture_mode=automation.TimedCaptureMode(duration_seconds=CAPTURE_DURATION_S)
    )

    print(f"Starting {CAPTURE_DURATION_S}s capture at {DIGITAL_SAMPLE_RATE//1_000_000} MHz...")
    try:
        cap = mgr.start_capture(
            device_id=device_id,
            device_configuration=dev_cfg,
            capture_configuration=cap_cfg,
        )
        cap.wait()
        print("Capture complete.")
    except Exception as e:
        print(f"Capture failed: {e}")
        mgr.close()
        return None

    # Export to temp dir
    export_dir = os.path.join(os.path.dirname(__file__), "logic_export")
    os.makedirs(export_dir, exist_ok=True)
    csv_path = os.path.join(export_dir, "digital.csv")

    print(f"Exporting CSV to {export_dir}...")
    try:
        cap.export_raw_data_csv(
            directory=export_dir,
            digital_channels=[CHANNEL],
        )
    except Exception as e:
        print(f"Export failed: {e}")
        cap.close()
        mgr.close()
        return None

    cap.close()
    mgr.close()

    if not os.path.exists(csv_path):
        print(f"CSV not found at {csv_path}")
        return None

    print(f"CSV exported: {os.path.getsize(csv_path)} bytes")
    return csv_path


def verify_frames(csv_path):
    print("\n=== Step 2: SENT frame decode ===")
    print(f"File: {csv_path}")
    print(f"Config: tick={NOM_TICK_US}us, {NUM_NIBBLES} nibbles, CRC {CRC_MODE} seed=0x{CRC_SEED:02X}")
    print(f"Expected: status=0, data=[0,1,2,3,4,5], CRC=1")
    print()

    frames = decode_sent_csv(csv_path, inverted=True)

    if not frames:
        print("No SENT frames decoded!")
        print("Trying non-inverted interpretation...")
        frames = decode_sent_csv(csv_path, inverted=False)

    if not frames:
        print("Still no frames. Check wiring and signal level.")
        return False

    print()
    print(f"{'#':>3}  {'sync_us':>8}  {'tick_us':>8}  {'ST':>3}  {'Data':>12}  {'CRC':>5}  Result")
    print("-" * 60)
    ok = 0
    for i, f in enumerate(frames):
        data_str = "".join(f"{d:X}" for d in f["data"])
        result = "OK" if f["crc_ok"] else f"FAIL(calc={f['crc_calc']:X})"
        print(f"{i:>3}  {f['sync_us']:>8.1f}  {f['tick_us']:>8.4f}  "
              f"{f['status']:>3}  {data_str:>12}  {f['crc_recv']:>5}  {result}")
        if f["crc_ok"]:
            ok += 1

    print()
    print(f"Total: {len(frames)}  OK: {ok}  CRC errors: {len(frames)-ok}")

    if frames and frames[0]["crc_ok"]:
        f = frames[0]
        if f["status"] == 0 and f["data"] == [0,1,2,3,4,5] and f["crc_recv"] == 1:
            print("PASS: Frame matches expected pattern (status=0, data=012345, CRC=1)")
        else:
            print(f"INFO: Frame decoded OK but different data: "
                  f"status={f['status']}, data={f['data']}, CRC={f['crc_recv']}")
    return ok > 0


def check_com8_rx(port="COM8", baud=115200, timeout_s=6.0):
    print(f"\n=== Step 3: Check {port} SENT RX output ===")
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"Cannot open {port}: {e}")
        return

    # Query firmware version
    ser.write(b'V\r')
    time.sleep(0.2)
    ver = ser.read(50).decode("ascii", errors="replace").strip()
    print(f"  Firmware version: {ver!r}")

    # Open SLCAN channel
    ser.write(b'O\r')
    time.sleep(0.2)
    ack = ser.read(10).decode("ascii", errors="replace").strip()
    print(f"  Channel open ack: {ack!r}")

    print(f"  Listening for {timeout_s}s...")
    deadline = time.time() + timeout_s
    lines = []
    while time.time() < deadline:
        raw = ser.readline()
        if raw:
            line = raw.decode("ascii", errors="replace").strip()
            if line:
                lines.append(line)
                print(f"  RX: {line}")
    ser.close()

    if not lines:
        print(f"  No data received from {port} in {timeout_s}s.")
        print("  >> Wire TX device PA4 to this device's PA2 (pin 8)")
    else:
        tframes = [l for l in lines if l.startswith('t') or l.startswith('T')]
        print(f"\n  Total lines: {len(lines)}  SLCAN t-frames: {len(tframes)}")
        if tframes:
            # Decode first t-frame: t<id3><dlc><data...>
            f0 = tframes[0]
            print(f"  First frame: {f0}")
            try:
                dlc  = int(f0[4])
                data = bytes.fromhex(f0[5:5+dlc*2])
                print(f"  Decoded bytes: {list(data)}")
            except Exception:
                pass


def main():
    # Allow passing an existing CSV to skip capture
    if len(sys.argv) > 1 and os.path.exists(sys.argv[1]):
        csv_path = sys.argv[1]
        print(f"Using existing CSV: {csv_path}")
    else:
        csv_path = capture_and_decode()
        if csv_path is None:
            print("Capture failed, aborting.")
            sys.exit(1)

    ok = verify_frames(csv_path)
    check_com8_rx(port="COM8")


if __name__ == "__main__":
    main()
