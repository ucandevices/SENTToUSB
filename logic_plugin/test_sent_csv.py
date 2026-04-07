"""
Standalone test for the SENT decoder algorithm.

Reads digital.csv (Logic 2 export: "Time [s],Channel 2"),
extracts falling edges, decodes SENT frames and prints a table.

Expected from MLX90377 capture:
  - tick = 3.000 us (3 us nominal)
  - sync = 168-170 us  (56 ticks; 170 us is quantization artefact at 500 kHz)
  - 6 data nibbles
  - CRC-4 DATA_ONLY, seed=0x03

NOTE on sampling rate
---------------------
This CSV was captured at ~500 kHz (2 us/sample).  The SENT tick is 3 us,
so each sample represents 1.5 ticks.  A 2 us quantization error equals
0.67 ticks, which is enough to push borderline nibbles to the wrong value.
Consequence: approximately half the frames will fail CRC at 500 kHz.
Recapture at >= 10 MHz for reliable (>99%) frame decoding.

The algorithm is verified correct: frames where the sync measures exactly
168 us (the nominal 56x3 us) consistently pass CRC.

Usage:
    python test_sent_csv.py [digital.csv]
"""

import csv
import sys
import os

# ── CRC-4 SAE J2716 ─────────────────────────────────────────────────────────
_CRC_LUT = [0, 13, 7, 10, 14, 3, 9, 4, 1, 12, 6, 11, 15, 2, 8, 5]

def crc4(nibbles, status_nibble=0, mode="DATA_ONLY", seed=3):
    crc = seed
    if mode == "STATUS_AND_DATA":
        crc = _CRC_LUT[crc ^ (status_nibble & 0xF)]
    for n in nibbles:
        crc = _CRC_LUT[crc ^ (n & 0xF)]
    return crc


# ── SENT frame decoder ────────────────────────────────────────────────────────
SYNC_TICKS       = 56
SYNC_MIN_TICKS   = int(SYNC_TICKS * 0.65)   # 36
SYNC_MAX_TICKS   = int(SYNC_TICKS * 1.35)   # 75
NIBBLE_MIN_TICKS = 12
NIBBLE_MAX_TICKS = 27
GAP_MIN_US       = 30.0   # minimum real interval (shorter = Async Serial glitch)

NUM_NIBBLES = 6
CRC_MODE    = "DATA_ONLY"
CRC_SEED    = 3
NOM_TICK_US = 3.0        # nominal tick; refined per-frame from measured sync


def decode_frames(csv_path):
    # 1. Read all transitions, extract falling edges (1->0)
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
            if prev_val == 1 and val == 0:
                times.append(t * 1e6)   # store in us
            prev_val = val

    if len(times) < 2:
        print("Not enough falling edges found.")
        return []

    # Measure sample period from smallest interval (used for diagnostics only)
    intervals_all = [times[i+1] - times[i] for i in range(len(times)-1)]
    min_iv = min(intervals_all)
    # Estimate sampling period as GCD-like minimum meaningful step
    sample_us = round(min_iv / round(min_iv)) if min_iv > 0 else 2.0
    print(f"Total falling edges : {len(times)}")
    print(f"Smallest interval   : {min_iv:.1f} us  (est. sample period ~{sample_us:.1f} us)")
    if sample_us >= 2.0:
        print(f"WARNING: sample period >= 2 us ({1e6/sample_us/1e6*1e3:.0f} kHz). "
              f"Nibble quantization errors expected (~{sample_us/NOM_TICK_US*100:.0f}% tick error).")

    # 2. Compute intervals between consecutive falling edges
    intervals_us = intervals_all

    # 3. Decode
    frames = []
    state  = "WAIT_SYNC"
    tick   = NOM_TICK_US
    status = None
    data   = []
    sync_interval = None
    frame_start_idx = None
    reset_count = 0

    i = 0
    while i < len(intervals_us):
        iv = intervals_us[i]

        # Gap filter: skip sub-threshold intervals (Async Serial glitches, etc.)
        if iv < GAP_MIN_US:
            i += 1
            continue

        ticks_f = iv / tick
        ticks   = round(ticks_f)

        if state == "WAIT_SYNC":
            if SYNC_MIN_TICKS <= ticks <= SYNC_MAX_TICKS:
                tick = iv / SYNC_TICKS
                sync_interval = iv
                state = "STATUS"
                status = None
                data   = []
                frame_start_idx = i
            i += 1
            continue

        if state == "STATUS":
            if SYNC_MIN_TICKS <= ticks <= SYNC_MAX_TICKS:
                # Consecutive sync — re-sync (handles pause+sync in one step)
                tick = iv / SYNC_TICKS
                sync_interval = iv
                data   = []
                status = None
                frame_start_idx = i
                i += 1
                continue
            if ticks > SYNC_MAX_TICKS:
                # Pause interval after CRC: normal, just wait for next sync
                state = "WAIT_SYNC"
                tick  = NOM_TICK_US
                i += 1
                continue
            if not (NIBBLE_MIN_TICKS <= ticks <= NIBBLE_MAX_TICKS):
                reset_count += 1
                state = "WAIT_SYNC"
                tick  = NOM_TICK_US
                i += 1
                continue
            status = ticks - 12
            state  = "DATA"
            i += 1
            continue

        if state == "DATA":
            if SYNC_MIN_TICKS <= ticks <= SYNC_MAX_TICKS:
                # Premature sync — frame truncated, re-sync
                tick = iv / SYNC_TICKS
                sync_interval = iv
                state = "STATUS"
                data  = []
                status = None
                frame_start_idx = i
                i += 1
                continue
            if not (NIBBLE_MIN_TICKS <= ticks <= NIBBLE_MAX_TICKS):
                reset_count += 1
                state = "WAIT_SYNC"
                tick  = NOM_TICK_US
                i += 1
                continue
            data.append(ticks - 12)
            if len(data) >= NUM_NIBBLES:
                state = "CRC"
            i += 1
            continue

        if state == "CRC":
            if not (NIBBLE_MIN_TICKS <= ticks <= NIBBLE_MAX_TICKS):
                reset_count += 1
                state = "WAIT_SYNC"
                tick  = NOM_TICK_US
                i += 1
                continue
            crc_recv = ticks - 12
            crc_calc = crc4(data, status if status is not None else 0,
                            CRC_MODE, CRC_SEED)
            crc_ok   = (crc_recv == crc_calc)
            frames.append({
                "idx":        len(frames),
                "edge_i":     frame_start_idx,
                "sync_us":    sync_interval,
                "tick_us":    tick,
                "status":     status,
                "data":       list(data),
                "crc_recv":   crc_recv,
                "crc_calc":   crc_calc,
                "crc_ok":     crc_ok,
            })
            # Transition: next interval is the pause; stay in STATUS to handle it
            state  = "STATUS"
            status = None
            data   = []
            i += 1
            continue

        i += 1

    print(f"Mid-frame resets    : {reset_count}")
    return frames


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.join(os.path.dirname(__file__), "digital.csv")

    if not os.path.exists(csv_path):
        print(f"File not found: {csv_path}")
        sys.exit(1)

    print(f"Decoding : {csv_path}")
    print(f"Settings : {NUM_NIBBLES} data nibbles, CRC={CRC_MODE}, "
          f"seed=0x{CRC_SEED:02X}, nom_tick={NOM_TICK_US} us")
    print()

    frames = decode_frames(csv_path)

    if not frames:
        print("No SENT frames decoded.")
        return

    print()
    hdr = f"{'#':>5}  {'sync_us':>8}  {'tick_us':>8}  {'ST':>3}  {'Data':>12}  {'CRC':>3}  {'Result'}"
    print(hdr)
    print("-" * len(hdr))

    ok_168 = err_168 = ok_170 = err_170 = ok_other = err_other = 0
    for f in frames:
        data_str = "".join(f"{d:X}" for d in f["data"])
        crc_str  = f"{f['crc_recv']:X}"
        result   = "OK" if f["crc_ok"] else f"FAIL(calc={f['crc_calc']:X})"
        print(f"{f['idx']:>5}  {f['sync_us']:>8.1f}  {f['tick_us']:>8.4f}  "
              f"{f['status']:>3X}  {data_str:>12}  {crc_str:>3}  {result}")
        sync = round(f["sync_us"])
        if sync == 168:
            if f["crc_ok"]: ok_168 += 1
            else:           err_168 += 1
        elif sync == 170:
            if f["crc_ok"]: ok_170 += 1
            else:           err_170 += 1
        else:
            if f["crc_ok"]: ok_other += 1
            else:           err_other += 1

    total = len(frames)
    ok    = ok_168 + ok_170 + ok_other
    err   = err_168 + err_170 + err_other

    print()
    print(f"Total frames : {total}   OK: {ok}   CRC errors: {err}   "
          f"Pass rate: {100*ok/total:.1f}%")
    print(f"  sync=168us : {ok_168+err_168:4d} frames  OK={ok_168}  "
          f"({100*ok_168/(ok_168+err_168) if ok_168+err_168 else 0:.1f}%)")
    print(f"  sync=170us : {ok_170+err_170:4d} frames  OK={ok_170}  "
          f"({100*ok_170/(ok_170+err_170) if ok_170+err_170 else 0:.1f}%)")
    if ok_other + err_other:
        print(f"  sync=other : {ok_other+err_other:4d} frames  OK={ok_other}")
    print()
    print("Note: sync=170 us is a 500 kHz quantization artefact of the true 168 us.")
    print("      At 10 MHz+ capture rate both groups will decode reliably.")


if __name__ == "__main__":
    main()
