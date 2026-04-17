#!/usr/bin/env python3
"""
tx_test.py - Diagnose COM8 TX and verify reception on COM9.

Steps:
  1. Query COM8 debug counters (before anything)
  2. Start TX mode on COM8
  3. Send 1 frame, wait 100ms, query counters → see if TIM14 ISR fired
  4. Try sending more frames to confirm queue drains
  5. Monitor COM9 for t510 frames

Usage:
  python tx_test.py             # run TX/RX test
  python tx_test.py --boot8     # trigger DFU bootloader on COM8
  python tx_test.py --boot9     # trigger DFU bootloader on COM9
"""
import serial, time, sys

COM8 = "COM8"   # TX device (STLINK attached)
COM9 = "COM9"   # RX device

def boot_device(port):
    """Send 'boot' command to trigger DFU bootloader."""
    print(f"Sending boot command to {port}...")
    s = serial.Serial(port, 115200, timeout=0.5)
    s.write(b"boot\r")
    time.sleep(0.2)
    resp = s.read(s.in_waiting or 64)
    print(f"  boot -> {resp!r}")
    s.close()

def open_port(port, timeout=0.1):
    return serial.Serial(port, 115200, timeout=timeout)

def send_cmd(ser, cmd, wait=0.05):
    if not cmd.endswith('\r'):
        cmd += '\r'
    ser.write(cmd.encode())
    time.sleep(wait)
    return ser.read(ser.in_waiting or 64)

def query_debug(ser):
    """Send debug query t7FF0.
    Returns dict with kick, isr, rx_edges (TIM2 captures), tim_sr (TIM14->SR)."""
    ser.write(b"t7FF0\r")
    time.sleep(0.05)
    resp = ser.read(128).decode(errors='replace')
    for line in resp.split('\r'):
        if line.startswith('t7FF8') and len(line) >= 21:
            kick      = int(line[5:9],   16)
            isr       = int(line[9:13],  16)
            rx_edges  = int(line[13:17], 16)  # TIM2 CH3 rising-edge captures
            tim_sr    = int(line[17:21], 16)  # TIM14->SR: UIF=bit0
            return dict(kick=kick, isr=isr, rx_edges=rx_edges, tim_sr=tim_sr)
    print(f"    [debug raw: {resp!r}]")
    return dict(kick=0, isr=0, rx_edges=0, tim_sr=0)

def main():
    if "--boot8" in sys.argv:
        boot_device(COM8)
        return
    if "--boot9" in sys.argv:
        boot_device(COM9)
        return

    print(f"Opening {COM8} (TX device) and {COM9} (RX device)...")
    try:
        s8 = open_port(COM8)
        s9 = open_port(COM9)
    except Exception as e:
        print(f"ERROR opening ports: {e}")
        sys.exit(1)

    s8.reset_input_buffer()
    s9.reset_input_buffer()

    def show_debug(d, label):
        uif = bool(d['tim_sr'] & 0x0001)
        print(f"    [{label}] kick={d['kick']}  isr={d['isr']}  "
              f"rx_edges={d['rx_edges']}  "
              f"TIM14_SR=0x{d['tim_sr']:04X}(UIF={'1' if uif else '0'})")

    # --- Step 1: Query debug counters before any TX ---
    print("\n[1] Debug counters BEFORE TX:")
    d0 = query_debug(s8)
    show_debug(d0, "before")

    DATA_NIBBLES = 4   # set to 4, 6, or 8 to match sensor

    # --- Step 2: Open SLCAN channel, configure nibbles, then switch to TX mode ---
    print("\n[2] Opening SLCAN channel (O) on COM8...")
    r = send_cmd(s8, "O", wait=0.05)
    print(f"    ACK: {r!r}")

    print(f"    Setting data_nibbles={DATA_NIBBLES} on COM8...")
    r = send_cmd(s8, f"t0011{DATA_NIBBLES:02X}", wait=0.05)
    print(f"    ACK: {r!r}")

    print("    Starting TX mode (t600102)...")
    r = send_cmd(s8, "t600102", wait=0.05)
    print(f"    ACK: {r!r}")

    # Open COM9 and start RX mode
    print(f"\n[3] Opening SLCAN channel on COM9 (RX device) and starting RX mode...")
    s9.reset_input_buffer()
    r9 = send_cmd(s9, "C", wait=0.05)    # close first in case already open
    r9 = send_cmd(s9, "O", wait=0.05)
    print(f"    O ACK: {r9!r}")
    print(f"    Setting data_nibbles={DATA_NIBBLES} on COM9...")
    r9 = send_cmd(s9, f"t0011{DATA_NIBBLES:02X}", wait=0.05)
    print(f"    config ACK: {r9!r}")
    r9 = send_cmd(s9, "t600101", wait=0.1)
    print(f"    start-RX ACK: {r9!r}")
    s9.reset_input_buffer()

    # Packed MSB-first: 4 nibbles [1,2,3,4]=0x00001234, 6 nibbles [1..6]=0x00123456
    FRAMES = {4: b"t52050000001234\r", 6: b"t52050000123456\r", 8: b"t52050012345678\r"}
    FRAME_US = {4: 460, 6: 520, 8: 580}
    FRAME = FRAMES[DATA_NIBBLES]
    SENT_FRAME_US = FRAME_US[DATA_NIBBLES]

    # --- Step 3: Send 1 frame, wait, check counters ---
    print(f"\n[4] Sending 1 TX frame: {FRAME!r}")
    s8.write(FRAME)
    time.sleep(0.02)
    ack = s8.read(10)
    print(f"    Frame 1 ACK: {ack!r}  (expect b'z\\r')")

    print("    Waiting 200ms for TIM14 to drain...")
    time.sleep(0.2)

    print("\n[5] Debug counters AFTER 200ms:")
    d1 = query_debug(s8)
    show_debug(d1, "after ")

    if d1['kick'] == 0:
        print("\n*** tim14_kick() was NEVER CALLED. Check dispatch_slcan_line. ***")
    elif d1['isr'] > 0:
        print(f"\n*** TIM14 TX ISR fired {d1['isr']} times — TX working! ***")
    else:
        print(f"\n*** TIM14 ISR not firing. SR=0x{d1['tim_sr']:04X} ***")

    # --- Step 4: Send 3 more frames rapidly, watch for NACKs ---
    print("\n[6] Sending frames 2-5 rapidly (10ms apart)...")
    for i in range(2, 6):
        s8.write(FRAME)
        time.sleep(0.01)
        a = s8.read(10)
        status = "ACK" if b'z' in a else "NACK"
        print(f"    Frame {i}: {a!r}  [{status}]")
        time.sleep(0.2)   # Wait after each so TIM14 can drain

    # --- Step 5: Send 10 frames with ACK-based pacing, monitor COM9 ---
    # Root cause of 3/10: Python bundles all writes into one USB packet, so the
    # STM32 processes all 10 submits back-to-back before TIM14 drains the first
    # frame.  Fix: wait for 'z\r' ACK before sending the next frame; if '\a'
    # (slot busy), back off 2ms and retry.
    N_FRAMES = 10
    print(f"\n[7] Sending {N_FRAMES} frames with ACK-paced flow control, watching COM9...")
    s9.reset_input_buffer()
    s8.reset_input_buffer()
    s8.timeout = 0.05   # 50ms read timeout per byte

    accepted = 0
    rejected = 0
    for i in range(N_FRAMES):
        for attempt in range(20):           # up to 20 retries per frame
            s8.write(FRAME)
            # Read until we see 'z' (ACK) or BEL/'\a' (NACK), up to 50ms
            deadline = time.time() + 0.05
            resp = b""
            while time.time() < deadline:
                ch = s8.read(1)
                if not ch:
                    break
                resp += ch
                if b'z' in resp or b'\x07' in resp:
                    break
            if b'z' in resp:
                accepted += 1
                print(f"    Frame {i+1:2d}/attempt {attempt+1}: ACK  {resp!r}")
                # Wait for TIM14 to finish the SENT frame before next submit
                time.sleep(SENT_FRAME_US / 1_000_000 + 0.001)
                break
            else:
                rejected += 1
                print(f"    Frame {i+1:2d}/attempt {attempt+1}: NACK {resp!r} — retrying in 2ms")
                time.sleep(0.002)
        else:
            print(f"    Frame {i+1:2d}: gave up after 20 attempts")

    print(f"\n    TX summary: {accepted} accepted, {rejected} total NACKs across {N_FRAMES} frames")

    # Give COM9 1 second to flush decoded frames
    time.sleep(1.0)
    rx_data = s9.read(s9.in_waiting or 2048)
    raw_str = rx_data.decode(errors='replace')

    if raw_str.strip():
        frames_510 = [f for f in raw_str.split('\r') if f.startswith('t510')]
        all_items  = [f for f in raw_str.split('\r') if f.strip()]
        print(f"\n    COM9 output ({len(all_items)} lines, {len(frames_510)} t510 frames):")
        for f in all_items[:20]:
            print(f"    {f!r}")
        if len(frames_510) == N_FRAMES:
            print(f"\n    ✓ All {N_FRAMES}/{N_FRAMES} frames received!")
        else:
            print(f"\n    ✗ Only {len(frames_510)}/{N_FRAMES} frames received.")
    else:
        print("    COM9 received NOTHING.")
        print("    --> Check: is PA4 (COM8) physically wired to PA2 (COM9)?")
        print("    --> Check: is COM9 firmware running RX mode?")

    s8.close()
    s9.close()
    print("\nDone.")

if __name__ == "__main__":
    main()
