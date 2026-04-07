#!/usr/bin/env python3
"""
tx_test.py - Diagnose COM8 TX and verify reception on COM9.

Steps:
  1. Query COM8 debug counters (before anything)
  2. Start TX mode on COM8
  3. Send 1 frame, wait 100ms, query counters → see if TIM14 ISR fired
  4. Try sending more frames to confirm queue drains
  5. Monitor COM9 for t510 frames
"""
import serial, time, sys

COM8 = "COM8"   # TX device (STLINK attached)
COM9 = "COM9"   # RX device

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

    # --- Step 2: Open SLCAN channel, then switch to TX mode ---
    print("\n[2] Opening SLCAN channel (O) on COM8...")
    r = send_cmd(s8, "O", wait=0.05)
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
    r9 = send_cmd(s9, "t600101", wait=0.1)
    print(f"    start-RX ACK: {r9!r}")
    s9.reset_input_buffer()

    # --- Step 3: Send 1 frame, wait, check counters ---
    FRAME = b"t52050000123456\r"
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

    # --- Step 5: Continuously TX and monitor COM9 ---
    print("\n[7] Sending 10 frames with 50ms spacing, watching COM9 raw output...")
    s9.reset_input_buffer()
    for i in range(10):
        s8.write(FRAME)
        time.sleep(0.05)

    # Give COM9 1 second to output decoded frames
    time.sleep(1.0)
    rx_data = s9.read(s9.in_waiting or 2048)
    raw_str = rx_data.decode(errors='replace')

    if raw_str.strip():
        frames_510 = [f for f in raw_str.split('\r') if f.startswith('t510')]
        all_items  = [f for f in raw_str.split('\r') if f.strip()]
        print(f"    COM9 output ({len(all_items)} lines, {len(frames_510)} t510 frames):")
        for f in all_items[:20]:
            print(f"    {f!r}")
    else:
        print("    COM9 received NOTHING.")
        print("    --> Check: is PA4 (COM8) physically wired to PA2 (COM9)?")
        print("    --> Check: is COM9 firmware running RX mode?")

    s8.close()
    s9.close()
    print("\nDone.")

if __name__ == "__main__":
    main()
