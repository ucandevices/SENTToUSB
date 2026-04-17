#!/usr/bin/env python3
"""
tx_signal.py - Continuously send SENT frames on COM8 for logic analyzer capture.
Press Ctrl+C to stop.

Usage:
  python tx_signal.py           # stream SENT frames
  python tx_signal.py --boot    # trigger DFU bootloader on COM8
"""
import serial, time, sys

COM8 = "COM8"

def boot_device(port):
    """Send 'boot' command to trigger DFU bootloader."""
    print(f"Sending boot command to {port}...")
    s = serial.Serial(port, 115200, timeout=0.5)
    s.write(b"boot\r")
    time.sleep(0.2)
    resp = s.read(s.in_waiting or 64)
    print(f"  boot -> {resp!r}")
    s.close()

def main():
    if "--boot" in sys.argv:
        boot_device(COM8)
        return

    print(f"Opening {COM8}...")
    try:
        s = serial.Serial(COM8, 115200, timeout=0.1)
    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    s.reset_input_buffer()

    def send(cmd):
        if not cmd.endswith('\r'):
            cmd += '\r'
        s.write(cmd.encode())
        time.sleep(0.05)
        return s.read(s.in_waiting or 32)

    # Number of data nibbles to transmit (4, 6, or 8)
    DATA_NIBBLES = 4

    # SENT frame payloads (status=0, data nibbles packed MSB-first into 4 bytes)
    #   4 nibbles [1,2,3,4] → packed=0x00001234 → data bytes 00 00 12 34
    #   6 nibbles [1,2,3,4,5,6] → packed=0x00123456 → data bytes 00 12 34 56
    FRAMES = {
        4: b"t52050000001234\r",   # D=[1,2,3,4]  ~453 µs
        6: b"t52050000123456\r",   # D=[1,2,3,4,5,6]  ~561 µs
        8: b"t52050012345678\r",   # D=[1,2,3,4,5,6,7,8]  ~669 µs
    }
    FRAME_US = {4: 460, 6: 520, 8: 580}
    FRAME = FRAMES[DATA_NIBBLES]
    SENT_FRAME_US = FRAME_US[DATA_NIBBLES]

    # Open channel, configure data nibbles, switch to TX mode
    print("Opening SLCAN channel and starting TX mode...")
    r = send("O")
    print(f"  O -> {r!r}")
    r = send(f"t0011{DATA_NIBBLES:02X}")
    print(f"  config nibbles={DATA_NIBBLES} -> {r!r}")
    r = send("t600102")
    print(f"  start TX -> {r!r}")

    print(f"\nSending SENT frames (ACK-paced). Watch PA4 on logic analyzer.")
    print("Press Ctrl+C to stop.\n")

    s.timeout = 0.05
    count = 0
    nacks = 0
    try:
        while True:
            s.write(FRAME)
            # Wait for z (ACK) or BEL (NACK/busy)
            deadline = time.time() + 0.05
            resp = b""
            while time.time() < deadline:
                ch = s.read(1)
                if not ch:
                    break
                resp += ch
                if b'z' in resp or b'\x07' in resp:
                    break
            if b'z' in resp:
                count += 1
                # Wait for TIM14 to finish before next submit
                time.sleep(SENT_FRAME_US / 1_000_000 + 0.001)
            else:
                nacks += 1
                time.sleep(0.002)   # 2ms back-off on busy
            if (count + nacks) % 100 == 0 and (count + nacks) > 0:
                print(f"  {count} sent, {nacks} NACKs")
    except KeyboardInterrupt:
        print(f"\nStopped: {count} frames sent, {nacks} NACKs.")

    s.close()

if __name__ == "__main__":
    main()
