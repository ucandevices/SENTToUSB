#!/usr/bin/env python3
"""
tx_signal.py - Continuously send SENT frames on COM8 for logic analyzer capture.
Press Ctrl+C to stop.
"""
import serial, time, sys

COM8 = "COM8"

def main():
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

    # Open channel, switch to TX mode
    print("Opening SLCAN channel and starting TX mode...")
    r = send("O")
    print(f"  O -> {r!r}")
    r = send("t600102")
    print(f"  start TX -> {r!r}")

    # SENT frame: status=0, nibbles [0,0,1,2,3,4,5,6] (first 6 used = [0,0,1,2,3,4])
    FRAME = b"t52050000123456\r"
    SENT_FRAME_US = 520   # µs — conservative SENT frame duration at 3µs/tick

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
