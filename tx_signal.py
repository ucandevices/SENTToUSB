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

    print(f"\nSending SENT frames every 10ms. Watch PA4 on logic analyzer.")
    print("Press Ctrl+C to stop.\n")

    count = 0
    try:
        while True:
            s.write(FRAME)
            time.sleep(0.01)
            ack = s.read(s.in_waiting or 4)
            count += 1
            if count % 50 == 0:
                status = "ok" if b'z' in ack else f"?{ack!r}"
                print(f"  {count} frames sent, last ACK: {status}")
    except KeyboardInterrupt:
        print(f"\nStopped after {count} frames.")

    s.close()

if __name__ == "__main__":
    main()
