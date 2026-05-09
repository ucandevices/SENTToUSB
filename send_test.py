#!/usr/bin/env python3
"""Set tick to 3.0 us, enter TX mode, send N test frames spaced out.

Repeated transmission lets a logic analyzer trigger on any sync edge and
guarantees at least one full frame (sync + 9 intervals + pause) lands in
the capture window."""
import serial, sys, time

PORT  = sys.argv[1] if len(sys.argv) > 1 else "COM8"
COUNT = int(sys.argv[2]) if len(sys.argv) > 2 else 20

s = serial.Serial(PORT, 115200, timeout=0.1)
s.reset_input_buffer()

def send(cmd, wait=0.05):
    s.write((cmd + "\r").encode())
    time.sleep(wait)
    return s.read(256)

print(f"{'O':30s} -> {send('O')!r}")
print(f"{'t6003051E00':30s} -> {send('t6003051E00')!r}")  # tick = 3.0 us
print(f"{'t600102':30s} -> {send('t600102')!r}")          # TX mode

for i in range(COUNT):
    r = send("t52050100123456", wait=0.02)
    print(f"  frame {i+1}: {r!r}")
    if b'z' not in r:
        time.sleep(0.005)
        continue
    time.sleep(0.002)   # let the SENT frame finish on the wire (~580 us)

print(f"sent {COUNT} frames")
time.sleep(0.1)
s.close()
