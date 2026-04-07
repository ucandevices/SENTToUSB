"""Quick check: open COM9 and listen for SENT frames from COM8 TX."""
import serial, time, sys

TX_PORT = "COM8"
RX_PORT = "COM9"
LISTEN_S = 5.0

def main():
    # Confirm TX device is alive
    try:
        tx = serial.Serial(TX_PORT, 115200, timeout=1)
        tx.write(b'V\r'); time.sleep(0.2)
        ver = tx.read(20).decode('ascii','replace').strip()
        print(f"{TX_PORT} (TX): {ver}")
        tx.close()
    except Exception as e:
        print(f"{TX_PORT} error: {e}")

    # Open RX and listen
    try:
        rx = serial.Serial(RX_PORT, 115200, timeout=0.1)
        rx.write(b'O\r'); time.sleep(0.2)
        ack = rx.read(10).decode('ascii','replace').strip()
        print(f"{RX_PORT} (RX) open ack: {ack!r}")
        print(f"Listening {LISTEN_S}s for SENT frames...")
        t0 = time.time()
        frames = []
        while time.time() - t0 < LISTEN_S:
            line = rx.readline()
            if line:
                s = line.decode('ascii','replace').strip()
                if s:
                    frames.append(s)
                    print(f"  {s}")
        rx.close()
        print(f"\nTotal: {len(frames)} SLCAN frames received")
        if frames:
            f = frames[0]
            if f.startswith('t') and len(f) >= 6:
                try:
                    dlc = int(f[4])
                    data = bytes.fromhex(f[5:5+dlc*2])
                    print(f"First frame decoded: {list(data)}")
                except: pass
        return len(frames) > 0
    except Exception as e:
        print(f"{RX_PORT} error: {e}")
        return False

if __name__ == "__main__":
    ok = main()
    sys.exit(0 if ok else 1)
