#!/usr/bin/env python
"""Quick diagnostic to check SENT device communication."""

import serial
import time

TX_PORT = "COM15"
RX_PORT = "COM20"

def test_port(port_name):
    """Test communication with a single port."""
    print(f"\n{'='*60}")
    print(f"Testing {port_name}")
    print(f"{'='*60}")
    
    try:
        s = serial.Serial(port_name, 115200, timeout=1)
        time.sleep(0.2)
        
        # Test version command
        print(f"\n1. Sending: V\\r")
        s.write(b'V\r')
        time.sleep(0.2)
        resp = s.read(100)
        print(f"   Response: {resp!r}")
        print(f"   Decoded: {resp.decode('ascii', 'replace').strip()}")
        
        # Test serial number
        print(f"\n2. Sending: N\\r")
        s.write(b'N\r')
        time.sleep(0.2)
        resp = s.read(100)
        print(f"   Response: {resp!r}")
        print(f"   Decoded: {resp.decode('ascii', 'replace').strip()}")
        
        # Test RX mode command
        print(f"\n3. Sending: O\\r (Start RX)")
        s.write(b'O\r')
        time.sleep(0.2)
        resp = s.read(100)
        print(f"   Response: {resp!r}")
        print(f"   Decoded: {resp.decode('ascii', 'replace').strip()}")
        print(f"   Response length: {len(resp)}")
        print(f"   Response bool (bool(resp)): {bool(resp)}")
        
        # Test TX mode command
        print(f"\n4. Sending: t60002\\r (Start TX)")
        s.write(b't60002\r')
        time.sleep(0.2)
        resp = s.read(100)
        print(f"   Response: {resp!r}")
        print(f"   Decoded: {resp.decode('ascii', 'replace').strip()}")
        
        s.close()
        return True
        
    except Exception as e:
        print(f"Error: {e}")
        return False

if __name__ == "__main__":
    print("SENT Device Diagnostic Tool")
    print("Testing both devices...")
    
    tx_ok = test_port(TX_PORT)
    rx_ok = test_port(RX_PORT)
    
    print(f"\n{'='*60}")
    print(f"Summary:")
    print(f"  TX ({TX_PORT}): {'✓ OK' if tx_ok else '✗ FAILED'}")
    print(f"  RX ({RX_PORT}): {'✓ OK' if rx_ok else '✗ FAILED'}")
    print(f"{'='*60}")
