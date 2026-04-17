#!/usr/bin/env python3
"""
Capture SENT signal from Saleae Logic 2 analyzer (channel 2) and export to CSV.
Requires: pip install logic2-automation

Usage:
    python capture_sent.py [duration_ms] [sample_rate_hz]
    
Arguments:
    duration_ms: Capture duration in milliseconds (default: 500)
    sample_rate_hz: Sampling rate in Hz (default: 1000000 = 1MHz)

Output:
    CSV file with timestamp and channel 2 data: logic_capture_sent_YYYY-MM-DD_HHMMSS.csv
"""

import sys
import os
from datetime import datetime


def capture_sent(duration_ms=500, sample_rate=None):
    """Capture SENT signal from Logic 2 analyzer and export to CSV."""
    
    try:
        from logic2_automation import Logic2Session
    except ImportError:
        print("ERROR: logic2-automation module not found")
        print("Install with: pip install logic2-automation")
        return False
    
    try:
        # Connect to Logic 2 application (must be running)
        print("Connecting to Logic 2 application...")
        session = Logic2Session()
        print("✓ Connected to Logic 2")
        
        # Get available devices
        devices = session.get_devices()
        if not devices:
            print("ERROR: No Logic analyzers detected")
            print("Please connect a Logic analyzer via USB")
            print("Make sure Logic 2 application is running")
            return False
        
        print(f"✓ Found {len(devices)} device(s)")
        
        # Select first device
        device = devices[0]
        print(f"Using device: {device.product_name if hasattr(device, 'product_name') else 'Logic Analyzer'}")
        
        # Configure capture
        duration_seconds = duration_ms / 1000.0
        print(f"\nCapture settings:")
        print(f"  Duration: {duration_ms} ms ({duration_seconds}s)")
        
        # Set sample rate
        if sample_rate is None:
            sample_rate = 1_000_000  # Default to 1 MHz
        
        print(f"  Sample rate: {sample_rate / 1_000_000:.1f} MHz")
        print(f"  Channel: 2 (SENT signal)")
        print(f"  Total samples: {int(sample_rate * duration_seconds):,}")
        
        # Configure channels
        print("\nConfiguring channel 2...")
        session.set_active_digital_channels([2])
        session.set_sample_rate(sample_rate)
        
        # Start capture
        print(f"\nStarting capture ({duration_ms}ms)...")
        session.start_capture(duration_seconds=duration_seconds)
        
        # Wait for capture to complete
        print("Waiting for capture to complete...")
        session.wait_until_finished()
        
        # Export to CSV
        timestamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
        csv_filename = f"logic_capture_sent_{timestamp}.csv"
        
        print(f"\nExporting to CSV: {csv_filename}")
        session.capture_to_file(
            filepath=csv_filename,
            radix="DEC"  # Decimal format
        )
        
        # Verify file was created
        if os.path.exists(csv_filename):
            file_size = os.path.getsize(csv_filename)
            print(f"✓ Capture complete!")
            print(f"  File: {csv_filename}")
            print(f"  Size: {file_size:,} bytes")
            
            # Try to show preview
            try:
                with open(csv_filename, 'r') as f:
                    lines = f.readlines()
                    print(f"  Rows: {len(lines) - 1:,} samples")
                    if len(lines) > 1:
                        print(f"  Header: {lines[0].strip()}")
                        print(f"  First sample: {lines[1].strip()}")
            except:
                pass
            
            return True
        else:
            print("✗ Error: CSV file was not created")
            return False
        
    except Exception as e:
        print(f"✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    # Parse command-line arguments
    duration_ms = 500
    sample_rate = None
    
    if len(sys.argv) > 1:
        try:
            duration_ms = int(sys.argv[1])
            if duration_ms <= 0:
                print("Duration must be positive")
                sys.exit(1)
        except ValueError:
            print(f"Invalid duration: {sys.argv[1]}")
            sys.exit(1)
    
    if len(sys.argv) > 2:
        try:
            sample_rate = int(sys.argv[2])
            if sample_rate <= 0:
                print("Sample rate must be positive")
                sys.exit(1)
        except ValueError:
            print(f"Invalid sample rate: {sys.argv[2]}")
            sys.exit(1)
    
    print("=" * 60)
    print("Saleae Logic 2 SENT Signal Capture")
    print("=" * 60)
    print()
    
    success = capture_sent(duration_ms, sample_rate)
    
    if success:
        print("\n" + "=" * 60)
        print("✓ Capture succeeded")
        print("=" * 60)
        sys.exit(0)
    else:
        print("\n" + "=" * 60)
        print("✗ Capture failed")
        print("=" * 60)
        sys.exit(1)
