# SENT USB Converter Integration Test

Comprehensive pytest-based integration test for bidirectional SENT communication between two SENTToUSB devices.

## Overview

The test suite verifies:
- ✓ Both TX and RX devices are present and responsive
- ✓ Devices can enter RX/TX modes
- ✓ SENT frames transmit and receive correctly
- ✓ Multiple tick times work: 3µs, 6µs, 9µs, 12µs
- ✓ Frame sequences are handled correctly
- ✓ Tick time settings persist across multiple frames

## Setup

### Hardware Requirements
- 2x SENTToUSB devices connected to USB
- Default configuration: **COM15 (TX)** and **COM20 (RX)**
  - Customize by modifying `TX_PORT` and `RX_PORT` in the test class

### Software Requirements

1. **Install pytest and pyserial:**
   ```bash
   pip install -r test_requirements.txt
   ```

   Or manually:
   ```bash
   pip install pytest>=7.0.0 pyserial>=3.5
   ```

## Running Tests

### Run all tests with verbose output:
```bash
pytest test_sent_integration.py -v -s
```

### Run only tick-time parametrized tests:
```bash
pytest test_sent_integration.py::TestSENTIntegration::test_tx_rx_with_tick_time -v -s
```

### Run a specific tick time (e.g., 3µs):
```bash
pytest test_sent_integration.py::TestSENTIntegration::test_tx_rx_with_tick_time[3.0] -v -s
```

### Run tests without capturing output (real-time logging):
```bash
pytest test_sent_integration.py -v -s --capture=no
```

### Generate test report:
```bash
pytest test_sent_integration.py -v --tb=short --html=report.html
```

## Test Cases

### Basic Connectivity Tests
| Test | Purpose |
|------|---------|
| `test_devices_present_and_responsive` | Verify both devices enumerate and respond to version query |
| `test_can_enter_rx_mode` | Verify RX device accepts RX mode command |
| `test_can_enter_tx_mode` | Verify TX device accepts TX mode command |

### Communication Tests
| Test | Purpose |
|------|---------|
| `test_tx_rx_with_tick_time[X]` | Send/receive frames at tick time X µs (parametrized for 3, 6, 9, 12) |
| `test_tx_rx_with_tick_and_frame_size[X-5]` | Combined parametrized test: tick time X µs with DLC=5 (firmware accepts DLC=5 only on 0x520) |
| `test_multiple_frames_sequence` | Verify 10-frame sequence transmits and receives in order |
| `test_rx_only_without_tx` | Verify RX device can listen without errors on quiet channel |
| `test_tick_time_persistence_across_frames[X]` | Verify tick time setting persists for 5 consecutive frames |
| `test_all_frame_sizes_with_default_tick` | Sanity-check default-tick TX (DLC=5 baseline) |

> **Note:** the firmware bridge rejects 0x520 TX frames with `DLC < 5`
> (see `handle_tx_can_frame` in `Core/Src/sent_bridge.c`).  Tests therefore
> only exercise DLC=5.  The number of data nibbles encoded inside the
> frame is set via the 0x001 config frame, not via DLC.

## Test Frames

### SLCAN Command Format
```
t<ID><DLC><data>\r   Standard frame (11-bit CAN ID)
T<ID><DLC><data>\r   Extended frame (29-bit CAN ID)
```

### Control Commands (CAN ID 0x600)
```
t600101\r       Start RX mode      (DLC=1, data=[0x01])
t600102\r       Start TX mode      (DLC=1, data=[0x02])
t600103\r       Stop               (DLC=1, data=[0x03])
t600305LLHH\r   Set TX tick        (DLC=3, data=[0x05, LL, HH])
                LLHH = little-endian 16-bit value in 0.1 µs units
                e.g. 3.0 µs → tick_x10 = 30 = 0x001E → t6003051E00
```

### TX Data Frames (CAN ID 0x520)
```
t52050100123400\r
  ID=0x520, DLC=5, data=[0x01, 0x00, 0x12, 0x34, 0x00]
  data[0]   = status nibble (low 4 bits): 0x1
  data[1..3]= 6 nibbles MSB-first: [0x0, 0x0, 0x1, 0x2, 0x3, 0x4]
  data[4]   = unused for 6-nibble config (default), but DLC must still be 5
```

### RX Decoded Frames (CAN ID 0x510, default)
```
t5103AABBCC\r
  Fast channel output: 3 bytes (6 nibbles)
  DLC=3 for fast-channel only
  DLC=7 when slow-channel message completes
```

## Customizing Port Configuration

Edit the test class at the top of `test_sent_integration.py`:

```python
class TestSENTIntegration:
    TX_PORT = "COM15"      # Change to your TX device port
    RX_PORT = "COM20"      # Change to your RX device port
    TICK_TIMES = [3.0, 6.0, 9.0, 12.0]  # Modify tested tick times
```

## Expected Output

The suite collects 18 test cases:

| Group | Count |
|-------|-------|
| Connectivity (`test_devices_present_and_responsive`, `test_can_enter_rx_mode`, `test_can_enter_tx_mode`) | 3 |
| `test_tx_rx_with_tick_time` (4 tick × 1) | 4 |
| `test_tx_rx_with_tick_and_frame_size` (4 tick × DLC=5) | 4 |
| `test_multiple_frames_sequence` | 1 |
| `test_rx_only_without_tx` | 1 |
| `test_tick_time_persistence_across_frames` (4 tick × 1) | 4 |
| `test_all_frame_sizes_with_default_tick` | 1 |
| **Total** | **18** |

A full pass typically takes 60–90 s; each tick-parametrized case waits
several frame periods to drain the RX side before asserting.

## Debugging

### Enable detailed logging:
Add to test file or run with environment variable:
```bash
pytest test_sent_integration.py -v -s --log-cli-level=DEBUG
```

### Check device connectivity before running tests:
```bash
python -c "import serial; s = serial.Serial('COM15', 115200, timeout=1); s.write(b'V\r'); print(s.read(50).decode())"
```

### Verify devices are present:
```bash
# On Windows PowerShell:
Get-PnpDevice -Class Ports | Where-Object {$_.Status -eq 'OK'}

# Or use the device manager GUI
```

## Troubleshooting

### "Failed to connect to COM15/COM20"
- Check devices are plugged in
- Verify correct COM port numbers (use Device Manager)
- Update port numbers in test class

### "No frames received"
- Verify firmware is built and flashed to both devices
- Check USB cable connections
- Try with loopback test: connect TX and RX outputs together
- Review hardware timing fixes in [SENT_timing_fixes.md](SENT_timing_fixes.md)

### "DLC mismatch" or frame parse errors
- Check SLCAN command format is correct
- Verify data length matches DLC field
- Review SLCAN protocol in main [README.md](README.md)

### Test passes locally but fails in CI/CD
- Serial port enumeration may differ in CI environment
- Use environment variables to override port numbers
- Consider adding parametrization for multiple port configurations

## Performance Notes

- Each tick-time test takes ~5-10 seconds (including device initialization, frame transmission, and RX timeout)
- Full test suite (~13 tests) completes in 45-60 seconds
- Frame latency: typically 10-100 ms depending on USB CDC overhead
- RX timeout set to 2.0s per frame batch; adjust if seeing false negatives

## Related Files

- **Test Results Log:** See [SENT_debugging_log.md](SENT_debugging_log.md) for previous test history
- **Timing References:** See [SENT_timing_fixes.md](SENT_timing_fixes.md) for tick time calculations
- **Python Utilities:** 
  - [sent_viewer.py](sent_viewer.py) — GUI SLCAN monitor
  - [check_rx.py](check_rx.py) — Simple loopback test
  - [mlx_viewer.py](mlx_viewer.py) — MLX90377 angle display

## License

Test suite follows the same license as the SENTToUSB project.
