# Quick Start: Running the Integration Tests

## 1. Install Dependencies (First Time Only)

```bash
pip install -r test_requirements.txt
```

This installs:
- `pytest` — test framework
- `pyserial` — serial port communication

## 2. Verify Device Connection

Make sure both devices are connected:
```bash
python diagnostic.py
```

Expected output:
```
TX (COM15): ✓ OK
RX (COM20): ✓ OK
```

If devices show as FAILED, check:
- Devices are plugged into USB
- Correct COM port numbers (edit `test_sent_integration.py` if needed)
- Firmware is flashed on both devices

## 3. Run Tests

### Option A: Using pytest directly (Full control)

```bash
# All 29 tests
python -m pytest test_sent_integration.py -v -s

# Just connectivity tests (quick, ~10 seconds)
python -m pytest test_sent_integration.py::TestSENTIntegration::test_devices_present_and_responsive test_sent_integration.py::TestSENTIntegration::test_can_enter_rx_mode test_sent_integration.py::TestSENTIntegration::test_can_enter_tx_mode -v -s

# Just tick time tests (4 tests)
python -m pytest test_sent_integration.py -k "test_tx_rx_with_tick_time" -v -s

# Just frame size tests (16 parametrized tests)
python -m pytest test_sent_integration.py -k "test_tx_rx_with_tick_and_frame_size" -v -s

# Single test (e.g., 3µs tick with DLC=4)
python -m pytest "test_sent_integration.py::TestSENTIntegration::test_tx_rx_with_tick_and_frame_size[3.0-4]" -v -s
```

### Option B: Using Python runner (Convenient shortcuts)

```bash
# All tests
python run_tests.py all

# Connectivity only (quick check)
python run_tests.py quick

# Frame size testing
python run_tests.py framesize

# All tick × frame combinations (16 tests)
python run_tests.py combined

# Single tick time (e.g., 3µs)
python run_tests.py 3

# With detailed logging
python run_tests.py debug

# Generate HTML report
python run_tests.py report
```

## 4. Understanding Test Results

Each test shows output like:
```
test_sent_integration.py::TestSENTIntegration::test_devices_present_and_responsive PASSED
```

✅ **PASSED** — Test succeeded
❌ **FAILED** — Test failed with error details shown below

Summary at end:
```
==================== 29 passed in 87.23s ====================
```

## 5. Customizing Port Numbers

Edit [test_sent_integration.py](test_sent_integration.py#L271) line ~271:

```python
class TestSENTIntegration:
    TX_PORT = "COM15"   # Change to your TX device port
    RX_PORT = "COM20"   # Change to your RX device port
```

Then re-run tests.

## 6. Interpreting Test Logs

With `-s` flag, you'll see detailed logs:

```
2026-05-12 14:30:45,123 - INFO - Connected to COM15
2026-05-12 14:30:45,234 - INFO - TX device version: V0101
2026-05-12 14:30:45,345 - INFO - RX device version: V0101
...
2026-05-12 14:30:50,123 - INFO - ✓ Successfully transmitted and received 3 frames at 3.0 µs
```

This helps diagnose issues:
- Missing `Connected` messages → Port not found
- Missing `version` → Device not responding  
- Missing frame count → Communication failed

## 7. Test Categories

### Connectivity Tests (3 tests, ~10 seconds)
- Device version/serial queries
- RX mode entry
- TX mode entry

### Tick Time Tests (4 tests, ~20 seconds)
- Communication at 3, 6, 9, 12 µs tick times
- Default frame size (DLC=5)

### Frame Size Tests (16 parametrized tests, ~40 seconds)
- Combines 4 tick times × 4 frame sizes (DLC 2-5)
- Tests different nibble counts
- Each combo sends 3 frames with different patterns

### Sequence & Persistence Tests (6 tests, ~20 seconds)
- 10-frame rapid transmission
- Silent listening (RX only)
- Tick time persistence across 5 frames

## 8. Common Commands Cheat Sheet

```bash
# Quick connectivity check (~10s)
python -m pytest test_sent_integration.py::TestSENTIntegration::test_devices_present_and_responsive test_sent_integration.py::TestSENTIntegration::test_can_enter_rx_mode test_sent_integration.py::TestSENTIntegration::test_can_enter_tx_mode -v -s

# All tick time tests (~20s)
python -m pytest test_sent_integration.py -k "test_tx_rx_with_tick_time" -v -s

# All frame size combos (~40s)
python -m pytest test_sent_integration.py -k "test_tx_rx_with_tick_and_frame_size" -v -s

# Full test suite (~90s)
python -m pytest test_sent_integration.py -v -s

# Test specific parameters
python -m pytest test_sent_integration.py::TestSENTIntegration::test_tx_rx_with_tick_and_frame_size[9.0-3] -v -s
```

## 9. Troubleshooting

### Test fails with "No frames received"
- Verify firmware is correctly flashed
- Check devices with: `python diagnostic.py`
- Ensure USB cables are firmly connected
- Try loopback test: connect TX (COM15) PA0 output to RX (COM20) PA2 input

### pytest command not found
```bash
python -m pytest --version  # Use full path if needed
pip install pytest          # Reinstall if missing
```

### Port already in use / Permission denied
- Close other serial terminals (PuTTY, Minicom, etc.)
- Unplug and replug USB devices
- Restart Python interpreter

### Device responds with empty acknowledgments
- This is normal! The diagnostic tool shows raw responses
- Tests accept any response (including `\r` or `\x07`) as ACK

---

See [TEST_INTEGRATION_README.md](TEST_INTEGRATION_README.md) for detailed documentation.
