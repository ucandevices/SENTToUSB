---
name: logic-capture-sent
description: 'Capture SENT signal from Logic analyzer (channel 2) and export to CSV. Use for analyzing SENT protocol data and debugging signal timing.'
argument-hint: '[500|1000|2000] capture duration in ms'
---

# Logic Capture SENT Signal

## When to Use

- Capture live SENT protocol signals from the Logic analyzer
- Export captured waveform data to CSV for analysis and post-processing
- Debug signal timing and verify protocol compliance  
- Generate test data for SENT decoder development
- Troubleshoot signal integrity issues

## Requirements

**Hardware:**
- Saleae Logic analyzer (any model)
- Connected to USB with SENT signal on **channel 2**
- Channel 2 configured for digital input
- Sample rate: 500kHz minimum (1-5MHz recommended for SENT)

**Software:**
- Saleae Logic 2 at `C:\Program Files\Logic\Logic.exe`
- Python 3.6+ with logic2-automation (`pip install logic2-automation`)

## Quick Start - Manual Method

1. **Open Logic 2** from Start menu or `C:\Program Files\Logic\Logic.exe`
2. **Verify channel 2** is enabled (connected to SENT signal pin)
3. **Set sample rate** to 1 MHz (Device → Settings → Sample Rate)
4. **Click the Capture button** (or press Spacebar)
5. **Wait** for 500ms-1s depending on what you're capturing
6. **Pause/Stop capture** (press Spacebar again or wait for auto-stop)
7. **Export data**: Hamburger menu → Export → Export to File
8. **Select CSV format** and check "Include Headers"
9. **Select channel 2** in the channel list before exporting
10. **Save** as `sent_capture.csv`

CSV file is ready for analysis in Python, Excel, or your SENT decoder.

## Automated Capture - Python

For integration with test scripts and CI/CD pipelines:

### Setup (First Time Only)

```bash
pip install logic2-automation
```

### Run Capture (Recommended Method)

```bash
cd C:\Users\LJ\STM32CubeIDE\workspace_1.19.0\SENTToUSB
python .github\skills\logic-capture-sent\scripts\capture_sent.py [duration_ms] [sample_rate_hz]
```

**Examples:**
```bash
# 500ms at default 1MHz sample rate
python .github\skills\logic-capture-sent\scripts\capture_sent.py 500

# 1 second at 2MHz sample rate (higher precision)
python .github\skills\logic-capture-sent\scripts\capture_sent.py 1000 2000000

# 500ms at 5MHz (maximum precision for timing analysis)
python .github\skills\logic-capture-sent\scripts\capture_sent.py 500 5000000
```

**Output:** CSV file saved to current directory with name `logic_capture_sent_YYYY-MM-DD_HHMMSS.csv`

**Note**: Requires Logic 2 application to be running first

## CSV Data Analysis

The exported CSV contains two columns:
- **Time (s)**: Timestamp of each sample in seconds
- **Channel 3**: Digital value (0 or 1) representing SENT signal

### Analyze with Python

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load CSV
df = pd.read_csv('sent_capture.csv')

# Plot signal over time
plt.figure(figsize=(12, 4))
plt.plot(df['Time (s)'], df['Channel 3'], drawstyle='steps-post')
plt.xlabel('Time (s)')
plt.ylabel('SENT Signal Level')
plt.title('SENT Protocol Capture')
plt.grid(True)
plt.show()

# Find edges (falling edges are SENT timing markers)
edges = df['Channel 3'].diff().fillna(0)
falling_edges = df[edges < 0]['Time (s)'].values
print(f"Found {len(falling_edges)} edges")
```

### Analyze with Excel

1. Open exported CSV file in Excel
2. Insert a chart → XY (Scatter) with Lines
3. Set X-axis to **Time (s)**, Y-axis to **Channel 3**
4. Zoom in to measure pulse widths and intervals
5. Verify timing against SAE J2716 SENT specifications

## SENT Signal Specifications

Reference for verifying captured data:
- **Sync pulse**: 56 nominal ticks (typically represents ~168µs at 3µs/tick)
- **Data nibble**: 12-27 ticks per nibble (36-81µs)
- **Tick period**: ~3µs for typical sensors (varies by design)
- **Frame rate**: Usually 100-500 frames/second
- **Sampling requirement**: Minimum 10× signal frequency (10 MHz for worst case)

## Troubleshooting

| Issue | Solution |
|---|---|
| **No devices detected** | Ensure Logic 2 is running; check USB connection; restart Logic 2 |
| **logic2-automation module not found** | Run `pip install logic2-automation` |
| **Channel 3 not enabled** | Logic 2 → Device → Channels → Enable Channel 3 |
| **Signal looks like noise** | Increase sample rate; check wiring; verify signal is on correct pin |
| **CSV export fails** | Check folder permissions; ensure disk space available |
| **File not created** | Check current working directory; verify write permissions |
| **Connection refused** | Start Logic 2 application first (Python API requires it running) |

## Files

- [capture_sent.py](./scripts/capture_sent.py) - Python script using logic2-automation for automated SENT capture

## Related Skills

- **STM32 Build and Flash** - Compile and program the SENT transmitter firmware on STM32F042
