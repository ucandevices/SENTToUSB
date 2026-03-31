# SENTToUSB — STM32F042 USB ↔ SENT Bridge

A USB CDC (Virtual COM Port) adapter that decodes and encodes **SAE J2716 SENT** sensor signals.
Connect any SENT sensor to a PC and receive decoded frames over a standard serial port using the
**SLCAN** text protocol.

Full documentation and product page: **https://ucandevices.github.io**

---

## Table of Contents

1. [What it does](#what-it-does)
2. [Hardware](#hardware)
3. [Pinout](#pinout)
4. [Getting Started](#getting-started)
5. [SLCAN Protocol Reference](#slcan-protocol-reference)
6. [CAN Frame Reference](#can-frame-reference)
7. [Python Tools](#python-tools)
8. [Sensor Presets](#sensor-presets)
9. [Learn Mode](#learn-mode)
10. [TX Mode](#tx-mode)
11. [Firmware Defaults](#firmware-defaults)
12. [Quick Reference Card](#quick-reference-card)

---

## What it does

- **Receives** SENT signals from automotive sensors (SAE J2716 / ISO 21097)
- **Decodes** each frame: sync detection, nibble extraction, CRC validation
- **Sends** decoded data to the PC as SLCAN `t`-frames over USB CDC
- **Transmits** SENT frames synthesised from host-supplied CAN data (TX mode)
- **Auto-detects** tick period, nibble count and CRC mode via Learn mode
- Supports 4-, 6- and 8-nibble sensors, both CRC modes, seeds 0x03 and 0x05
- USB-C connector, no driver needed on Windows 10+, Linux, macOS

---

## Hardware

| Item | Value |
|------|-------|
| MCU | STM32F042K6 |
| Clock | 48 MHz HSI48 (no external crystal) |
| USB | Full-Speed 12 Mbit/s, CDC class (Virtual COM Port) |
| SENT RX timer | TIM2 CH3 — 48 MHz input capture, 16-bit counter |
| SENT TX timer | TIM14 — prescaler 143 → 1 tick = 3 µs |
| Connector | USB-C |
| Size | 17 × 28 mm (without USB connector) |
| Bootloader | USB DFU compatible |

---

## Pinout

| Pin | Direction | Function |
|-----|-----------|----------|
| PA2 | Input | SENT RX — TIM2 CH3 rising-edge capture |
| PA1 | Output | SENT TX — TIM14-driven push-pull |

- **RX pull-up:** fitted on the board by default — no external pull-up needed.
- **TX pull-up:** optional, enable by closing the onboard solder jumper.

---

## Getting Started

1. Plug the device into a USB port.
2. The OS creates a Virtual COM Port (`COM3` on Windows, `/dev/ttyACM0` on Linux).
3. Open the port at **any baud rate** — USB CDC ignores baud rate settings.
4. Send `O\r` to start receiving SENT data.

Minimal session:
```
→  O\r                              open channel / start RX with default config
←  \r                               ACK

→  t001706000305050510\r            configure: 6 nibbles, DATA_ONLY, seed=0x03,
←  z\r                                  tick 2.5–5 µs, output CAN ID=0x510

←  t5103AABBCC\r                    decoded SENT frame (repeats ~100–1000×/s)

→  C\r                              stop
←  \r
```

---

## SLCAN Protocol Reference

All commands are plain ASCII terminated with `\r` (CR, 0x0D).
Responses end with `\r`; the NACK is a bare `\a` (BEL, 0x07).

### Commands

| Command | Description | Response |
|---------|-------------|----------|
| `O\r` | Open channel — starts SENT RX | `\r` |
| `L\r` | Listen-only open — same as `O` | `\r` |
| `C\r` | Close channel — stops all activity | `\r` |
| `V\r` | Hardware version query | `V0101\r` |
| `v\r` | Firmware version query | `v0101\r` |
| `N\r` | Serial number query | `N` + 4 hex digits unique to each MCU |
| `F\r` | Status flags | `F00\r` |
| `t<ID><DLC><DATA>\r` | Send a CAN frame (control / config / TX data) | `z\r` or `\a` |

The serial number returned by `N` is a 16-bit XOR-fold of the STM32F042 96-bit unique device ID,
so each unit reports a different value.

### Responses

| Response | Meaning |
|----------|---------|
| `\r` | Command ACK |
| `\a` | NACK — malformed command or invalid config |
| `z\r` | Frame ACK (reply to a `t` command) |
| `t<ID><DLC><DATA>\r` | Decoded SENT frame pushed to host (unsolicited) |

---

## CAN Frame Reference

All frames use standard 11-bit IDs with the `t` prefix.

### Host → Device

#### `0x001` — Configuration (DLC = 7)

| Byte | Field | Notes |
|------|-------|-------|
| B0 | Data nibbles | 4, 6, or 8 |
| B1 | CRC mode | 0 = DATA\_ONLY, 1 = STATUS\_AND\_DATA |
| B2 | CRC init seed | 0x03 (SAE APR2016) or 0x05 (legacy/GM) |
| B3 | Min tick | units of 0.5 µs (e.g. 5 → 2.5 µs) |
| B4 | Max tick | units of 1 µs (e.g. 5 → 5.0 µs) |
| B5–B6 | Output RX CAN ID | big-endian, 11-bit (e.g. 0x510 → `05 10`) |

Example — 6 nibbles, DATA\_ONLY, seed=0x03, tick 2.5–5 µs, output ID=0x510:
```
t001706000305050510\r
```

#### `0x600` — Control (DLC = 1)

| Byte | Action |
|------|--------|
| `01` | Start RX — begin receiving and decoding SENT frames |
| `02` | Start TX — begin transmitting frames supplied via `0x520` |
| `03` | Stop — halt all activity |
| `04` | Learn — auto-detect tick, nibble count and CRC mode |

```
t600101\r    start RX
t600102\r    start TX
t600103\r    stop
t600104\r    start learn mode
```

Learn mode: device replies `z\r` immediately, streams normal decoded frames while searching,
then sends one `0x601` result frame and **stops RX** — send `O\r` or `t600101\r` to resume.

#### `0x520` — TX Frame Data (DLC ≥ 5, TX mode only)

| Byte | Field |
|------|-------|
| B0 | Status nibble (low 4 bits) |
| B1–B4 | Data nibbles packed big-endian (nibble[0] at bits 31..28) |
| B5–B6 | Pause ticks, little-endian (optional, default 12) |

Example — status=1, nibbles [1,2,3,4,5,6] → packed 0x00123456:
```
t52050100123456\r
```

### Device → Host

#### Configured ID (default `0x510`) — Decoded RX Frame

Pushed for every CRC-valid decoded SENT frame. Two nibbles packed per byte (high nibble first).

```
t5103AABBCC\r     3 bytes for a 6-nibble sensor
```

Unpack: `nibble[0] = byte[0] >> 4`, `nibble[1] = byte[0] & 0x0F`, etc.

The output CAN ID is set via B5–B6 of the `0x001` config frame (default 0x510).

#### `0x601` — Learn Result (DLC = 4)

Sent once when learn mode locks.

| Byte | Field |
|------|-------|
| B0–B1 | Learned tick × 10 in units of 0.1 µs, little-endian (e.g. `1E 00` = 30 → **3.0 µs**) |
| B2 | Nibble count locked: 4, 6, or 8 |
| B3 | CRC mode: 0 = DATA\_ONLY, 1 = STATUS\_AND\_DATA |

---

## Python Tools

### Requirements

```
pip install pyserial
```

Python 3.10+ (tkinter included with most Python distributions).

---

### sent\_viewer.py — GUI Monitor

Real-time GUI for monitoring and transmitting SENT frames.

```
python sent_viewer.py
```

**Left panel — controls:**

| Control | Description |
|---------|-------------|
| Port selector | Lists available COM ports with auto-refresh |
| Connect / Disconnect | Opens port, sends `O\r`; close sends `C\r` |
| Preset | Loads all parameters for MLX90377, 04L 906 051 L, or GM 12643955 |
| Tick Size (µs) | Sensor tick reference; used to compute min/max tick for config frame |
| Data Nibbles | 4, 6, or 8 |
| CRC Seed | 0x03 (SAE APR2016) or 0x05 (legacy/GM) |
| CRC Mode | DATA\_ONLY or STATUS\_AND\_DATA |
| RX CAN ID | Output CAN ID of decoded frames (default 0x510) |
| Apply Config | Sends current parameters as a `0x001` config frame |
| Learn | Sends `t600104\r`; updates fields automatically from the `0x601` result |
| Pause / Clear | Freeze display or reset counters and table |

**Right panel — display:**

- Large blue hex bytes — last decoded frame data
- Nibble row — individual nibbles unpacked
- Stats bar — frame count, CRC errors, sync errors, rate (fps)
- Received Frames table — scrollable history with timestamp, CAN ID, DLC, data, nibbles
- Raw Log — colour-coded SLCAN traffic

**TX SENT Frame panel** (right of the live display):

| Field | Description |
|-------|-------------|
| Status | 1 hex nibble (status field of the SENT frame) |
| Nibbles | Hex string of data nibbles (e.g. `123456` for 6 nibbles) |
| Period (ms) | Repeat interval; minimum 10 ms |
| Send | One-shot: sends one frame then returns to RX mode |
| Start / Stop | Periodic: switches to TX mode, sends at the set period; Stop resumes RX |

---

### sent\_test.py — CLI Test Tool

Command-line tool for quick validation without the GUI.

```
python sent_test.py COM8
python sent_test.py COM8 --nibbles 6 --crc-mode 0 --seed 0x03 --can-id 0x510
python sent_test.py COM8 --learn
```

Options:

| Flag | Default | Description |
|------|---------|-------------|
| `port` | (required) | Serial port, e.g. `COM8` |
| `--nibbles` | 6 | Data nibbles: 4, 6, or 8 |
| `--crc-mode` | 0 | 0 = DATA\_ONLY, 1 = STATUS\_AND\_DATA |
| `--seed` | 0x03 | CRC init seed |
| `--tick-min` | 5 | Min tick in units of 0.5 µs |
| `--tick-max` | 5 | Max tick in units of 1 µs |
| `--can-id` | 0x510 | Output RX CAN ID |
| `--learn` | off | Run learn mode instead of fixed config |
| `--timeout` | 10.0 | Listen duration in seconds |

The tool queries `V`, `v`, `N` at startup, sends the config or learn command, then prints all
received frames for the timeout duration.

---

## Sensor Presets

| Sensor | Tick | Nibbles | CRC Mode | Seed |
|--------|------|---------|----------|------|
| MLX90377 (angle) | 3 µs | 6 | DATA\_ONLY | 0x03 |
| 04L 906 051 L (VW/Audi DPF) | ~3 µs | 6 | STATUS\_AND\_DATA | 0x05 |
| GM 12643955 (MAP) | ~3 µs | 6 | DATA\_ONLY | 0x05 |

### MLX90377 nibble decode

```
Frame bytes: A5 3C 7F
Nibbles:     A  5  3  C  7  F

angle = (A<<8 | 5<<4 | 3) = 0xA53
mag   = (C<<8 | 7<<4 | F) = 0xC7F
```

---

## Learn Mode

When the sensor type or tick period is unknown:

```
t600104\r
```

The firmware tries all combinations of {4, 6, 8} nibbles × {DATA\_ONLY, STATUS\_AND\_DATA} CRC
modes. It requires 3 consecutive CRC-valid frames before locking. Once locked it sends one
`0x601` result frame and stops RX.

Resume after learning:
```
O\r          (or t600101\r)
```

The `sent_viewer.py` **Learn** button handles this automatically — it sends the learn command,
parses the `0x601` result, and populates the parameter fields.

---

## TX Mode

The device synthesises and transmits SENT frames on PA1.

```
t600102\r              start TX mode
t52050100123456\r      transmit: status=1, nibbles [1,2,3,4,5,6]
t600101\r              switch back to RX mode
```

TX timing: TIM14 at 48 MHz, prescaler 143 → 1 tick = 3 µs.
Frame sequence: sync (56 ticks) + status nibble + N data nibbles + CRC nibble + pause (default 12 ticks).

The **TX SENT Frame** panel in `sent_viewer.py` automates mode switching:
- **Send** fires a single frame and returns to RX
- **Start/Stop** sends frames at the configured period (≥ 10 ms) until stopped

---

## Firmware Defaults

| Parameter | Default |
|-----------|---------|
| SENT tick | 3 µs (min accepted 2.5 µs) |
| Data nibbles | 6 |
| CRC mode | DATA\_ONLY |
| CRC seed | 0x03 (SAE APR2016) |
| Output CAN ID | 0x510 |
| TX pause ticks | 12 |
| RX active at power-on | No — starts after `O\r` or `t600101\r` |

---

## Quick Reference Card

```
OPEN / START RX        O\r
CLOSE / STOP           C\r
VERSION                V\r  ->  V0101\r
FIRMWARE               v\r  ->  v0101\r
SERIAL NUMBER          N\r  ->  Nxxxx\r  (unique per device)
STATUS FLAGS           F\r  ->  F00\r

START RX  (CAN ctrl)   t600101\r
START TX  (CAN ctrl)   t600102\r
STOP      (CAN ctrl)   t600103\r
LEARN     (CAN ctrl)   t600104\r

CONFIG (6-nibble, DATA_ONLY, seed=0x03, tick 2.5-5us, ID=0x510)
                       t001706000305050510\r

TX FRAME (status=1, nibbles 1,2,3,4,5,6)
                       t52050100123456\r

DECODED DATA OUT       t5103AABBCC\r   (ID=0x510, DLC=3)
LEARN RESULT           t60104LLHHNCM\r (ID=0x601, DLC=4)

ACK                    \r
NACK                   \a  (0x07)
FRAME ACK              z\r
```

Full documentation: **https://ucandevices.github.io**
