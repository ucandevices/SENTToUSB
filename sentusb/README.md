# sentusb: Python library, CLI and MCP server

Host software for the ucandevices SENT-to-USB converters. Every tool works with both boards:

| Board | MCU | Firmware | Channels |
|---|---|---|---|
| SENTToUSB | STM32F042 | V0102 (v1.1) and V02xx | 1 |
| SENT_4chanel (rev 2f / rev 3) | STM32C071 | V0200 | 4, plus termination switching and supply measurement |

| Part | What it is for |
|---|---|
| `sentusb/` (this folder) | Python library. Everything else is built on it. |
| `sentusb` CLI | Scripts, quick checks, production test. |
| MCP server | AI agents (Claude Code, Claude Desktop). |
| `../sent_viewer.py` | Desktop GUI (does not use the library yet). |

Channels are numbered **1..4 as printed on the board** (SENT1..SENT4) in all of them.

The board is detected when a tool connects: the 4-channel firmware answers command 0x06 with its channel
count and features, and v1.x firmware rejects it and is run as one channel with the v1.1 frame IDs.

**One port, one program:** Windows lets only one program open the COM port, so close the viewer before
using the CLI or the MCP server, and the other way round.

## Install

```
pip install "sentusb[mcp] @ git+https://github.com/ucandevices/SENTToUSB"   # from GitHub
pip install -e ".[mcp]"                                                     # from a clone (repo root)
```

Both put `sentusb` and `sentusb-mcp` on PATH; drop `[mcp]` if you only need the library and CLI. Without
installing, run the CLI from the repo root as `python -m sentusb ...` (needs `pip install pyserial`, plus
`pip install "mcp>=2.2,<3"` for the server).

**Emulator:** every tool accepts the port `emu://4ch` (or `emu://1ch`, or `emu://4ch?loop=12,34` for
wired channel pairs, or `emu://1ch?wire=A` twice for two boards wired together) and then runs against a
software model of the converter. Use it to try things without
hardware. It produces decoded frames, not electrical signals.

## Library

```python
from sentusb import SentUSB, SentFormat

with SentUSB() as dev:                          # first converter; or SentUSB("COM7")
    print(dev.info.board, dev.channel_count)
    fmt = dev.learn(1).to_format()               # find CH1's format; applied and RX restarted
    frame = dev.read(1, timeout=1.0)             # RxFrame: status, nibbles, slow message
    res = dev.capture(2.0, decoder="mlx90377", csv_path="ch1.csv")
    print(res.summary())                         # fps, error counters, value ranges, hints
    dev.configure(4, SentFormat(3.0, 6, "data_only", 0x03))
    job = dev.transmit(4, ["000000", "800000", "FFF000"], period_ms=5, duration_s=10)   # emulate a sensor
```

- **Connect:** `SentUSB(port, listen_only=False, formats=...)` configures every channel when it connects.
  The converter cannot report its current format, and the library needs the nibble count to decode.
  `listen_only=True` keeps the termination off, for monitoring a live ECU–sensor link.
- **Receive:** `learn`, `read`, `latest`, `history`, `stream()` (a queue of frames), `subscribe(callback)`,
  `capture(...)`.
- **Transmit:** `send(ch, '123456')` sends one frame. `transmit(...)` sends in the background and returns a
  `TxJob` with sent/rejected counts.
  - The firmware has no TX queue: a frame that arrives while the previous one is still being sent is rejected.
    The library therefore paces frames itself, by default at the frame length + 0.5 ms. It never sends a
    frame sooner than one frame length after the previous one was acknowledged, even after a late
    wake-up, and retries a rejected frame once.
  - `transmit` refuses a channel on which a sensor is sending (frames or sync/CRC errors seen) unless
    `force=True`.
- **Settings and status:** `set_termination(ch, 'auto'|'on'|'off')`, `set_listen_only()`, `supply()`,
  `refresh_info()`, `channel_state(ch)`, `counters(ch)`.
- **Decoders** (`sentusb.decoders`): `raw`, `h1_dual12`, `h4_secure`, `h5_single12`, `mlx90377`.
  - They follow SAE J2716 APR2016 appendix H. In H.1, fast channel 2 is sent least-significant nibble first.
  - `mlx90377.angle_deg` assumes the 12-bit value spans 0–360°.
  - The two pressure-sensor presets give raw 12-bit values, because no kPa curve is documented here.
- **Self-test:** `run_selftest(dev)`. Wire SENT1–SENT2 and SENT3–SENT4, with no sensors connected.
  - Board checks: firmware, supply 4.4–5.5 V on USB, termination on/off/auto read back per channel.
  - Loopback: each pair in both directions over 6 formats (3–50 µs, 3–8 nibbles, both CRC modes, seed 5,
    legacy CRC). Every frame must arrive unchanged, with no CRC or sync errors.
  - **Two boards** (for example two 1-channel SENTToUSB on their current V0102 firmware):
    `run_selftest(a, peer=b)`. Wire the SENT line of one board to the other, plus GND. On the 1-channel
    board that means TX (PB0) of each board to RX (PA2) of the other, as for `test_sent_integration.py`.
    Both boards get the board checks, and the loopback runs A→B and B→A over the same 6 formats.
    `peer_channels=(1, 3)` picks other channels, for example a 1-channel board against CH3 of a 4-channel board.

## CLI

```
sentusb ports                                    # converters first
sentusb info                                     # board, channels, termination, supply
sentusb learn 1                                  # prints the format and the flags to reuse
sentusb capture -c 1 2 -s 2 --preset MLX90377
sentusb capture -c 2 --learn --decoder h1_dual12 --csv ch2.csv
sentusb monitor -c 1 --decoder mlx90377          # live, Ctrl+C stops
sentusb tx 4 000000 800000 FFF000 --seconds 10   # values are cycled
sentusb selftest                                 # full; --quick, --no-loopback
sentusb -p COM8 selftest --peer COM9             # two boards, SENT lines wired (--channel, --peer-channel)
sentusb -p COM8 selftest --peer COM9 --direction b-to-a   # one wire: COM9 TX -> COM8 RX
sentusb --json ...                               # machine-readable output
sentusb --listen-only ...                        # termination off
sentusb --port emu://4ch ...                     # emulator
```

Each command opens and closes the port and configures the channels again, so give each command the
format it needs: `--learn`, `--preset`, or `--tick/--nibbles/--crc/--seed/--legacy-crc`. Exit codes:
0 ok, 1 error, 2 self-test failed.

## MCP server

Register it once in Claude Code:

```
claude mcp add sentusb -- sentusb-mcp                                  # after pip install
claude mcp add sentusb -- python C:\path\to\SENTToUSB\sentusb_mcp.py     # from a clone, no install
claude mcp add sentusb -e SENTUSB_PORT=COM7 -- sentusb-mcp             # fixed port
claude mcp add sentusb-emu -e SENTUSB_PORT=emu://4ch -- sentusb-mcp    # emulator
```

The server keeps one connection open between calls, so a learned format, termination settings and background
transmissions persist. It has 18 tools:

| Group | Tools |
|---|---|
| Connection | `list_ports`, `connect`, `disconnect`, `device_status`, `supply_status` |
| Configuration | `configure_channel` (format or preset), `learn_format`, `set_termination`, `set_listen_only`, `start_rx`, `stop_channel` |
| Receive | `capture` (statistics and hints, optional CSV), `read_frames`, `list_decoders` |
| Transmit | `start_transmit`, `stop_transmit`, `transmit_status` |
| Test | `run_self_test` |

Safety choices:
- `connect` defaults to `listen_only=True`, which is safe on a live ECU–sensor link. A sensor alone on the
  bench needs `listen_only=false`; the capture hints say so when no edges arrive.
- `start_transmit` refuses a channel on which a sensor is sending unless `force=true`. The server's
  instructions tell the agent to use `force` only with the user's confirmation.
- There is no tool for the `boot` (DFU) command. Flashing stays with `flash_usb.bat`.
- `supply_status` warns when an external supply above 5.5 V reaches every VS pin.

Example request to the agent: "connect to the converter, learn CH1, capture 2 s with the mlx90377 decoder and
tell me whether the angle is stable and whether any frames were lost".

## Viewer

```
python sent_viewer.py                            # from the repo root
python build_viewer.py                           # dist/sent_viewer.exe
```

- **Connecting:** it detects 1 or 4 channels. On the 4-channel board, *Listen only* keeps the termination off;
  set it before connecting.
- **Channel controls:** *Channel* buttons (or a click in the *Channels* table) choose what the parameter,
  learn and TX controls act on. Each channel keeps its own settings. *Apply to All* copies the selected
  channel's settings (not its RX CAN ID).
- **Termination:** *Auto* / *On* / *Off*. The *Channels* table shows the actual state, read back every second.
- **Learn:** one channel at a time. After a result the viewer sends the learned format and restarts RX.
- **TX:** *Send* sends one frame, *Start* sends periodically. Several channels can transmit at once.
- **Supply bar:** VS, current and source. It turns orange above 5.5 V and red on a USB supply fault.
- **Frames:** *Show* filters the frame table by channel. *Save Log* and *Record* write CSV with a `channel`
  column.

## Tests

```
python -m pytest            # from the repo root: tests/, 35 tests, about 45 s, emulator only
```

The tests check:
- **Protocol:** bytes compared with the firmware sources and with the v1.1 viewer.
- **Decoders:** checked against J2716 appendix H.
- **Library on the emulator (4-channel and v1.1):**
  - detection; listen-only; learn, including the "another channel is learning" refusal;
  - capture statistics, hints and CSV;
  - the TX guard, pacing, and the firmware rejecting a frame while the line is busy;
  - link resync after a lost answer;
  - the self-test, passing on wired pairs and failing on an unwired pair;
  - the two-board self-test: two 1-channel boards on all 6 formats, a 1-channel board against a
    4-channel board, and two boards that are not wired.
- **CLI:** commands and exit codes.
- **MCP:** in-memory client, and a real stdio session.

**Not tested on hardware yet.** First hardware test: two 1-channel SENTToUSB boards wired together,
`sentusb -p COMa selftest --peer COMb`. On the first rev 3 board, run `sentusb selftest` with the loopback
wiring.

## Version history

| Version | Change |
|---|---|
| viewer 1.1 | 1 channel (SENTToUSB). |
| viewer 2.0 | 1 or 4 channels detected from the device; per-channel parameters, learn and TX; *Channels* overview; termination and listen-only; supply bar; channel filter and `channel` column in CSV; COM port pre-selected by USB ID; reports an unplugged device. |
| sentusb 0.1.0 | Library, CLI, MCP server, emulator, self-test. |
| sentusb 0.1.0 (unreleased) | Two-board self-test (`--peer`, `peer_port`) and emulator wire (`?wire=`). TX pacing: no early frame after a late wake-up, one retry of a rejected frame (frames of 12.5 µs × 8 nibbles and 50 µs were rejected before). Viewer: TX period and the one-shot RX switch-back are at least one frame long. Serial answers no longer wait 50 ms and short TX waits no longer sleep 8–15 ms on Windows (TX rate up from ~16 to ~710 frames/s at 3 µs). Selftest `--direction`. |
