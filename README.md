# SENT (SAE J2716) USB Converter — SENTToUSB

[![Buy on Tindie](https://img.shields.io/badge/Tindie-Buy_SENT_USB_Converter-fb5b1b?logo=tindie&logoColor=white)](https://www.tindie.com/products/lll7/sent-j2716-usb-converter/)
[![Buy on Lectronz](https://img.shields.io/badge/Lectronz-Buy_in_EU-5468ff)](https://lectronz.com/stores/ucandevices)
[![Product page](https://img.shields.io/badge/docs-ucandevices.github.io-1f6feb)](https://ucandevices.github.io/sentusb.html)

> **Open source USB dongle that decodes SAE J2716 / ISO 21097 SENT automotive sensors. Auto-learns tick period, nibble count and CRC mode. Plug-and-play virtual COM port on Windows, Linux and macOS — no proprietary drivers.**

<p align="center">
  <img src="https://ucandevices.github.io/img/products/sent.png" alt="SENT (SAE J2716) USB Converter - dongle photo" width="60%">
</p>

USB CDC (Virtual COM Port) adapter for **SAE J2716 / ISO 21097 SENT** sensors. Decodes frames with 1 to 8 data nibbles at any tick time from 2 to 113 µs, with or without pause pulse, with both `DATA_ONLY` and `STATUS_AND_DATA` CRC modes, supports the SAE J2716 recommended CRC (`0x03`), the J2716 legacy CRC (FEB2008) and the `0x05` variant used by some GM / VW sensors, decodes Short and Enhanced serial messages, and includes a TX mode for synthesizing SENT frames to bench-test ECUs. Talks to the host using the **SLCAN** text protocol over any serial terminal. SENT protocol implementation: [open-sent-c](https://github.com/ucandevices/open-sent-c).

A commercial SENT analyzer from an automotive tool vendor typically costs hundreds to thousands of euros. This one is around **$23** on [Tindie](https://www.tindie.com/products/lll7/sent-j2716-usb-converter/).

## Where to buy

| Where | Region | Link |
|---|---|---|
| **Tindie** | Worldwide | https://www.tindie.com/products/lll7/sent-j2716-usb-converter/ |
| **Lectronz** | EU | https://lectronz.com/stores/ucandevices |
| **Elty** | Poland / EU | https://elty.pl/ |
| **Kamami** | Poland / EU | https://kamami.pl/ |

---

## Hardware

| | |
|---|---|
| MCU | STM32F042G4UX (UFQFPN28, 48 MHz HSI48, no crystal) |
| **PA2** | SENT **RX** — TIM2 CH3 input capture, internal pull-up |
| **PB0** | SENT **TX** — TIM3 CH3 output compare (toggle, AF1), DMA-driven |
| USB | Full-speed CDC (Virtual COM Port, no driver needed on Win10+) |

---

## How RX works

```
Sensor ──SENT signal──► PA2 (TIM2 CH3, FALLING edge capture)  [RX]
                              │
                    ISR: raw 32-bit timer value stored in the edge ring
                              │
                    Main loop: bridge aligns to frames and decodes them
                              │
                    SLCAN 't' frame → USB → host
```

1. Host sends `O\r` — bridge enters RX mode.
2. Every FALLING edge on PA2 triggers the TIM2 CH3 capture ISR, which stores the raw counter value. SAE J2716 defines every pulse period from falling edge to falling edge. TIM2 free-runs over its full 32 bits at 48 MHz, so there is no overflow bookkeeping and the timing resolution is 20.8 ns.
3. `SentApp_Process()` (main loop) drains the edge ring into the bridge. The bridge keeps the most recent edges and, as soon as it holds one frame's worth (sync + status + data nibbles + CRC), decodes that window:
   - success → the frame is forwarded, and its last edge becomes the first edge of what follows;
   - failure → the oldest edge is dropped and the search continues. This is how a pause pulse is stepped over and how alignment is recovered after a disturbance.

   Frames are found by their structure (a sync pulse followed by the right number of valid nibbles and a matching CRC), not by a time threshold, so no tick-time setting is needed to receive.
4. Checks per SAE J2716 5.3.3: sync pulse inside the configured tick range, every nibble 12–27 ticks, CRC, and successive calibration pulses within 1/64 of each other.
5. The frame is serialised as `t<ID><DLC><data>\r` and flushed to USB.

**Default config:** 6 data nibbles, DATA\_ONLY CRC, recommended CRC (`0x03`), tick range 2.0–113 µs, output CAN ID 0x510.

---

## How TX works

```
Host ──SLCAN 't' frame──► USB CDC RX callback (ISR) → byte ring → main loop
                              │
                    bridge → TX HAL (pre-expands frame into toggle array)
                    tim3_dma_start(): arm DMA and start TIM3 if idle
                              │
              TIM3 CH3 OC toggle (AF1 on PB0) fires at CCR3=1 each period:
                First toggle → pin LOW (SENT active-LOW pulse start)
                Subsequent toggles alternate LOW/HIGH per interval
              DMA1_Ch3 reloads TIM3→ARR from pre-computed buffer on each update
              Final DMA TC → TIM3 update ISR → stop timer, pin forced HIGH (idle)
```

- TIM3 runs from 48 MHz. ARR values are pre-computed per interval and stored in a DMA buffer. DMA1_Channel3 reloads TIM3→ARR from that buffer on every update event, so the ISR is only needed for the single end-of-frame cleanup.
- The timer prescaler is chosen per frame so that the longest interval fits the 16-bit ARR. This is what makes tick times above 24 µs and pause pulses up to 768 ticks possible.
- Default **1 tick = 3 µs**; the host can change the TX tick period at runtime (see SLCAN `SET_TX_TICK` below).
- Each SENT interval starts with an active-LOW pulse of nominally 5 ticks. PB0 is driven for about 10 µs less than that (never below 8 µs), because the output transistor switches off about 11 µs late through its 10 kΩ gate resistor. Without this the bus stays LOW for 26–29 µs and falling edges shift by up to half a tick at 3 µs.
- Frames are sent one per host command, with the line idle in between. Tick times below 3 µs are outside what the output stage can drive cleanly.
- The bridge builds the full interval sequence (sync 56T + status + nibbles + CRC + 12T pause) and the DMA drains it without any per-interval ISR involvement.

Switch to TX mode and send one frame:
```
O\r                       open SLCAN channel
t600102\r                 start TX mode
t52050100123456\r         transmit: status=1, nibbles [1,2,3,4,5,6]
```

---

## SLCAN quick reference

| Command | Effect |
|---------|--------|
| `O\r` | Open — start SENT RX |
| `C\r` | Close — stop |
| `V\r` / `v\r` | Hardware / firmware version |
| `N\r` | Serial number (unique per MCU, XOR-fold of 96-bit UID) |
| `F\r` | Status flags |
| `t<ID><DLC><data>\r` | Send CAN frame |

**Control frames (CAN ID 0x600):**

| data[0] | DLC | Action |
|---------|-----|--------|
| `01` | 1 | Start RX |
| `02` | 1 | Start TX |
| `03` | 1 | Stop |
| `04` | 1 | Learn tick period, nibble count (1–8) and CRC variant from the live signal; answers once with a `0x601` frame |
| `05` | 3 | Set TX tick period: `data[1..2]` = `tick_x10_us` (little-endian, 0.1 µs units; valid range 20–900, i.e. 2.0–90.0 µs) |

Example — set TX tick to 5.0 µs (`tick_x10_us` = 50 = `0x0032`):
```
t60030532 00\r  →  t6003053200\r
```

**TX data frame (CAN ID 0x520):**
```
t52050100123456\r
      ↑↑ ↑
      ││ └─ status byte then data nibbles packed (status=0x01, nibbles=0x00,0x12,0x34,0x56)
      │└─── DLC = 5
      └──── CAN ID 0x520
```

**Configuration frame (host → device, CAN ID 0x001):** each byte is optional (shorter DLC leaves the rest unchanged).
```
byte 0     data nibbles, 1..8
byte 1     CRC mode: 0 = data nibbles only, 1 = status nibble + data
byte 2     CRC selector: 0x03 = SAE J2716 recommended (default)
                         0x05 = variant used by some GM / VW sensors
                         0x85 = SAE J2716 legacy CRC (FEB2008 and older)
byte 3     minimum tick, 0.5 µs units
byte 4     maximum tick, 1 µs units
byte 5..6  output CAN ID, big-endian
```

**Learn result (device → host, CAN ID 0x601, DLC 5):**
```
byte 0..1  tick, 0.1 µs units, little-endian
byte 2     data nibbles
byte 3     CRC mode
byte 4     CRC selector (same values as config byte 2)
```

**Decoded RX frame (device → host, default CAN ID 0x510):**
```
t5104AABBCC10\r    6 nibbles AA BB CC, status nibble 1
```
```
byte 0..B-1  data nibbles packed high-nibble-first, B = ceil(nibbles / 2)
byte I       info byte, I = max(3, B):
               bits 7..4  status and communication nibble
               bit 0      a slow-channel message completed with this frame
               bit 1      it is an Enhanced serial message (else Short)
               bit 2      Enhanced with 16-bit data / 4-bit ID
byte I+1     slow message ID        (only when bit 0 is set)
byte I+2..3  slow data, little-endian
```
DLC is I+1, or I+4 when a slow-channel message completed. For up to 6 data
nibbles the info byte is byte 3, so DLC is 4 or 7 and the slow-channel bytes
are where earlier firmware put them (which sent DLC 3 without the info byte).

The slow decoder supports SAE J2716 Short Serial Message and Enhanced Serial
Message (12-bit data / 8-bit ID and 16-bit data / 4-bit ID). Slow-channel CRC
is validated inside open-sent-c; only validated messages are forwarded. Short
messages are checked with the same CRC variant as the frame.

---

## Python scripts

| Script | Purpose |
|--------|---------|
| `sent_viewer.py` | General SLCAN monitor GUI with TX panel and Learn mode |
| `mlx_viewer.py` | MLX90377-specific GUI: angle dial + live history plot of angle & magnetic field |
| `sent_test.py` | CLI — open port, apply config, print received frames for N seconds |
| `check_rx.py` | Loopback sanity check: pings COM8 (TX), listens on COM9 (RX) |
| `tx_signal.py` | Continuous TX every 10 ms for logic-analyzer capture on PA4 |
| `verify_sent.py` | Logic2 automation: capture PA4 signal and cross-check with USB output |

### Requirements
```
pip install pyserial
```
`verify_sent.py` additionally needs `pip install saleae` and Logic2 running with the automation API enabled (port 10430).

---

## Integration Testing

A pytest suite (`test_sent_integration.py`) drives **two SENTToUSB
dongles** against each other to verify the full TX → wire → RX loop.

**Hardware setup**
- TX dongle on `COM15`, RX dongle on `COM20` (override at the top of
  `TestSENTIntegration` in `test_sent_integration.py`).
- Wire **PB0 of the TX dongle → PA2 of the RX dongle** plus a common
  ground.  Without that physical link the data-path tests will report
  *"No 0x510 frames received"*.

**Coverage** (18 cases, ~60–90 s)
- Connectivity: `V`/`N`/`F` queries, RX & TX mode entry
- TX → RX frames at 4 tick periods (3, 6, 9, 12 µs)
- 10-frame back-to-back sequence
- Quiet-channel listen (no TX driver)
- Per-tick persistence across multiple frames

Each tick test first sends a `0x001` config frame to the RX device to
narrow its tick range around the tick under test. This is optional with
current firmware, which receives any tick time on its default range.

**Run**
```bash
pip install -r test_requirements.txt   # pytest>=7, pyserial>=3.5
python diagnostic.py                   # sanity-check both ports
python -m pytest test_sent_integration.py -v -s
```

`run_tests.py` provides shortcuts (`all`, `quick`, `framesize`,
`combined`, `debug`, single tick periods).

See **[TEST_INTEGRATION_README.md](TEST_INTEGRATION_README.md)** for the
full command reference, per-test descriptions, and troubleshooting
guide.

### Files

| File | Purpose |
|------|---------|
| `test_sent_integration.py` | Main pytest suite (18 cases) |
| `test_requirements.txt` | Python dependencies |
| `TEST_INTEGRATION_README.md` | Full test documentation |
| `run_tests.py` | Cross-platform pytest runner with shortcuts |
| `diagnostic.py` | Standalone V/N/O/TX-mode probe for both ports |

---

## Build

STM32CubeIDE manages the Makefile.  To build from the command line, set the paths to the bundled tools and run make inside the `Debug` folder:

```bat
set MAKE=C:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin\make.exe
set GCC=C:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin
set PATH=%GCC%;%PATH%
cd Debug
%MAKE% -j4 all
```

Output: `Debug/SENTToUSB.elf` (~28 KB flash, ~6 KB RAM on STM32F042G4 with 32 KB flash / 6 KB SRAM).

---

## Flash

### SWD (ST-Link)
```bat
set CLI=C:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.200.202503041107\tools\bin\STM32_Programmer_CLI.exe
%CLI% -c port=SWD freq=4000 -w Debug/SENTToUSB.elf -v -hardRst
```

### USB DFU (no ST-Link required)

**Step 1** — trigger DFU from the running device (replace `COM8` with your port):
```
python -c "import serial,time; s=serial.Serial('COM8',115200,timeout=1); s.write(b'boot\r'); time.sleep(0.3); s.close()"
```

**Step 2** — flash (~3 s after step 1, once the device enumerates as DFU):
```bat
%CLI% -c port=USB1 -w Debug/SENTToUSB.elf -v -hardRst
```

The `boot` command writes a magic value (`0xDEADBEEF`) to a `.noinit` RAM variable and resets.
On the next boot the firmware detects the magic, clears it, and jumps to the STM32F042 ROM bootloader at `0x1FFFC400`.

---

## FAQ

**What is SENT (SAE J2716)?**
SENT (Single Edge Nibble Transmission) is the SAE J2716 / ISO 21097 single-wire digital protocol used by many modern automotive sensors — pressure, position, throttle, MAP, angle — to transmit data to an ECU. Each frame is a stream of 4-bit nibbles separated by falling edges, with a status nibble, data nibbles and a CRC.

**What is the cheapest USB SENT analyzer?**
The uCanDevices SENT USB Converter is an open source SAE J2716 / ISO 21097 dongle around $23. It enumerates as a USB CDC virtual COM port on Windows 10+, Linux and macOS with no proprietary driver, and decodes SENT sensors with 1 to 8 data nibbles with both `DATA_ONLY` and `STATUS_AND_DATA` CRC modes.

**Can it auto-detect SENT tick period and CRC mode from an unknown sensor?**
Yes. Send `t600104\r` to enter Learn mode. The firmware auto-detects tick period (in 0.1 µs units, 2–113 µs), nibble count (1 to 8), CRC mode and CRC variant, then replies once with a `0x601` result frame. A sensor that sends constant data together with a pause pulse of 12–27 ticks cannot be told apart from one with an extra data nibble; configure that one manually. Send `O\r` afterwards to resume RX with the learned configuration.

**Can it transmit SENT frames to simulate a sensor?**
Yes. Start TX mode with `t600102\r`, then send frame data as `0x520` CAN frames (B0 = status nibble, B1–B4 = data nibbles packed big-endian). Optional `0x600 / 05` control frame sets the TX tick period at runtime (2.0–90.0 µs). TX is driven from `PB0` via TIM3 CH3 + DMA, so there is no per-interval ISR jitter.

**Which sensors are known to work?**
The defaults match the MLX90377 angle sensor (3 µs tick, 6 nibbles, DATA_ONLY, seed `0x03`). Other verified presets include the VW/Audi 04L 906 051 L DPF sensor (~3 µs, 6 nibbles, STATUS_AND_DATA, seed `0x05`) and the GM 12643955 MAP sensor (~3 µs, 6 nibbles, DATA_ONLY, seed `0x05`). For other sensors, use Learn mode.

**What if my sensor uses a non-standard CRC seed?**
Three CRC variants are supported, selected in B2 of the `0x001` configuration frame: `0x03` is the SAE J2716 recommended CRC (JAN2010 and later), `0x85` is the J2716 legacy CRC of FEB2008 and older, and `0x05` is a non-standard start value found on some GM and VW sensors. Learn mode tries all three.

**Is the firmware open source?**
Yes — STM32F042G4 firmware in this repository, SENT protocol layer in [open-sent-c](https://github.com/ucandevices/open-sent-c). A USB DFU bootloader is included; `sent_viewer.py` is a free GUI client.

---

## Related projects

- SENT protocol layer: [open-sent-c](https://github.com/ucandevices/open-sent-c)
- Sibling products: [USB LIN Converter](https://ucandevices.github.io/ulc.html), [USB CAN Converter (UCCB)](https://ucandevices.github.io/uccb.html), [CAN FD USB Converter (CFUC)](https://ucandevices.github.io/cfuc.html)
- Product page: https://ucandevices.github.io/sentusb.html
- All uCanDevices products: https://ucandevices.github.io/
- Contact: devices.ucan@gmail.com
