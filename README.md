# SENTToUSB — STM32F042 USB ↔ SENT Bridge

USB CDC (Virtual COM Port) adapter for **SAE J2716 SENT** sensors.
Talks to the host using the **SLCAN** text protocol over any serial terminal.

---

## Hardware

| | |
|---|---|
| MCU | STM32F042G4UX (UFQFPN28, 48 MHz HSI48, no crystal) |
| **PA2** | SENT **RX** — TIM2 CH3 input capture, internal pull-up |
| **PA4** | SENT **TX** — GPIO push-pull driven by TIM14 |
| USB | Full-speed CDC (Virtual COM Port, no driver needed on Win10+) |

---

## How RX works

```
Sensor ──SENT signal──► PA2 (TIM2 CH3, RISING edge capture)
                              │
                    ISR: timestamp stored in RX HAL ring buffer
                              │
                    Main loop: bridge decodes batch of 10 edges
                              │
                    SLCAN 't' frame → USB → host
```

1. Host sends `O\r` — bridge enters RX mode.
2. Every RISING edge on PA2 triggers the TIM2 CH3 capture ISR, which records the raw 16-bit counter value.  A separate overflow ISR extends timestamps across the 16-bit rollover (~1.4 ms at 48 MHz).
3. `SentApp_Process()` (main loop) polls the RX HAL for complete batches of 10 edges: sync + status + 6 data nibbles + CRC + leading edge of next interval.
4. The bridge converts timestamps to µs intervals, extracts nibbles, validates CRC, and produces a CAN frame.
5. The frame is serialised as `t<ID><DLC><data>\r` and flushed to USB.

**Default config (MLX90377):** 3 µs/tick, 6 nibbles, DATA\_ONLY CRC, seed 0x03, output CAN ID 0x510.

---

## How TX works

```
Host ──SLCAN 't' frame──► USB CDC RX callback (ISR context)
                              │
                    bridge → TX HAL queue
                    tim14_kick(): start TIM14 if idle
                              │
              TIM14 ISR fires once per half-interval (3 µs/tick):
                Phase 0: pop next interval → PA4 LOW for 15 µs
                Phase 1: PA4 HIGH for remaining ticks
                Repeat until queue empty → PA4 idle HIGH, TIM14 stops
```

- TIM14 is reprogrammed on each TX kick: PSC = 0, ARR = (48 MHz × tick_us) − 1.  Default **1 tick = 3 µs**; the host can change the TX tick period at runtime (see SLCAN `SET_TX_TICK` below).
- Each SENT interval = `low_ticks` active-LOW pulse (5 ticks, SAE J2716 minimum) + HIGH for `(N − 5)` ticks.
- The bridge builds the full interval sequence (sync 56T + status + nibbles + CRC + 12T pause) and pushes it to the TX HAL.  The ISR drains it without any main-loop involvement.

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
| `04` | 1 | Learn tick period / nibble count / CRC mode from live signal |
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

**Decoded RX frame (device → host, default CAN ID 0x510):**
```
t5103AABBCC\r    DLC=3, 3 bytes = 6 nibbles packed high-nibble-first
```

---

## Python scripts

| Script | Purpose |
|--------|---------|
| `sent_MLX - Copy.py` | **MLX90377 GUI** — live angle gauge, decoded nibbles, frame log. Connect sensor to PA2, open port in the GUI; data appears automatically. |
| `sent_viewer.py` | General SLCAN monitor GUI with TX panel and Learn mode |
| `sent_test.py` | CLI — open port, apply config, print received frames for N seconds |
| `check_rx.py` | Loopback sanity check: pings COM8 (TX), listens on COM9 (RX) |
| `tx_signal.py` | Continuous TX every 10 ms for logic-analyzer capture on PA4 |
| `verify_sent.py` | Logic2 automation: capture PA4 signal and cross-check with USB output |

### Requirements
```
pip install pyserial
```
`verify_sent.py` additionally needs `pip install saleae` and Logic2 running with the automation API enabled (port 10430).

### MLX90377 quick start
```
python "sent_MLX - Copy.py"
```
Select the COM port, click Connect.  Firmware auto-starts RX on `O\r`; angle and magnitude update live.

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
