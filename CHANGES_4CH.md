# 4-channel firmware: change log

Starting point: plain copy of SENTToUSB v1.1 (`V0102`, STM32F042, 1 channel).
Spec: `../4channel_varinat.md`, sections 2.4–2.5, 4 and 5.
Nothing below has run on hardware. Verification so far is builds plus host tests.

References to `../4channel_varinat.md`, `../concept_review.md`, `board_summary` and `pcb/` point to the
hardware design notes and the KiCad project of the 4-channel board. They are kept in a separate,
non-public repository; this repository holds all of the software.

## Build and test commands

| What | Command | Result (2026-11-17) |
|---|---|---|
| F042, 1 channel | `./build.sh` | 26 012 text + 380 data = **26 392 B flash** (32 KB), 380 + 5 144 bss = **5 524 B RAM** (6 KB) |
| C071, 4 channels | `./build_c071.sh` | 23 576 text + 380 data = **23 956 B flash** (128 KB), **8 968 B RAM** (24 KB; includes the 1 KB minimum stack reserve) |
| Library tests | `make run` in `Core/Inc/sent/implementation/Tests` | 83/83 (78 from v1.1 + 5 new) |
| Application tests | `make run` in `Core/Tests` | 17/17 with `SENT_NUM_CHANNELS=4`, 17/17 with `=1` |

The host tests need gcc, so run them in WSL (`wsl -d Ubuntu -- bash -lc "cd …/Tests && make run"`).

## 1. Decoder speed-up (open-sent-c copy)

- `sent_decoder.c`: each candidate sync now costs **one division**, `UINT32_MAX / sync`. Each nibble and the
  pause are converted with a multiply by that reciprocal, then an exact remainder correction (at most two
  steps), so the results are the same as with plain division. The ±0.35-tick tolerance test is done with
  multiplication: `(diff-1)*100 >= sync*35`, which is the same test as `diff > ceil(sync*35/100)`. Before
  the reciprocal is computed, a shift/add-only check rejects any interval ≥ 28 or < 11.2 ticks, which the
  full check would also reject. On a noisy line most windows therefore cost no division at all.
  - v1.1 did about 10 software divisions per decode on a Cortex-M0 without a divide instruction.
- New `sent_decode_sync_in_range()`: a cheap sync-window test that uses the same expression as the decoder.
  The bridge calls it before the full decode, in normal RX and for every learn candidate.
- **Bit-identical check:** `Tests/ref_decoder_v11.c` is the v1.1 decoder, verbatim, with the function
  renamed. `Tests/test_sent_decoder_equiv.c` compares the return value, the status and every byte of the
  output frame:
  - 300 000 generated windows: all nibble counts, CRC variants and resolutions of 1–170 counts/µs; jitter
    right at the ±0.35 edge; nibbles of 10/11/28/29 ticks; pauses up to 790 ticks; garbage; wrap-around;
    zero and negative steps;
  - an exhaustive nibble sweep at 6 sync lengths;
  - a pause sweep across 0–800 ticks;
  - pre-check boundary tests.
  - No mismatches.

## 2. Multi-channel application layer (MCU-independent)

- **`Core/Src/sent_core.c` / `Core/Inc/sent_core.h` (new).** Everything that is not hardware moved out of
  `sent_app.c`:
  - USB RX ring, line assembly and SLCAN dispatch;
  - USB TX ring;
  - per-channel diagnostics;
  - the round-robin main loop.
  The platform supplies the RX/TX HAL tables plus four hooks: `now_ms`, `usb_transmit`, `rx_lost_edges`
  and `tx_kick`. This makes the whole protocol host-testable.
- **`Core/Inc/sent_app_config.h` (new).**
  - `SENT_NUM_CHANNELS`: 1 by default, 4 when `STM32C071xx` is defined, and overridable.
  - Per-build sizes:

    | Setting | 1 channel | 4 channels |
    |---|---|---|
    | USB TX ring | 384 B | 2 KB |
    | USB RX ring | 128 B | 512 B |
    | Edge budget per pass | 64 | 16 |

  - Version `"0200"`.
- **`sent_bridge.c/.h`**
  - `channel` field. The bridge owns its own IDs: config `0x001+n`, control `0x600` with `data[0] = n<<4 | cmd`,
    learn result `0x601+n`, default RX output `0x510+0x40n`, TX `0x520+0x40n`. Channel 0 uses exactly the v1.1 IDs.
  - New functions: `sent_bridge_set_channel()`, `sent_bridge_owns_frame()`, `sent_bridge_mode()` (OFF/RX/TX/LEARN)
    and `sent_bridge_is_learning()`.
  - **Shared learn state.** The about 120 B search state moved to `sent_bridge_learn_t`, one instance for
    all channels, with an `owner` pointer. A learn command on another channel is refused (NACK) until the
    current learn ends, which happens on result, stop, config frame, switch to TX or overrun reset.
  - Change from v1.1: **START_TX during a learn now ends the learn and restores the pre-learn config.** v1.1
    left `learn.active` set with the learn tick range through TX mode. With shared storage that would lock
    learning for every channel.
- **Host protocol.** For v1.1 tools, channel 0 behaves as before; the bullets marked as new are the exceptions.
  - `O`/`L` start RX on all channels and `C` stops all of them. With one channel this is exactly v1.1.
  - Control frames for a channel that does not exist are NACKed, as unknown commands were in v1.1. Unknown
    IDs, including other channels' config/TX IDs on the 1-channel build, are still accepted and ignored.
  - New: control command `0x06` (any channel nibble) replies `z` plus frame `0x60F`:
    - data[0]: channel count;
    - data[1..2]: version, BCD 02 00;
    - data[3]: 2-bit mode per channel (0 off, 1 RX, 2 TX, 3 learn);
    - data[4]: the channel that is learning, or 0xFF.
  - New: `V0200` / `v0200`.
  - Diagnostics are per channel at most every 100 ms: `0x511+0x40n` with the v1.1 layout. New:
    `0x512+0x40n` = lost-edge events LE32 + USB-buffer overflows LE32. It is sent only when one of these
    changes, so normal traffic is identical to v1.1.
  - USB-overflow counter: a frame that does not fit in the USB TX ring is still dropped whole, but it is now
    counted per channel.
- **Main-loop-only USB rule kept.** The USB interrupt only stores bytes (`sent_core_on_usb_rx`); parsing
  and replies happen in `sent_core_process()`.
- **Round robin.** Each pass serves the channels in turn, starting with a different channel each pass, and
  hands at most `SENT_EDGE_BUDGET` edges to each.
- **Found and fixed (affected v1.1 as well): USB TX ring reuse during a transfer.** v1.1 advanced the ring
  tail as soon as `CDC_Transmit_FS` accepted a block of up to 384 B. The USB driver copies only the first
  64-byte packet into packet memory at that point; it reads the rest from RAM later, and new lines could
  overwrite that RAM in the meantime. The core now hands over at most one 64-byte packet per call, which
  is copied immediately. The data stream is unchanged.
- **Lost-edge counter made monotonic.** `hal_stm32.c` no longer clears `dropped_edges` in `start_rx`. In
  v1.1, a restart after an overrun was reported as one more overrun.
- **Tests (`Core/Tests`, new).**
  - `test_bridge_multi.c`: 4 bridge instances on interleaved synthetic streams (3/5/12.5/90 µs, 6/4/3/8
    nibbles, 4 CRC variants, pauses 12/0/300/20 ticks, 32-bit wrap):
    - every frame is decoded on the right ID with the right payload;
    - a noisy channel does not disturb the others;
    - shared learn works one channel at a time;
    - each bridge owns only its own frame IDs.
  - `test_core.c` runs through `sent_core` with fake HALs:
    - version, serial and info;
    - channel addressing for config, control, TX tick and TX frames;
    - shared learn and the `0x601+n` result;
    - RX end-to-end on all channels with per-channel diagnostics;
    - configurable output ID;
    - the edge budget per pass;
    - lost-edge reporting;
    - USB overflow counting: no truncated lines, delivered + dropped = produced;
    - USB line assembly.

## 3. F042 build (1 channel)

- `sent_app.c` is now only the F042 platform layer. The TIM3/DMA TX code is the **v1.1 code unchanged**,
  including the interrupts-off sync start. It is hardware-proven, and with one channel the busy-wait is
  not a problem.
- `build.sh`:
  - adds `sent_core.c`;
  - follows the `hal_stm32` rename;
  - **compiles application code with `-Os`**, because at `-O0` the multi-channel core no longer fits in 32 KB.
- Expected differences from v1.1 on channel 0:
  - version `V0200`;
  - command `0x06` and frame `0x60F`;
  - `0x512` when edges or frames are lost;
  - START_TX ends a learn;
  - 64-byte USB hand-off;
  - no spurious overrun count after a restart.
- `Core/Inc/sent/sent_assert.h` now keys on `SENT_HAL_STM32`. The rename had briefly put libc `assert` and
  printf into the image.

## 4. STM32C071 target

- **Drivers.** `Drivers/CMSIS/Device/ST/STM32C0xx` and `Drivers/STM32C0xx_HAL_Driver`, from the public ST
  GitHub repos `cmsis-device-c0` and `stm32c0xx-hal-driver`, tag v1.4.1, with `.git` removed.
- **USB stack.** The classic ST USB Device library already in `Middlewares/` is reused. The C0 HAL PCD
  driver supports `USB_DRD_FS` with the same PCD API, so only `usbd_conf.c` changed:
  - instance and IRQ;
  - packet-memory layout (descriptor table 0x40, then EP0 OUT/IN, EP1 IN/OUT and EP2 IN);
  - CRS setup.
  This is the smallest path; USBX would be much bigger and would mean a new CDC glue layer.
- **New files (`C071/`)**

  | Path | Content |
  |---|---|
  | `STM32C071GBUX_FLASH.ld` | 128 K / 24 K |
  | `Startup/startup_stm32c071xx.s`, `Core/Src/system_stm32c0xx.c` | CMSIS templates |
  | `Core/Inc/stm32c0xx_hal_conf.h` | Template; only CORTEX/DMA/FLASH/GPIO/PCD/PWR/RCC enabled |
  | `Core/Inc/main.h` | |
  | `Core/Src/main_c071.c` | Clocks, GPIO, option bytes and DFU |
  | `Core/Src/stm32c0xx_it.c` | |
  | `Core/Src/sent_app_c071.c` | Timers, DMA, ISRs |
  | `USB_DEVICE/` | Copy of the F042 glue with C0 includes, `usbd_conf.c` changes and the `boot` command |

  `build_c071.sh` builds them into `Debug_C071/` (Cortex-M0+, `-Os`, `-DSTM32C071xx`, no warnings with
  `-Wall -Wextra`).
- **`hal_config.h`.** `STM32F042x6` or `STM32C071xx` selects `SENT_HAL_STM32`. `SENT_HAL_STM32F042` is still
  accepted.
- **HAL backend renamed.** `hal_stm32f042.c/.h` became `hal_stm32.c/.h` (`sent_stm32_*`), with the same
  logic. Two other changes:
  - the capture push is `static inline` for the register ISR;
  - new `sent_stm32_rx_note_lost_edges_isr()`.
- **Clocks.**
  - SYSCLK is HSIUSB48 at 48 MHz. This is the oscillator the CRS trims to USB SOF, the same arrangement as
    HSI48 on the F042, so the TX tick follows USB.
  - The USB clock is HSIUSB48.
  - CRS: sync source USB SOF, reload 48 000 − 1.
  - **Note: the v1.1 F042 code never enables the CRS.** There is no CRS code anywhere, so its HSI48 runs
    untrimmed. This is worth checking on the 1-channel product.
- **RX.** TIM2 CH1–CH4 on PA0/PA1/PA2/PA3 (AF3/AF3/AF6/AF3 as in the plan).
  - Setup: 32-bit free running, falling edge, `ICF=3`, internal pull-up (as v1.1).
  - `TIM2_IRQHandler` works at register level: it reads SR once, stores each CCRx in its channel's 64-entry
    ring and counts each CCxOF as a lost edge. A lost edge resets alignment and the slow channel for that
    channel.
  - Priority 0.
- **TX.** A per-channel table generalises `tim3_dma_start()`:

  | Channel | Timer | Pin | DMA1 channel | DMAMUX request |
  |---|---|---|---|---|
  | 0 | TIM3_CH3 | PB0 AF1 | 1 | 37 |
  | 1 | TIM1_CH1 | PA8 AF2 | 2 | 25 |
  | 2 | TIM16_CH1 | PA6 AF5 | 3 | 46 |
  | 3 | TIM17_CH1 | PA7 AF5 | 4 | 49 |

  - DMA and timer setup are register-level.
  - **MOE is set on TIM1 and also on TIM16/TIM17**, which have a BDTR too. The plan mentions only TIM1.
  - Kept from v1.1: the per-frame prescaler and the preloaded CCR = 0 end of frame.
  - **Frame start without busy-wait:** a dummy first period (about 2 µs) with CCR = 0. Its update event
    loads the frame CCR, asserts PWM1 (the start of the sync pulse) and DMA-loads the sync length into ARR,
    the same mechanism as every other interval.
  - The output-stage compensation `TX_LOW_STRETCH_US` / `TX_LOW_MIN_US` defaults to 0 for the C071 board.
    Measure it on the rev 2 hardware.
- **DFU.** Per RM0490, the C0 system memory is at 0x1FFF0000, and USB DFU is in the bootloader on the C071
  only. AN2606 itself was not checked here.
  - The `boot` command sets nBOOT0 = 0 (with nBOOT_SEL = 1, nBOOT1 = 1) and launches the option bytes,
    which resets the chip into the ROM DFU.
  - At start-up the application restores nBOOT_SEL = nBOOT1 = nBOOT0 = 1, so it boots from flash and
    ignores PA14-BOOT0 (SWCLK).
  - This is the same scheme as the F042, rather than a direct jump, which has reported USB enumeration
    problems on the C071.
  - "Disable the USB IRQ before `HAL_Init`" is kept.
  - The 1 ms wait after "Entering DFU" is a spin loop, not `HAL_Delay`. It runs in the USB interrupt, and
    SysTick has a lower priority there.
- **Other pins.** PB1 LED (output, off); PA5 VS_EN (output, driven high at start-up, *assumes an active-high
  load-switch enable*); PA4 VS_FLT (input, pull-up).

## Unverified and open

- **Hardware** (none tested):
  - C071 clock tree and CRS lock;
  - USB enumeration with the DRD packet-memory layout;
  - option-byte DFU entry and exit;
  - the TX dummy-period start: does OCxREF assert at the first update event as expected?
  - DMA timing on four timers at once;
  - TIM2 ISR CPU load at 4 × 3 µs;
  - the TX compensation constants;
  - VS_EN polarity.
- **Pin-level AF numbers.** They are taken from the plan (DS14693). The C0 HAL confirms AF1/2/3/5/6 carry
  TIM1/3/2/16/17 functions but not the pin mapping. Generate a CubeMX project to cross-check.
- **Not done:**
  - new CubeMX `.ioc` (the init code is hand-written);
  - continuous TX streaming;
  - host tools and docs (plan 4.6);
  - `.cproject` (CubeIDE) still lists the old file set.
- **RX capture IRQs.** A channel in TX mode still takes RX capture interrupts from its own line (the edges
  are discarded). Disabling CCxIE for TX channels would save CPU.

## 2026-11-17 follow-up

- RX pins PA0-PA3: internal pull-up removed (`GPIO_NOPULL`). The board has a 10 k pull-up to the 5 V sensor supply, and DS14693 forbids the internal pull-up on FT pins above VDD. Found by the concept review (`../concept_review.md`, finding 2). The C071 build is clean afterwards (23 960 B flash).

## Rev 2b (2026-11-17): software side of the concept-review fixes

Spec: `../4channel_varinat.md` section 2.6 (hardware/software split, rev 2b pin plan), findings
B1, M1–M4, M8, m2 in `../concept_review.md`. Nothing of this has run on hardware.

| What | Command | Result |
|---|---|---|
| F042, 1 channel | `./build.sh` | 26 452 text + 380 data = **26 832 B flash** (32 KB), 380 + 5 176 bss = **5 556 B RAM** (6 KB), 0 warnings |
| C071, 4 channels | `./build_c071.sh` | **26 156 B flash** (128 KB; includes the 1 008 B .data/.RamFunc load image), **9 696 B RAM** (24 KB; includes the 628 B TIM2 handler in SRAM and the 1 KB stack reserve), 0 warnings |
| Library tests | `make run` in `Core/Inc/sent/implementation/Tests` | 83/83 |
| Application tests | `make run` in `Core/Tests` | 21/21 with `SENT_NUM_CHANNELS=4`, 21/21 with `=1` (4 new tests) |

### 1. TX output (B1): push-pull buffer, inverted polarity, TX_EN

- Each output channel now has `CCxP = 1` (TIM3_CH3 PB0, TIM1_CH1 PA8, TIM16_CH1 PA6, TIM17_CH1 PA7;
  `CCxNP`/`CCxNE` stay 0). PWM active phase = LOW pin = SENT LOW pulse through the non-inverting
  SN74LV1T125. Forced-inactive (idle, between frames, after an abort) = HIGH pin.
- `TX_LOW_STRETCH_US` / `TX_LOW_MIN_US` stay 0 (the buffer is symmetric); the comment is updated.
- TX_EN1..4 = PB3..PB6, active low: driven low only while the channel is in TX mode.
- Leaving TX mode while a frame runs now stops it (`tx_abort()`: timer, DMA and update interrupt off,
  output forced-inactive, TX HAL released) before TX_EN goes high.
- The channel's TIM2 capture interrupt (CCxIE) is off while it transmits. It is now enabled only while
  the channel is in RX or learn mode, so it is also off when the channel is OFF. When it is enabled
  again, the stale CCRx/CCxOF are discarded first.
- Mechanism: new optional platform hook `line_control(ch, mode, termination)` in
  `sent_core_platform_t`. The core calls it:
  - once per channel at init (OFF, termination off);
  - on every change of mode or termination, in `dispatch_slcan_line()` **before** `tx_kick()`, so a
    frame never starts with the buffer disabled;
  - once per main-loop pass, because a learn can end on an edge.
- Order in `platform_line_control()`:
  - entering TX: capture IRQ off, termination, TX_EN low;
  - leaving TX: abort, TX_EN high, termination, capture IRQ on (RX/learn only).

### 2. Termination (M2): control command 0x07

- `0x600`, `data[0] = (n << 4) | 0x07`, `data[1]` = 0 off / 1 on / 2 auto. A DLC below 2, a value above 2
  or a channel that does not exist is NACKed (`\a`). Handled in `sent_core.c`, not in the bridge.
- AUTO (default): on in RX and learn, off in TX and when the channel is OFF. ON and OFF are forced in
  every mode.
- Power-up: all channels AUTO and OFF, so all termination is off. The board's pull-downs keep it off
  until the firmware runs.
- TERM_EN1..4 = PA15, PB7, PC14, PC15, active high.
- Info frame `0x60F` (still DLC 8; v0200 sent zeros in data[5..7], so old hosts are unaffected):
  - `data[5]` bits 3..0: termination currently on, channel n in bit n. Bits 7..4: setting is AUTO,
    channel n in bit 4+n. A clear AUTO bit means the state in bits 3..0 is forced.
  - `data[6]` board features: bit 0 switchable termination, bit 1 supply measurement (0x60E), bit 2
    push-pull TX buffer. C071 = 0x07, F042 = 0x00.
  - `data[7]` reserved, 0.
- F042: the platform has no `line_control` hook, so 0x07 is accepted and reported but has no effect.

### 3. Sensor supply and USB power (M3, m2)

- VS_EN (PA5) is **low at reset**. `CDC_Init_FS` (= SET_CONFIGURATION) switches it on.
  - It goes off on `CDC_DeInit_FS` (configuration cleared), USB bus reset, disconnect and suspend.
  - It comes back on at resume if the device is still configured.
  - Implemented in `SentApp_OnUsbConfigured()` / `SentApp_OnUsbSuspend()` in `sent_app_c071.c`,
    called from `usbd_cdc_if.c` and `usbd_conf.c`.
  - It still assumes an active-high EN.
- USB configuration descriptor: bus powered (`bmAttributes 0x80`), `bMaxPower 0xAF` (350 mA),
  `USBD_SELF_POWERED 0`.
  - The descriptor is in the shared `Middlewares/.../usbd_cdc.c`. Its two bytes now come from
    `USBD_CDC_CFG_BMATTRIBUTES` / `USBD_CDC_CFG_MAXPOWER`.
  - These default to the original 0xC0 / 0x32 and are overridden only in
    `C071/USB_DEVICE/Target/usbd_conf.h`, so the F042 descriptor is byte-identical to before.
- ADC (register level, no HAL ADC driver):
  - PB1 = ADC1 channel 18, plus VREFINT (channel 10);
  - clock PCLK/4 = 12 MHz, 160.5-cycle sampling, 16x hardware oversampling (shift 4, 12-bit result),
    WAIT mode;
  - one scan every 100 ms, started and collected in the main loop without blocking (about 0.5 ms of
    conversion);
  - VDDA = 3000 mV × VREFINT_CAL / VREFINT. If the calibration value is missing or the result is
    implausible, 3300 mV is used instead;
  - VS = raw × VDDA × 147 / (4095 × 47).
- New frame **`0x60E`**, every 500 ms while USB is configured. It is generated in `sent_core.c` through
  the optional `supply_status` hook, so it is host-tested.
  - DLC 3: `data[0..1]` VS in mV LE, `data[2]` bit 0 VS_EN on, bit 1 VS_FLT active (PA4 low).
  - Checked against every other ID, with no collision: config 0x001–0x004, RX/diag/TX 0x510–0x5E0,
    control 0x600, learn 0x601–0x604, info 0x60F.
  - A host can still configure a channel's RX output ID to 0x60E; this is not prevented.

### 4. Capture interrupt (M4)

- Edge-rate guard:
  - `SysTick` calls `SentApp_OnTick1ms()`, which clears the per-channel edge counters every 1 ms and
    counts down the hold time.
  - More than 100 edges in one 1 ms window on one channel turns that channel's CCxIE off for 10 ms,
    counts one guard trip and notes one lost edge (the main loop then resynchronises the channel).
  - Other channels are not affected.
- Loss frame `0x512 + 0x40n` (DLC 8):
  - `data[0..3]`: lost-edge events LE32 (a guard trip also counts as one);
  - **`data[4..5]`: USB-buffer overflows LE16**;
  - **`data[6..7]`: guard trips LE16**.
  - Both LE16 fields saturate at 0xFFFF. Before rev 2b `data[4..7]` was the overflow count LE32. No
    host tool reads this frame yet.
- SR handling:
  - Only flags of channels whose CCxIE is set are serviced.
  - CCxIF is cleared by the CCRx read and is never written. A write-0 clear after the read could wipe
    a capture that arrives in between, and that capture would be lost without any trace.
  - CCxOF is cleared with `TIM2->SR = ~of`: a write-0 mask of exactly the OF flags seen, after the CCRx
    reads, never read-modify-write.
- `TIM2_IRQHandler` is in `.RamFunc`. The C071 linker script already places `*(.RamFunc*)` in `.data`,
  which the startup code copies.
  - Everything it uses is force-inlined (the ring push is written out), so it makes no call into
    flash. The disassembly has no `bl`. Size 628 B.
  - `build_c071.sh` links with `--no-warn-rwx-segments`, because the RAM load segment is now
    executable on purpose.
- TIM2's capture interrupts start disabled (`DIER = 0`); `line_control` enables them per channel.
  `SentApp_Init()` now sets up the timers and the ADC before `sent_core_init()`.

### 5. Pins (rev 2b plan)

- `MX_GPIO_Init()` writes ODR first, then the mode:
  - TX_EN PB3–PB6 push-pull **high**;
  - TERM_EN PA15/PB7/PC14/PC15 push-pull **low**;
  - VS_EN low;
  - LED on **PB8** (was PB1), low;
  - VS_FLT PA4 input with pull-up;
  - PB1 analog (VS_SENSE), PA9/PA10 explicitly analog.
- PC6 is never touched and stays analog. The GPIOC clock is enabled for PC14/PC15.

### 6. Tests and the F042 build

- `Core/Tests/test_core.c`:
  - fakes for `line_control`, `rx_guard_trips` and `supply_status`;
  - the info test checks data[5..7];
  - new `test_line_state_follows_mode`: init call, RX → on, TX → off and set before the kick,
    learn → on, close → off;
  - new `test_termination_command`: forced on in OFF/TX, forced off in RX, back to AUTO, three NACK
    cases;
  - new `test_guard_trips_in_loss_frame`: layout and saturation;
  - new `test_supply_frame`: no frame when not configured, 500 ms period, layout.
- F042 (`sent_app.c`) sets none of the new hooks: no TX_EN/TERM_EN/ADC and no 0x60E. Command 0x07 is
  accepted without effect, and info data[6] = 0.
- Channel 0 traffic is otherwise unchanged, except that `0x512` (sent only on losses) now carries the
  overflow count as LE16 in data[4..5].

### Still unverified (needs the rev 2b board or a Nucleo)

- PB1 → ADC channel 18: taken from the plan/DS14693; the CubeC0 headers have no pin-to-channel table.
- The ADC sequence (calibration, CCRDY, WAIT + oversampling) and the VS scaling, against a meter.
- TX polarity on the scope: idle HIGH, LOW pulse length, no glitch when TX_EN switches, frame abort.
- VS_EN timing on a real host (enumeration, suspend/resume). A host that suspends an idle CDC port
  would switch the sensor off. VS_EN polarity is still an assumption.
- CPU load and latency of the RAM TIM2 handler with 4 × 3 µs channels; the guard threshold under real
  noise.
- Everything under "Unverified and open" above that this section does not supersede. The "RX capture
  IRQs" item there is now done.

## Rev 2d (2026-11-19): sensor current and supply source

Hardware in `../4channel_varinat.md` 2.9 (TPS2121 mux USB / EXT_IN, 0.1 Ω + INA180A2, VS divider 22/122).

- VS divider 100 k / 22 k: `VS_DIV_NUM/DEN` = 122/22 (VS up to ~17 V from EXT_IN).
- I_SENSE **PB2 = ADC1_IN19**, analog. The scan is now channels 10, 18, 19 (ascending): `g_adc_raw[]`
  = VREFINT, VS, I_SENSE. Same VDDA-from-VREFINT correction for both; scan about 0.7 ms, still
  non-blocking. I [mA] = V(PB2) [mV] / 5 (0.1 Ω × 50 V/V), 0–~0.66 A.
- VS_SRC **PB9** input, no pull (10 k pull-up on the board): TPS2121 ST, high = EXT_IN (or no source
  on), low = USB path.
- VS_EN (PA5) only switches the USB path; an external supply is always passed through. VS and current
  are reported as measured, never assumed 0 when VS_EN is off.
- `supply_status` hook now takes a `sent_core_supply_t*` (vs_mv, flags, is_ma, source). sent_core
  pre-fills is_ma = 0xFFFF and source = 0xFF, so a platform reports only what it has.
- Frame **`0x60E`**, DLC 3 → **6**; data[0..2] unchanged:
  - `data[0..1]` VS in mV LE;
  - `data[2]` bit 0 VS_EN on (USB path enabled), bit 1 VS_FLT active;
  - `data[3..4]` sensor-rail current in mA LE, 0xFFFF = not measured;
  - `data[5]` supply source: 0 USB, 1 EXT_IN (or no source on), 0xFF = not reported.
- F042: no `supply_status` hook, unchanged (no 0x60E).
- `test_supply_frame`: checks DLC 6, the "not measured" defaults and the rev 2d fields.
- Builds (2026-11-19, GNU Tools for STM32 13.3.rel1 from STM32CubeIDE 1.19.0): C071 26 248 B flash,
  9 696 B RAM; F042 26 476 text + 380 data; no warnings. Application tests 21/21 + 21/21 (WSL).
- PB1 = ADC1_IN18 and PB2 = ADC1_IN19 cross-checked against the CubeMX 6.15 pin database
  (`db/mcu/STM32C071K8Tx.xml` in STM32CubeIDE 1.19.0).
- Unverified: current scaling and INA180 offset near 0 mA, ST polarity on the real TPS2121.

## 2026-11-20: review fixes (firmware)

- **Status LED (PB8)** was never switched on. New `led_poll()` in `sent_app_c071.c`: off while USB is not
  configured or suspended, on when configured, a 40 ms off-blink when a SENT frame is decoded (at most every
  200 ms), 4 Hz blink while the USB load switch reports a fault (VS_EN on and VS_FLT low).
- **`CDC_Transmit_FS`** (C071 and F042 copies) returns `USBD_BUSY` when the device is not configured. Before,
  it read `pClassData->TxState` through a NULL pointer whenever the core flushed data after a bus reset or
  de-configuration (harmless only because flash at 0x214 happened to be non-zero).
- **`'L'` is now listen-only:** RX on every channel like `'O'`, but AUTO termination stays off, so a live
  ECU-sensor link can be monitored without the 10 k + 2.2 nF load, also by host tools that do not know
  command 0x07. `'O'` and `'C'` clear it. Forced ON/OFF (0x07) still wins. Test added to
  `test_line_state_follows_mode`.
- **TX start glitch:** `tx_start()` now loads CCR shadow = 0 (UG) before switching the output to PWM1. Before,
  the first frame after init (shadow CCR = 1) or after an abort drove the line LOW for a few CPU cycles. Init
  CCR is now 0.
- Comments in `C071/Core/Inc/main.h` updated for the LQFP32 board (PC6 = pin 20, PA9/PA10 not connected,
  I_SENSE scale).
- Verified against the CubeMX 6.15 pin database: TIM AF numbers (PA0/1/3 AF3, PA2 AF6, PB0 AF1, PA8 AF2,
  PA6/PA7 AF5), DMAMUX request IDs and ADC channels. TPS2121 ST polarity confirmed from the datasheet
  (high = IN1/EXT or off, low = IN2/USB).
- Builds: C071 26 520 B flash / 9 712 B RAM, F042 OK, no warnings. Application tests 21/21 (4 ch) and
  21/21 (1 ch).

## Rev 3 board (2026-11-24)

Rev 3 changes only Q1–Q4 (termination capacitor switches, AO3400A → 2N7002). Same N-channel gate polarity,
driven by TERM_EN at 3.3 V (2N7002 V_GS(th) ≤ 2.5 V), so **the rev 2f firmware runs unchanged on rev 3.**

- Checked against the rev 3 PCB netlist (`pcb/sent_pcb.kicad_pcb`), pin by pin, with the LQFP32 positions from
  the CubeMX 6.15 database: all 26 signal pins match `C071/Core/Inc/main.h` and the TX/RX tables in
  `sent_app_c071.c`. Levels confirmed from the netlist and no longer "assumed":
  - VS_EN: TPS2553 EN, active high, 100 k pull-down;
  - TERM_EN: 100 k pull-down, active high (also OE of the SN74LV1T126 pull-up buffer);
  - TX_EN: 100 k pull-up, active low;
  - LED: active high through 1 k;
  - VS divider 100 k / 22 k; I_SENSE through 1 k / 100 nF from the INA180A2.
- Comments in `main.h` and `main_c071.c` updated accordingly; no code change.
- `build_c071.sh` now also writes `Debug_C071/SENTToUSB_4CH_C071.bin` and `.hex`.
- `flash_usb.bat --4ch`: flashes the C071 over USB DFU. Default image: release asset `SENTToUSB_4CH.bin`
  (none published yet), else `SENT4CH_*.bin` next to the script, else the local build. Both board types now
  read the chip ID over DFU first (0x445 F042, 0x493 C071) and refuse a mismatch. On the C071 a wrong image
  can only be recovered over SWD, because the firmware disables the BOOT0 pin. Dry run without a board:
  stops cleanly at "No STM32 device found in DFU mode".
- Host tools: `sent_viewer.py` v2.0 detects 1 or 4 channels. `sentusb/` is a Python library, CLI and MCP
  server for both boards, with an emulator and `sentusb selftest` (loopback SENT1-SENT2, SENT3-SENT4) for
  the first rev 3 board. This closes "host tools" from the open list above. Other host scripts (`tx_test.py`, `test_sent_integration.py`, `mlx_viewer.py`,
  ...) are still the 1-channel v1.1 versions and only talk to channel 0.
- Builds (GNU Tools for STM32 13.3.rel1): C071 26 520 B flash / 9 712 B RAM, unchanged. Application tests
  21/21 (4 ch) and 21/21 (1 ch), library tests 83/83 (WSL).

## Merged into SENTToUSB (2026-11-24)

All software now lives in this repository (github.com/ucandevices/SENTToUSB); the 4-channel board's
hardware stays private.

- **The 1-channel firmware is untouched.** Every file the F042 build uses (`Core/`, `Drivers/`, `Middlewares/`,
  `USB_DEVICE/`, `.cproject`, `build.sh`, the open-sent-c submodule pin) is exactly as on `master`. Checked:
  `./build.sh` here and on `master` give a byte-identical `SENTToUSB.bin` (V0102).
- **The 4-channel firmware is self-contained in `C071/`:**
  - `C071/Core/Src`, `C071/Core/Inc`: C071 application plus its own copies of `sent_bridge`, `sent_core`,
    `sent_app.h`, `sent_app_config.h`, `slcan` and `can_frame.h` (`Core/Inc` is not on its include path);
  - `C071/Core/Inc/sent`: a copy of open-sent-c with the decoder speed-up (section 1), the `hal_stm32f042` →
    `hal_stm32` rename and the equivalence tests. Base: open-sent-c `06086e1` (the commit `master` pins).
    The same changes are stashed in the submodule (`git -C Core/Inc/sent stash list`) for upstreaming later;
  - `C071/USB_DEVICE/Class/usbd_cdc.c`: CDC class with configurable bmAttributes / MaxPower;
  - `C071/Drivers`: STM32CubeC0 HAL and CMSIS device files;
  - `C071/Tests`: application host tests (`make run`, 4 and 1 channels).
  - Shared with the F042 build, unchanged: CMSIS core headers (`Drivers/CMSIS/Include`) and the ST USB Device
    core (`Middlewares`). `C071/` is outside the CubeIDE source folders, so the CubeIDE project ignores it.
- Cost: two copies of the bridge/protocol code. A fix in `Core/Src/sent_bridge.c` does not reach
  `C071/Core/Src/sent_bridge.c` (or the reverse) unless it is copied.
- Python: `sentusb/` (library, CLI, MCP server), `tests/` (pytest, emulator), `pyproject.toml`
  (`pip install git+https://github.com/ucandevices/SENTToUSB`), `sentusb_mcp.py` (MCP without installing).
  `sentusb` and `sent_viewer.py` v2.0 run the 1-channel board on its existing V0102 firmware (v1.1 frame IDs);
  no 1-channel firmware change is needed for them.
- Two-board self-test (`sentusb -p COMa selftest --peer COMb`, MCP `run_self_test(peer_port=...)`), so the
  library and the loopback can be tested on two 1-channel SENTToUSB boards before the 4-channel board exists.
  Emulator: `emu://1ch?wire=A` twice. Fixed on the way: `transmit()` could send a frame early after a late
  wake-up, and the firmware rejected it (12.5 µs × 8 nibbles and 50 µs formats lost 2–3 of 8 frames). The
  viewer's TX period and one-shot RX switch-back are now at least one frame long.
- Checks: C071 build 26 520 B flash / 9 712 B RAM, no warnings, binary identical to the pre-merge build;
  application tests 21/21 + 21/21, library tests 83/83 (WSL); sentusb 36/36.
- First run on two real 1-channel boards (V0102, one wire: B TX → A RX, Saleae on the wire):
  `selftest --peer --direction b-to-a` passes all 6 formats, 25/25 frames each; waveform checked on the
  analyzer (3.02 µs tick, sync 169 µs, nibbles and CRC correct). Found and fixed in `sentusb` (PC side only,
  firmware unchanged): every answer arrived ~50 ms late (pyserial `read(512)` waits for its 50 ms timeout;
  now reads 1 byte then whatever is queued: 0.3 ms round trip), and TX waits under 1 ms slept 8–15 ms on
  Windows (now spins for the last 20 ms). TX at 3 µs × 6 nibbles went from 16 to ~710 frames/s, period
  1.36 ms on the wire. The emulator returns bytes at once, so the tests never showed it. New
  `selftest --direction both|a-to-b|b-to-a` (MCP `direction`) for a single wire.
