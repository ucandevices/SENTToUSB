SENTToUSB — Current Implementation Summary
  
  Hardware Topology

  Pin: PA2  Role: SENT RX input   
  Peripheral: TIM2 CH3 input capture (AF2)
  Notes: Falling-edge capture (J2716 measures falling edge to falling edge), internal pull-up. Sees SENT bus directly (idle HIGH, active LOW pulse).
  ────────────────────────────────────────
  Pin: PB0  Role: SENT TX output
  Peripheral: TIM3 CH3 output compare (AF1)  Notes: Drives an inverting NPN transistor that pulls the SENT bus to GND. PB0 HIGH → transistor ON → bus LOW (active pulse). PB0 LOW → bus HIGH (idle).  ────────────────────────────────────────
  Pin: USB  Role: CDC virtual COM
  Peripheral: USB FS  Notes: SLCAN protocol over CDC. Host sees a serial port.

  Clocks: STM32F042G4UX, HSI48 (48 MHz, no crystal), USB SOF auto-trim. APB1 = AHB = 48 MHz, so TIM2/TIM3 also clock at 48 MHz.

  ---  TX Path (host SLCAN → SENT bus)
  
  TIM3 configuration (MX_TIM3_Init)
  - PSC = 0 at init; tim3_dma_start() sets it per frame so the longest interval fits the 16-bit ARR
  - ARR = 65535 (placeholder; rewritten per frame)
  - ARPE = DISABLE (ARR writes immediate, no preload)
  - OC3 in FORCED_INACTIVE (OCxREF=0 → PB0 LOW → bus HIGH = idle)
  - CCR3 = 1 (placeholder; rewritten per frame), preloaded (OC3PE) so it can be changed mid-frame and apply at the next update
  - CC3E = 1 (output enabled)
  DMA1 Channel 3 (HAL_TIM_Base_MspInit for TIM3)
  - Direction: memory-to-peripheral
  - Source: g_tx_hal.intervals[1..n_raw-1] (memory increment ON)
  - Destination: &TIM3->ARR (peripheral increment OFF)
  - Width: half-word ↔ half-word
  - Mode: normal (auto-disable when CNDTR=0)
  - Priority: HIGH
  - Triggered by: TIM3 update event (UDE bit in DIER)
  - Linked via: __HAL_LINKDMA(htim3, hdma[TIM_DMA_ID_UPDATE], hdma_tim3_up)
  Frame transmit sequence (tim3_dma_start in sent_app.c)
  When the host sends t52050100123456 (TX SENT frame):

  1. Encode the SENT frame into LOW/HIGH-tick pairs in g_tx_hal.intervals[] via sent_build_intervals_ticks(). For 6 nibbles: 10 intervals (sync 56T +  status + 6 data + CRC + pause 12T).
  2. Choose the prescaler: div = ceil(longest interval in cycles / 65536). Needed above a 24 µs tick (sync = 56 ticks) and for long pause pulses (768 ticks  is 110592 cycles at 3 µs).
  3. Fold each [LOW, HIGH] pair into a single ARR value round((LOW+HIGH)×cycles_per_tick/div) − 1, in-place.
  4. LOW drive time: nominal 5 ticks minus 10 µs, at least 8 µs. The transistor switches off ~11 µs after PB0 falls (10 kΩ gate resistor), so an  uncompensated 15 µs pulse is 26–29 µs LOW on the bus and shifts the next falling edge by up to 0.5 tick at 3 µs. Measured after compensation: 13–14 µs  LOW, edge error ~0.1 tick.
  5. URS+UG flush — load PSC, CCR3, ARR (= sync) with CCMR2 = FORCED_ACTIVE, without generating a UEV that would fire DMA.
  6. With interrupts masked: CR1 |= CEN (bus drops LOW = start of sync), spin until CNT >= CCR3, write FORCED_INACTIVE (bus HIGH), then PWM1.  PWM1 sees CNT >= CCR3 and stays inactive for the rest of the sync.  Runs in the main loop.
  7. Set DIER |= UDE and call HAL_DMA_Start_IT(intervals[1..n_raw-1] → &TIM3->ARR, length=n_raw-1).
  8. From here it's hardware: each UEV (end of an interval) triggers a DMA transfer that loads the next ARR. PWM1 produces the LOW pulse at the start  of each new interval (CNT < CCR3 → OCxREF=1) followed by HIGH (CNT ≥ CCR3 → OCxREF=0).
  Cleanup at end of frame

  - DMA TC callback (tim3_tx_dma_complete) fires right after the UEV that starts the last interval (pause): clears UDE, writes CCR3 = 0 (preloaded,  takes effect at the next UEV), sets UIE.
  - SentApp_OnTim3UpdateIrq (TIM3 update IRQ) fires at end-of-pause UEV: clears CEN, clears UIE, writes CCMR2 = FORCED_INACTIVE, resets g_tx_hal
  count/index. Bus returns to idle HIGH.

  Because CCR3 is 0 from that UEV on, PWM1 does not start another LOW pulse while the cleanup ISR is pending — no runt pulse after the frame.  The pause  interval therefore has no closing falling edge; the frame's last edge is the start of the pause pulse.
  ---
  RX Path (SENT bus → host SLCAN)

  TIM2 configuration

  - CH3 input capture on PA2, AF2, internal pull-up, falling-edge trigger, input filter 8 samples
  - PSC = 0, ARR = 0xFFFFFFFF: TIM2 is a 32-bit timer, free-running at 48 MHz (wraps every 89 s)
  - CC3 IRQ only.  No overflow interrupt: intervals are differences of captured values modulo 2^32.

  Capture ISR (sent_stm32f042_rx_on_capture_edge_isr)

  Stores the raw 32-bit capture value in a 64-entry single-producer / single-consumer ring.  Nothing else — no unit conversion, no framing.  If the  ring is full the edge is dropped and dropped_edges is incremented.

  Main-loop drain (SentApp_Process in sent_app.c)
  1. Drain the edge ring in chunks and feed each edge to sent_bridge_on_edge().
  2. The bridge keeps a window of the most recent edges.  When it holds data_nibbles + 4 edges it calls sent_decode_from_timestamps() with 48 counts  per µs:
    - intervals[0] (sync) must be in [56 × min_tick, 56 × max_tick]; default range 2.0–113 µs
    - tick reference = sync / 56; each of the nibbles (status + data + CRC) is rounded to ticks, must be within ±0.35 tick and in [12, 27]
    - CRC per configuration: recommended (start 0x03), variant (0x05) or J2716 legacy; data only or status + data
  3. Success → the frame is consumed, its last edge stays as the first edge of what follows.  Failure → the oldest edge is dropped (this steps over a  pause pulse and re-aligns after a disturbance).
  4. Between two good frames exactly one skipped interval is accepted as a pause pulse if it is 12..768 ticks long.  A longer one is an idle line (not  an error, but the frames are not consecutive); anything else counts one sync error.
  5. Successive calibration pulses differing by more than 1/64 reject the frame (J2716 5.3.3.1).
  6. Good frame → 0x510: data nibbles, then an info byte (status nibble in bits 7..4, slow-channel flags in bits 2..0), then slow ID + data when a  slow-channel message completed.
  7. Counters: crc_errors (well-formed frame, wrong checksum), sync_errors (malformed frame, bad gap, calibration mismatch), dropped_events (edge ring  overrun; alignment and slow channel are reset).  A 0x511 diag frame is emitted when a counter changed, at most every 100 ms.

  Learn mode (0x600 / 04) runs on the same edge stream: after every edge each nibble count 1..8 is tried on the window ending at that edge, against six  CRC variants.  A variant needs 8 consecutive matching frames; the longest qualifying nibble count wins after one more frame of settling.
  ---
  SLCAN Command Plane (USB CDC)  
  'O' open RX, 'C' close, 'V'/'v' HW/FW version, 'N' UID-derived serial, 'F' flags.
  Control frames (CAN ID 0x600):
  - data[0]=0x01 start RX  - data[0]=0x02 start TX (mutually exclusive with RX)
  - data[0]=0x03 stop
  - data[0]=0x04 learn tick / nibbles (1..8) / CRC mode / CRC variant from a live signal (8 consecutive hits)
  - data[0]=0x05, data[1..2]=tick_x10_us LE set TX tick (range 2.0–90.0 µs)

  Config frame (CAN ID 0x001): rewrite SENT params (data nibbles, CRC mode, seed, min/max tick, output CAN ID).
  TX data frame (CAN ID 0x520): status nibble + packed data nibbles MSB-first, optional explicit pause_ticks in DLC=7 form.
  RX output (CAN ID 0x510): decoded nibbles packed MSB-first + info byte (status nibble, slow flags); DLC 4, or 7 if a slow-channel message completed  (6 nibbles).

  Diag (CAN ID 0x511): auto-emitted (at most every 100 ms) when frames_decoded / crc_errors / sync_errors changed — data[0..3]=frames LE32, data[4..5]=crc LE16,   data[6..7]=sync LE16.
  ---
  Buffers & ISR Discipline
  
  - USB RX ring: 128 bytes, filled by SentApp_OnUsbRx (USB ISR), drained by the main loop, which assembles lines (64-byte buffer) and dispatches them.   All bridge state is touched from the main loop only.
  - USB CDC TX ring: 384-byte software ring, main loop only.  A line that does not fit is dropped whole.  usb_tx_flush hands the ring to CDC_Transmit_FS.
  - RX edge ring: 64 raw capture values, single-producer (TIM2 ISR) / single-consumer (main loop), no locks needed on Cortex-M0.
  - TX HAL intervals[]: populated by sent_stm32f042_tx_submit, then folded in-place by tim3_dma_start. volatile count published last (M0 in-order store  model).
  - NVIC priorities: TIM2 = 1, TIM3 = 1, DMA1_Ch2_3 = 1 (USB at default 0). USB takes precedence over SENT processing.