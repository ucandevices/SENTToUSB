SENTToUSB — Current Implementation Summary
  
  Hardware Topology

  Pin: PA2  Role: SENT RX input   
  Peripheral: TIM2 CH3 input capture (AF2)
  Notes: Rising-edge capture, internal pull-up. Sees SENT bus directly (idle HIGH, active LOW pulse).
  ────────────────────────────────────────
  Pin: PB0  Role: SENT TX output
  Peripheral: TIM3 CH3 output compare (AF1)  Notes: Drives an inverting NPN transistor that pulls the SENT bus to GND. PB0 HIGH → transistor ON → bus LOW (active pulse). PB0 LOW → bus HIGH (idle).  ────────────────────────────────────────
  Pin: USB  Role: CDC virtual COM
  Peripheral: USB FS  Notes: SLCAN protocol over CDC. Host sees a serial port.

  Clocks: STM32F042G4UX, HSI48 (48 MHz, no crystal), USB SOF auto-trim. APB1 = AHB = 48 MHz, so TIM2/TIM3 also clock at 48 MHz.

  ---  TX Path (host SLCAN → SENT bus)
  
  TIM3 configuration (MX_TIM3_Init)
  - PSC = 0 (full 48 MHz tick rate)
  - ARR = 65535 (placeholder; rewritten per frame)
  - ARPE = DISABLE (ARR writes immediate, no preload)
  - OC3 in FORCED_INACTIVE (OCxREF=0 → PB0 LOW → bus HIGH = idle)
  - CCR3 = 1 (placeholder; rewritten per frame to low_ticks × cycles_per_tick)
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
  2. Fold each [LOW, HIGH] pair into a single ARR value (LOW+HIGH)×cycles_per_tick − 1, in-place. After fold: intervals[0..9] hold per-interval ARR  values.
  3. URS+UG flush — preload TIM3 state without generating a UEV that would fire DMA:
    - CR1 |= URS (only counter overflow generates UEV/DMA request)
    - Write CCR3, ARR (= sync ARR), and CCMR2 = FORCED_ACTIVE (OCxREF=1 forced)
    - EGR = UG to refresh the shadow registers and reset CNT (no UEV because URS=1)    - SR = 0 (clear UIF)
    - CR1 &= ~URS, CNT = 0
  4. CR1 |= CEN — counter starts. OCxREF=1 (forced) → PB0 HIGH → bus drops LOW instantly = start of sync's 15 µs active-LOW pulse.
  5. Spin until TIM3->CNT >= ccr3 (~15 µs of busy-wait inside the USB CDC ISR).
  6. Write CCMR2 = FORCED_INACTIVE (immediate effect, OCxREF=0 → bus rises HIGH = end of sync's LOW pulse).
  7. Write CCMR2 = PWM1 (queued — takes effect at next UEV, end of sync at 168 µs). At that UEV the OC unit starts driving subsequent intervals' LOW  pulses automatically.
  8. Set DIER |= UDE and call HAL_DMA_Start_IT(intervals[1..n_raw-1] → &TIM3->ARR, length=n_raw-1).
  9. From here it's hardware: each UEV (end of an interval) triggers a DMA transfer that loads the next ARR. PWM1 produces a 15 µs LOW pulse at the start  of each new interval (CNT < CCR3 → OCxREF=1) followed by HIGH (CNT ≥ CCR3 → OCxREF=0).
  Cleanup at end of frame

  - DMA TC callback (tim3_tx_dma_complete) fires after the last DMA transfer (= end-of-CRC UEV, with pause ARR loaded): clears UDE, sets UIE.
  - SentApp_OnTim3UpdateIrq (TIM3 update IRQ) fires at end-of-pause UEV: clears CEN, clears UIE, writes CCMR2 = FORCED_INACTIVE, resets g_tx_hal
  count/index. Bus returns to idle HIGH.

  There's a known harmless ~6 µs end-of-frame glitch caused by ISR latency between the end-of-pause UEV and the cleanup ISR's CCMR2 write — the RX HAL's  sync-detect re-aligns on the next frame's status edge so it doesn't enter the decoded batch.
  ---
  RX Path (SENT bus → host SLCAN)

  TIM2 configuration

  - CH3 input capture on PA2, AF2, internal pull-up  - Rising-edge trigger
  - PSC = 0, free-running 16-bit at 48 MHz (overflow every ~1.37 ms)
  - CC3 IRQ + TIM2 update IRQ (overflow) both enabled

  TIM2 ISRs (main.c)
  - HAL_TIM_IC_CaptureCallback — every rising edge on PA2 captures TIM2 CCR3 (16-bit) and calls SentApp_OnSentRxCaptureEdge(captured).
  - HAL_TIM_PeriodElapsedCallback — every TIM2 overflow calls SentApp_OnSentRxTimerOverflow() so the RX HAL can stitch 16-bit captures into 64-bit
  timestamps.

  RX HAL state (sent_stm32f042_rx_hal_t)

  - capture_batch_size = data_nibbles + 4 = 10 edges for 6-nibble frames (sync_end + status_end + 6 nibble_ends + CRC_end + pause_end)
  - ready_queue_depth = 3 (ISR→main lock-free ring of completed batches)
  - sync_min_us = 100 µs — any inter-edge gap ≥ this triggers batch re-alignment
  - Q12 fixed-point ticks→µs converter with frac accumulator (mul_q12 = 85 for 48 MHz; 0.39 % systematic error, frac carries it across calls so cumulative
   error stays bounded)
  Capture ISR (sent_stm32f042_rx_on_capture_edge_isr)

  Per rising edge:  1. Stitch 16-bit CCR with 32-bit overflow count → 64-bit absolute tick.
  2. Compute Δticks since previous edge → Δµs via Q12 (no division in ISR).
  3. Update last_timestamp_us.
  4. Sync detection: if Δµs ≥ sync_min_us AND active_count > 0, the previous edge was the sync's rising edge — reset the active batch and seed ts[0] = 
  previous_timestamp. This fires twice per frame: once on the new sync_rise (huge inter-frame gap) and once on the next status_rise (sync's 168 µs > 100
  µs threshold), leaving ts[0]=sync_rise, ts[1]=status_rise correctly aligned.  5. Append current timestamp to the active batch.
  6. When active_count == capture_batch_size (10), push batch to the ready queue (lock-free, single-producer-single-consumer ring).

  After push, active_count is reset to 1 (not 0) with ts[0] carrying the last-pushed edge — this preserves the sync-detect guard for the next frame.
  Main-loop drain (SentApp_Process in sent_app.c)
  While in RX mode ('O' opened the channel):
  1. Drain all completed batches from the RX HAL ring.
  2. Each batch (10 timestamps) → sent_bridge_on_sent_timestamps_us → sent_decode_from_timestamps_us:
    - Compute 9 intervals from 10 timestamps.
    - For start = 0 (only valid start with 9 intervals and required=9):
        - Validate intervals[0] (sync) is in [56 × min_tick, 56 × max_tick] µs (= 140–5040 µs for our config).
      - Treat sync as the tick reference: tick = sync_us / 56.
      - For each of 8 nibbles (status + 6 data + CRC), round interval × 56 / sync to integer ticks; verify within ±0.5 ticks (SENT_TOLERANCE_NUM = 50 % —
  bumped from 35 to match SAE J2716 spec max).      - Verify nibble_ticks ∈ [12, 27].
      - Recompute CRC4 over data + status (per crc_mode); compare to received CRC.
  3. On success → pack nibbles MSB-first into a CAN frame (ID 0x510, DLC=3) and queue as a SLCAN t510…\r line in the USB TX ring.
  4. Slow-channel decoder runs in parallel; if a slow message completes, the same 0x510 frame extends to DLC=7 with slow-channel data appended.
  5. On failure → increment crc_errors (CRC-mode mismatch only) or sync_errors (everything else: sync out-of-range, shape error, or no valid start). A
  0x511 diag SLCAN frame is auto-emitted to USB whenever any counter changes.
  ---
  SLCAN Command Plane (USB CDC)  
  'O' open RX, 'C' close, 'V'/'v' HW/FW version, 'N' UID-derived serial, 'F' flags.
  Control frames (CAN ID 0x600):
  - data[0]=0x01 start RX  - data[0]=0x02 start TX (mutually exclusive with RX)
  - data[0]=0x03 stop
  - data[0]=0x04 learn tick/nibbles/CRC mode from a live signal (3 hits per combo)
  - data[0]=0x05, data[1..2]=tick_x10_us LE set TX tick (range 2.0–90.0 µs)

  Config frame (CAN ID 0x001): rewrite SENT params (data nibbles, CRC mode, seed, min/max tick, output CAN ID).
  TX data frame (CAN ID 0x520): status nibble + packed data nibbles MSB-first, optional explicit pause_ticks in DLC=7 form.
  RX output (CAN ID 0x510): decoded nibbles packed MSB-first; extends to DLC=7 if a slow-channel message completed.

  Diag (CAN ID 0x511): auto-emitted whenever frames_decoded / crc_errors / sync_errors change — data[0..3]=frames LE32, data[4..5]=crc LE16,   data[6..7]=sync LE16.
  ---
  Buffers & ISR Discipline
  
  - USB CDC TX ring: 384-byte software ring. ISRs push (usb_tx_push); main loop calls CDC_Transmit_FS from usb_tx_flush (CDC must not be called from ISR).
  - SLCAN RX line buffer: 64 bytes; assembled byte-by-byte in SentApp_OnUsbRx (USB ISR), dispatched on \r/\n.
  - RX HAL ring: 3 slots, single-producer (TIM2 ISR) / single-consumer (main loop), no locks needed on Cortex-M0.
  - TX HAL intervals[]: populated by sent_stm32f042_tx_submit, then folded in-place by tim3_dma_start. volatile count published last (M0 in-order store  model).
  - NVIC priorities: TIM2 = 1, TIM3 = 1, DMA1_Ch2_3 = 1 (USB at default 0). USB takes precedence over SENT processing.