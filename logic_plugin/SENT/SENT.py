"""
SENT Protocol High Level Analyzer for Logic 2 (SAE J2716).

Input : any LLA that produces frames whose start_time equals a SENT
        falling edge  (Async Serial at high baud works; UARTAnalyzer
        also works).  Only the *timestamps* of the LLA frames are used.

Settings
--------
tick_us       : nominal tick period in µs (default 3.0 for MLX90377)
num_nibbles   : data nibbles per frame (default 6)
crc_mode      : "DATA_ONLY" or "STATUS_AND_DATA"
crc_seed      : "0x03 (APR2016)" or "0x05 (legacy)"

Frame types emitted
-------------------
"sync"   : sync interval detected
"nibble" : one decoded nibble (status or data)
"frame"  : complete decoded SENT frame with all nibbles and CRC result
"error"  : CRC mismatch or out-of-range nibble

Algorithm
---------
Consecutive falling-edge timestamps give *intervals*.
  interval_ticks = round(interval_us / tick_us)
  sync  : 56 ticks  (tolerance ±35%)  → 36..75 ticks
  nibble: 12..27 ticks (value = ticks - 12)

The gap filter rejects intervals shorter than GAP_MIN_TICKS = 10 ticks.
These arise from the Async Serial LLA re-detecting edges on the SENT
HIGH-to-LOW pulse (which is only ~3 ticks wide).  Any interval >= 56*0.65
ticks is treated as sync/pause and resets the state machine.

State machine: WAIT_SYNC -> STATUS -> DATA*N -> CRC -> (emit frame) -> STATUS...

NOTE: capture the SENT signal at >= 10 MHz for reliable decoding.
At 500 kHz the 2 us quantization (~0.67 tick) causes frequent nibble
rounding errors; this is a hardware limitation, not a decoder bug.
"""

from saleae.analyzers import HighLevelAnalyzer, AnalyzerFrame, NumberSetting, ChoicesSetting

# SAE J2716 CRC-4 LUT, poly = 0x0D
_CRC_LUT = [0, 13, 7, 10, 14, 3, 9, 4, 1, 12, 6, 11, 15, 2, 8, 5]

def _crc4(nibbles, status_nibble, mode, seed):
    """Compute SAE J2716 CRC-4.

    nibbles       : list of data nibble values (0-15)
    status_nibble : status nibble value (0-15)
    mode          : "DATA_ONLY" or "STATUS_AND_DATA"
    seed          : integer seed (3 or 5)
    """
    crc = seed
    if mode == "STATUS_AND_DATA":
        crc = _CRC_LUT[crc ^ (status_nibble & 0xF)]
    for n in nibbles:
        crc = _CRC_LUT[crc ^ (n & 0xF)]
    return crc


# Tick tolerance: SAE J2716 allows ±25%; we use ±35% to be robust.
_SYNC_TICKS       = 56
_SYNC_MIN_TICKS   = int(_SYNC_TICKS * 0.65)   # 36
_SYNC_MAX_TICKS   = int(_SYNC_TICKS * 1.35)   # 75
_NIBBLE_MIN_TICKS = 12
_NIBBLE_MAX_TICKS = 27
_GAP_MIN_TICKS    = 10   # intervals shorter than this are Async-Serial glitches


class SentAnalyzer(HighLevelAnalyzer):

    # ── Settings ──────────────────────────────────────────────────────────────
    tick_us     = NumberSetting(min_value=0.5, max_value=100.0)
    num_nibbles = NumberSetting(min_value=1,   max_value=8)
    crc_mode    = ChoicesSetting(choices=("DATA_ONLY", "STATUS_AND_DATA"))
    crc_seed    = ChoicesSetting(choices=("0x03 (APR2016)", "0x05 (legacy)"))

    # Frame types → result columns shown in Logic 2
    result_types = {
        "sync":   {"format": "SYNC tick={{data.tick_us}}µs"},
        "nibble": {"format": "{{data.kind}}[{{data.index}}]=0x{{data.value}}"},
        "frame":  {"format": "SENT {{data.status}} | {{data.data}} | CRC={{data.crc_result}}"},
        "error":  {"format": "ERR {{data.reason}}"},
    }

    # ── Internal state ─────────────────────────────────────────────────────────
    _STATE_WAIT_SYNC = 0
    _STATE_STATUS    = 1
    _STATE_DATA      = 2
    _STATE_CRC       = 3

    def __init__(self):
        self._seed  = 3   # resolved in first decode
        self._state = self._STATE_WAIT_SYNC
        self._sync_start  = None   # start_time of sync edge
        self._sync_end    = None   # start_time of first edge after sync
        self._tick_us     = None   # learned from sync interval
        self._status      = None
        self._data        = []
        self._nibble_starts = []   # start_time of each nibble edge
        self._prev_time   = None   # start_time of last accepted edge

    # ── Logic 2 callback ──────────────────────────────────────────────────────
    def decode(self, frame: AnalyzerFrame):
        """Called once per LLA frame.  We use frame.start_time as the
        falling-edge timestamp.
        """
        t = frame.start_time
        outputs = []

        if self._prev_time is None:
            self._prev_time = t
            return outputs

        interval_s  = float(t - self._prev_time)
        interval_us = interval_s * 1e6

        # Seed from settings (resolved once)
        seed = 3 if "03" in self.crc_seed else 5

        # Nominal tick from settings; will be refined after sync
        nom_tick = float(self.tick_us)
        ref_tick = self._tick_us if self._tick_us else nom_tick

        # Gap filter: ignore sub-threshold intervals (Async Serial glitches)
        if interval_us < _GAP_MIN_TICKS * ref_tick:
            # Do NOT advance _prev_time — the real next edge is still coming
            return outputs

        sync_start_time = self._prev_time   # save before overwriting
        self._prev_time = t

        ticks = interval_us / ref_tick

        # ── Sync / reset detection ─────────────────────────────────────────
        if _SYNC_MIN_TICKS <= round(ticks) <= _SYNC_MAX_TICKS:
            # Commit learned tick from actual sync interval
            self._tick_us = interval_us / _SYNC_TICKS
            self._sync_start = sync_start_time  # edge that started this interval
            self._sync_end   = t
            self._state      = self._STATE_STATUS
            self._status     = None
            self._data       = []
            self._nibble_starts = []
            outputs.append(AnalyzerFrame(
                "sync",
                self._sync_start,
                t,
                {"tick_us": f"{self._tick_us:.3f}"}
            ))
            return outputs

        # Long interval that is NOT in sync range: treat as pause/reset
        if ticks > _SYNC_MAX_TICKS:
            self._state  = self._STATE_WAIT_SYNC
            self._tick_us = None
            return outputs

        # ── Nibble decoding ────────────────────────────────────────────────
        if self._state == self._STATE_WAIT_SYNC:
            return outputs

        nibble_ticks = round(ticks)
        if not (_NIBBLE_MIN_TICKS <= nibble_ticks <= _NIBBLE_MAX_TICKS):
            outputs.append(AnalyzerFrame(
                "error",
                sync_start_time,
                t,
                {"reason": f"ticks={nibble_ticks:.1f} out of range [12,27]"}
            ))
            self._state = self._STATE_WAIT_SYNC
            return outputs

        value = nibble_ticks - 12
        edge_start = sync_start_time   # start of this nibble's interval

        if self._state == self._STATE_STATUS:
            self._status = value
            outputs.append(AnalyzerFrame(
                "nibble", edge_start, t,
                {"kind": "ST", "index": 0, "value": f"{value:X}"}
            ))
            self._state = self._STATE_DATA

        elif self._state == self._STATE_DATA:
            idx = len(self._data)
            self._data.append(value)
            outputs.append(AnalyzerFrame(
                "nibble", edge_start, t,
                {"kind": "D", "index": idx, "value": f"{value:X}"}
            ))
            if len(self._data) >= int(self.num_nibbles):
                self._state = self._STATE_CRC

        elif self._state == self._STATE_CRC:
            crc_recv = value
            crc_calc = _crc4(
                self._data,
                self._status if self._status is not None else 0,
                self.crc_mode,
                seed,
            )
            crc_ok = (crc_recv == crc_calc)
            outputs.append(AnalyzerFrame(
                "nibble", edge_start, t,
                {"kind": "CRC", "index": int(self.num_nibbles) + 1,
                 "value": f"{crc_recv:X}"}
            ))

            data_str  = "".join(f"{d:X}" for d in self._data)
            crc_label = "OK" if crc_ok else f"ERR(calc={crc_calc:X})"
            frame_type = "frame" if crc_ok else "error"

            outputs.append(AnalyzerFrame(
                frame_type,
                self._sync_end if self._sync_end else edge_start,
                t,
                {
                    "status":     f"{self._status:X}" if self._status is not None else "?",
                    "data":       data_str,
                    "crc_result": crc_label,
                    "reason":     f"CRC mismatch recv={crc_recv:X} calc={crc_calc:X}" if not crc_ok else "",
                }
            ))

            # Ready for next frame's status nibble
            self._state  = self._STATE_STATUS
            self._status = None
            self._data   = []

        return outputs
