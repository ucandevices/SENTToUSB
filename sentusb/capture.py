"""capture(): record frames for a while and reduce them to statistics.

Statistics, not raw streams, are what a person or an agent can act on: frame rate,
error counters from the converter, value ranges, rolling-counter gaps, slow-channel
messages and hints when something looks wrong.  The raw frames can go to CSV.
"""
from __future__ import annotations

import csv
import time
from collections import Counter
from dataclasses import dataclass, field

from .decoders import Decoder, get_decoder
from .protocol import ChannelCounters, RxFrame


@dataclass
class ChannelCapture:
    channel: int
    mode: str
    format: dict
    duration_s: float
    frames: list[RxFrame]
    before: ChannelCounters
    after: ChannelCounters
    decoder: Decoder | None
    listen_only: bool
    decoded: list[dict] = field(default_factory=list)

    def stats(self) -> dict:
        n = len(self.frames)
        delta = self.after.minus(self.before)
        d: dict = {"channel": self.channel, "mode": self.mode, "format": self.format,
                   "frames": n, "fps": round(n / self.duration_s, 1) if self.duration_s else 0.0,
                   # error counters only: the converter's frame counter is reported every
                   # 100 ms and does not line up with the capture window
                   "device_counters_delta": {k: max(0, v) for k, v in delta.to_dict().items()
                                             if k != "frames"}}
        if n:
            ts = [f.timestamp for f in self.frames]
            gaps = [b - a for a, b in zip(ts, ts[1:])]
            d["host_max_gap_ms"] = round(max(gaps) * 1000, 2) if gaps else None
            d["last_nibbles"] = self.frames[-1].hex
            d["status_counts"] = {f"0x{k:X}": v for k, v in sorted(Counter(f.status for f in self.frames).items())}
            words = [int(f.hex, 16) for f in self.frames]
            d["raw_value"] = {"min": f"0x{min(words):X}", "max": f"0x{max(words):X}",
                              "unique": len(set(words))}
            slow: dict = {}
            for f in self.frames:
                if f.slow:
                    key = f"{f.slow.kind}:0x{f.slow.id:02X}"
                    e = slow.setdefault(key, {"count": 0})
                    e["count"] += 1
                    e["last_value"] = f.slow.value
            if slow:
                d["slow_messages"] = slow
        if self.decoder and n:
            d["decoder"] = self.decoder.name
            d["values"] = self._value_stats()
            if self.decoder.sequence_field:
                d["counter_jumps"] = self._sequence_jumps()
        d["hints"] = self._hints(d, delta)
        return d

    def _value_stats(self) -> dict:
        out: dict = {}
        fields: dict[str, list] = {}
        for v in self.decoded:
            for k, x in v.items():
                fields.setdefault(k, []).append(x)
        for k, xs in fields.items():
            if k == "decode_error":
                out[k] = xs[-1]
            elif all(isinstance(x, bool) for x in xs):
                out[k] = {"true": sum(xs), "false": len(xs) - sum(xs)}
            elif all(isinstance(x, (int, float)) for x in xs):
                out[k] = {"min": min(xs), "max": max(xs), "mean": round(sum(xs) / len(xs), 3), "last": xs[-1]}
        return out

    def _sequence_jumps(self) -> int:
        key, mod = self.decoder.sequence_field, self.decoder.sequence_modulus
        seq = [v[key] for v in self.decoded if key in v]
        return sum(1 for a, b in zip(seq, seq[1:]) if b != (a + 1) % mod)

    def _hints(self, d: dict, delta: ChannelCounters) -> list[str]:
        h = []
        n = d["frames"]
        if self.mode == "tx":
            h.append("channel is transmitting; it does not receive in TX mode")
        elif n == 0:
            if self.mode not in ("rx", "learn"):
                h.append(f"channel is '{self.mode}', not receiving: start RX")
            elif delta.sync_errors > 0:
                h.append("edges arrive but do not match the configured tick/nibble count: run learn")
            elif delta.crc_errors > 0:
                h.append("frames arrive but fail CRC: wrong crc_mode / crc_seed / legacy_crc, run learn")
            else:
                h.append("no SENT edges: check wiring (GND-SENT-VS), sensor supply and the channel number")
                if self.listen_only:
                    h.append("listen-only is on (termination off): a sensor alone on the bench usually "
                             "needs termination, reconnect with listen_only=false")
        else:
            if delta.crc_errors:
                h.append(f"{delta.crc_errors} CRC errors during the capture")
            if delta.sync_errors:
                h.append(f"{delta.sync_errors} sync errors during the capture")
            if delta.lost_edges or delta.usb_overflows:
                h.append(f"converter lost data: {delta.lost_edges} lost edges, {delta.usb_overflows} "
                         "frames dropped on USB (too much traffic or a slow host)")
            if d.get("counter_jumps"):
                h.append(f"{d['counter_jumps']} rolling-counter jumps: frames missing or the sensor "
                         "repeats/skips (secure sensor check)")
            v = d.get("values", {})
            if "inverted_ok" in v and v["inverted_ok"]["false"]:
                h.append(f"{v['inverted_ok']['false']} frames fail the inverted-nibble check")
            for flag in ("error1", "error2"):
                if flag in v and v[flag]["true"]:
                    h.append(f"sensor reports {flag} (status bit) in {v[flag]['true']} frames")
            if "decode_error" in v:
                h.append(v["decode_error"])
        return h


@dataclass
class CaptureResult:
    duration_s: float
    channels: list[ChannelCapture]
    csv_path: str | None = None

    def to_dict(self, include_frames: int = 0) -> dict:
        out = {"duration_s": round(self.duration_s, 3), "csv_path": self.csv_path,
               "channels": [c.stats() for c in self.channels]}
        if include_frames:
            for c, s in zip(self.channels, out["channels"]):
                s["first_frames"] = [f.to_dict() for f in c.frames[:include_frames]]
        return out

    def summary(self) -> str:
        lines = []
        for c in self.channels:
            s = c.stats()
            err = s["device_counters_delta"]
            line = (f"CH{s['channel']} {s['mode']:<5} {s['frames']:6d} frames {s['fps']:8.1f} fps  "
                    f"CRC {err['crc_errors']}  sync {err['sync_errors']}  lost {err['lost_edges']}")
            if s["frames"]:
                line += f"  last {s['last_nibbles']}"
            lines.append(line)
            for k, v in s.get("values", {}).items():
                lines.append(f"      {k}: {v}")
            for hint in s["hints"]:
                lines.append(f"      ! {hint}")
        if self.csv_path:
            lines.append(f"CSV: {self.csv_path}")
        return "\n".join(lines)


def run_capture(dev, duration_s: float, channels=None, *, decoder=None,
                csv_path=None) -> CaptureResult:
    if duration_s <= 0:
        raise ValueError("duration must be > 0")
    numbers = [dev._ch(c).number for c in (channels or range(1, dev.channel_count + 1))]
    decoders = {}
    for n in numbers:
        name = decoder.get(n) if isinstance(decoder, dict) else decoder
        decoders[n] = get_decoder(name)
    dev.refresh_info()
    before = {n: dev.counters(n) for n in numbers}
    got: dict[int, list] = {n: [] for n in numbers}

    def cb(fr):
        lst = got.get(fr.channel)
        if lst is not None:
            lst.append(fr)

    dev.subscribe(cb)
    t0 = time.perf_counter()
    try:
        time.sleep(duration_s)
    finally:
        dev.unsubscribe(cb)
    elapsed = time.perf_counter() - t0
    time.sleep(0.15)                       # diagnostics frames come at most every 100 ms
    caps = []
    for n in numbers:
        ch = dev._ch(n)
        cap = ChannelCapture(n, ch.mode, ch.fmt.to_dict(), elapsed, got[n], before[n], dev.counters(n),
                             decoders[n], dev.listen_only)
        if cap.decoder:
            cap.decoded = [cap.decoder.decode(f) for f in cap.frames]
        caps.append(cap)

    if csv_path:
        _write_csv(csv_path, caps)
    return CaptureResult(elapsed, caps, csv_path)


def _write_csv(path: str, caps: list[ChannelCapture]):
    fields: list[str] = []
    for c in caps:
        for v in c.decoded[:1]:
            fields += [k for k in v if k not in fields]
    rows = []
    for c in caps:
        for i, f in enumerate(c.frames):
            row = {"time": f"{f.timestamp:.6f}", "channel": f.channel, "can_id": f"{f.can_id:03X}",
                   "status": f"{f.status:X}", "nibbles": f.hex,
                   "slow_kind": f.slow.kind if f.slow else "", "slow_id": f"{f.slow.id:02X}" if f.slow else "",
                   "slow_value": f.slow.value if f.slow else ""}
            if c.decoded:
                row.update({k: v for k, v in c.decoded[i].items() if k in fields})
            rows.append(row)
    rows.sort(key=lambda r: r["time"])
    header = ["time", "channel", "can_id", "status", "nibbles", "slow_kind", "slow_id", "slow_value"] + fields
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=header)
        w.writeheader()
        w.writerows(rows)
