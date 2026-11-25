"""Board self-test, one 4-channel board or two boards of any kind.

Wiring for the loopback part:
  one 4-channel board: SENT1 to SENT2 and SENT3 to SENT4 (one wire each, signal
      pins only; GND is common on the board).
  two boards (peer=..., e.g. two 1-channel SENTToUSB): the SENT line of board A
      to the SENT line of board B, plus GND.  On the 1-channel board that is
      TX (PB0) of each board to RX (PA2) of the other; see the README.
      With a single wire, test that direction only (direction="b-to-a" for
      B's TX to A's RX): both boards run the same firmware, so one direction
      already covers TX and RX; the other only adds the two unused pins.
No sensors connected.

Checks:
  board        firmware answers, channel count, features
  supply       VS within 4.4..5.5 V on USB power, no load-switch fault
  termination  forced on / forced off / auto read back on every channel
  loopback     each pair, both directions, several formats: every frame sent
               on one channel is received unchanged on the other, no CRC/sync errors
"""
from __future__ import annotations

import queue
import time
from dataclasses import dataclass, field

from .device import SentUSB, SentUSBError
from .protocol import CrcMode, SentFormat

FORMATS_FULL = [
    SentFormat(3.0, 6, CrcMode.DATA_ONLY, 0x03),
    SentFormat(3.0, 6, CrcMode.STATUS_AND_DATA, 0x05),
    SentFormat(3.0, 6, CrcMode.DATA_ONLY, 0x05, legacy_crc=True),
    SentFormat(5.0, 4, CrcMode.DATA_ONLY, 0x03),
    SentFormat(12.5, 8, CrcMode.STATUS_AND_DATA, 0x03),
    SentFormat(50.0, 3, CrcMode.DATA_ONLY, 0x03),
]
FORMATS_QUICK = FORMATS_FULL[:2]


@dataclass
class Check:
    name: str
    passed: bool
    detail: str = ""
    data: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        d = {"name": self.name, "passed": self.passed, "detail": self.detail}
        if self.data:
            d["data"] = self.data
        return d


@dataclass
class SelfTestReport:
    checks: list[Check]

    @property
    def passed(self) -> bool:
        return all(c.passed for c in self.checks)

    def to_dict(self) -> dict:
        return {"passed": self.passed, "failed": [c.name for c in self.checks if not c.passed],
                "checks": [c.to_dict() for c in self.checks]}

    def summary(self) -> str:
        lines = [f"{'PASS' if c.passed else 'FAIL'}  {c.name}" + (f"  - {c.detail}" if c.detail else "")
                 for c in self.checks]
        lines.append(f"\n{'ALL PASSED' if self.passed else 'FAILED'}: "
                     f"{sum(c.passed for c in self.checks)}/{len(self.checks)}")
        return "\n".join(lines)


def _pattern(i: int, nibbles: int, with_status: bool) -> tuple[int, tuple[int, ...]]:
    status = (i & 0x3) if with_status else 0      # bits 2/3 carry the slow channel: keep them 0
    return status, tuple((i * 7 + k * 3 + (i >> 4)) & 0xF for k in range(nibbles))


def _fmt_name(fmt: SentFormat) -> str:
    return f"{fmt.tick_us:g}us {fmt.nibbles}nib {fmt.crc_mode.value} " \
           f"seed 0x{fmt.crc_seed:X}{' legacy' if fmt.legacy_crc else ''}"


def loopback_pair(dev: SentUSB, tx: int, rx: int, fmt: SentFormat, frames: int = 25,
                  rx_dev: SentUSB | None = None, label: str = "") -> Check:
    """Transmit `frames` frames on channel `tx` of `dev` and expect them unchanged on
    channel `rx` of `rx_dev` (default: the same board)."""
    tx_dev = dev
    rx_dev = rx_dev or dev
    name = f"loopback {label or f'CH{tx}->CH{rx}'} {_fmt_name(fmt)}"
    tx_dev.stop(tx)
    tx_dev.configure(tx, fmt)
    rx_dev.configure(rx, fmt)
    rx_dev.start_rx(rx)
    sent = [_pattern(i, fmt.nibbles, fmt.crc_mode is CrcMode.STATUS_AND_DATA) for i in range(frames)]
    before = rx_dev.counters(rx)
    with rx_dev.stream(rx) as q:
        job = tx_dev.transmit(tx, sent, count=frames, force=True)
        job.wait(timeout=frames * 0.2 + 2)
        got = []
        frame_s = tx_dev.frame_duration_s(tx, 15, [15] * fmt.nibbles)
        deadline = time.time() + max(0.2, 5 * frame_s)
        while time.time() < deadline:
            try:
                f = q.get(timeout=0.05)
                got.append((f.status, f.nibbles))
            except queue.Empty:
                pass
    time.sleep(0.15)
    delta = rx_dev.counters(rx).minus(before)
    tx_dev.stop_tx(tx, resume_rx=False)
    tx_dev.stop(tx)
    problems = []
    if job.rejected:
        problems.append(f"{job.rejected} TX frames rejected")
    if got != sent:
        missing = len(sent) - len(got)
        wrong = sum(1 for a, b in zip(got, sent) if a != b)
        problems.append(f"received {len(got)}/{len(sent)}" + (f", {wrong} differ" if wrong else "") +
                        (f", first mismatch sent {sent[next(i for i, (a, b) in enumerate(zip(got, sent)) if a != b)]}"
                         if wrong else "") + (f", {missing} missing" if missing > 0 else ""))
    if delta.crc_errors or delta.sync_errors:
        problems.append(f"CRC errors {delta.crc_errors}, sync errors {delta.sync_errors}")
    return Check(name, not problems, "; ".join(problems) or f"{len(got)} frames ok",
                 {"sent": len(sent), "received": len(got), "rejected": job.rejected,
                  "crc_errors": delta.crc_errors, "sync_errors": delta.sync_errors})


def _board_checks(dev: SentUSB, tag: str = "") -> list[Check]:
    checks: list[Check] = []
    info = dev.refresh_info()
    checks.append(Check(f"{tag}board answers", True, f"{info.board}, firmware {info.firmware}",
                        info.to_dict()))

    if info.has_supply:
        try:
            s = dev.supply()
            ok = not s.fault and (s.source == "ext" or 4.4 <= s.vs_v <= 5.5)
            checks.append(Check(f"{tag}sensor supply", ok,
                                f"VS {s.vs_v:.2f} V, {s.current_ma} mA, from {s.source}" +
                                (", FAULT" if s.fault else ""), s.to_dict()))
        except SentUSBError as e:
            checks.append(Check(f"{tag}sensor supply", False, str(e)))

    if info.has_termination:
        for ch in range(1, dev.channel_count + 1):
            dev.start_rx(ch)
            results = {}
            for mode, expect in (("on", True), ("off", False), ("auto", not dev.listen_only)):
                dev.set_termination(ch, mode)
                results[mode] = dev.refresh_info().termination_on[ch - 1]
                results[mode + "_ok"] = results[mode] == expect
            ok = all(v for k, v in results.items() if k.endswith("_ok"))
            checks.append(Check(f"{tag}termination CH{ch}", ok,
                                "" if ok else f"read back {results}", results))
    return checks


def _leave_receiving(dev: SentUSB):
    for n in range(1, dev.channel_count + 1):
        dev.configure(n, SentFormat())
    dev.start_rx()


DIRECTIONS = ("both", "a-to-b", "b-to-a")


def run_selftest(dev: SentUSB, pairs=((1, 2), (3, 4)), quick: bool = False, frames: int = 25,
                 loopback: bool = True, peer: SentUSB | None = None,
                 peer_channels: tuple[int, int] = (1, 1), direction: str = "both") -> SelfTestReport:
    """Self-test of `dev`.  With `peer` (a second board, e.g. two 1-channel boards):
    both boards get the board checks, and the loopback runs between channel
    peer_channels[0] of `dev` (A) and channel peer_channels[1] of `peer` (B), wired
    together, in `direction` ("both", "a-to-b" or "b-to-a"); `pairs` is then not used."""
    if direction not in DIRECTIONS:
        raise ValueError(f"direction must be one of {', '.join(DIRECTIONS)}")
    fmts = FORMATS_QUICK if quick else FORMATS_FULL
    if peer is None:
        checks = _board_checks(dev)
    else:
        checks = _board_checks(dev, "A: ") + _board_checks(peer, "B: ")

    if loopback and peer is not None:
        a, b = peer_channels
        legs = []
        if direction in ("both", "a-to-b"):
            legs.append((dev, a, peer, b, f"A CH{a}->B CH{b}"))
        if direction in ("both", "b-to-a"):
            legs.append((peer, b, dev, a, f"B CH{b}->A CH{a}"))
        for tx_dev, tx, rx_dev, rx, label in legs:
            for fmt in fmts:
                try:
                    checks.append(loopback_pair(tx_dev, tx, rx, fmt, frames, rx_dev=rx_dev, label=label))
                except (SentUSBError, ValueError) as e:
                    checks.append(Check(f"loopback {label} {_fmt_name(fmt)}", False, str(e)))
        _leave_receiving(dev)
        _leave_receiving(peer)
    elif loopback:
        if dev.channel_count < 2:
            checks.append(Check("loopback", False,
                                "needs the 4-channel board, or a second board (--peer)"))
        else:
            for a, b in pairs:
                for tx, rx in ((a, b), (b, a)):
                    for fmt in fmts:
                        try:
                            checks.append(loopback_pair(dev, tx, rx, fmt, frames))
                        except (SentUSBError, ValueError) as e:
                            checks.append(Check(f"loopback CH{tx}->CH{rx} {fmt.tick_us:g}us", False, str(e)))
            _leave_receiving(dev)
    return SelfTestReport(checks)
