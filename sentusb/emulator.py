"""Software model of a SENT-to-USB converter, used as port 'emu://...'.

Lets the CLI, the MCP server and the tests run without hardware.  It follows the
firmware's command rules (Core/Src/sent_core.c, sent_bridge.c): one answer token per
command, NACK cases, shared learn, TX NACKed while the previous frame is still on
the line, RX only when the configured format matches the sensor (else sync / CRC
errors are counted), diagnostics every 100 ms when changed, supply every 500 ms.
It produces decoded frames, not edges: timing and analog effects are not modelled.

URLs:
  emu://4ch                 4 channels, V0200; sensors: CH1 MLX90377-like (H.4),
                            CH2 dual 12-bit pressure (H.1, status+data CRC, seed 5),
                            CH3 MLX90377-like, CH4 nothing connected
  emu://4ch?loop=12,34      CH1<->CH2 and CH3<->CH4 wired together, no sensors on them
  emu://4ch?sensors=1:mlx,4:h1   choose sensors (mlx, h1, none)
  emu://1ch                 1-channel SENTToUSB with v1.1 firmware (V0102)
  ?wire=A                   two-board loopback: every emulator opened with the same
                            wire name has its CH1 on one shared wire (wire=A:2 uses CH2);
                            e.g. emu://1ch?wire=A and emu://1ch?wire=A, no sensor on it
  &rate=0.2                 scale the sensors' frame rate (default 1.0, about 1.2 kHz)
"""
from __future__ import annotations

import queue
import random
import threading
import time
from dataclasses import dataclass, field
from urllib.parse import urlparse, parse_qs

from . import protocol as P

# Wires between emulators (two-board loopback): name -> [(emulator, channel index)].
_WIRES: dict[str, list[tuple["Emulator", int]]] = {}
_WIRES_LOCK = threading.Lock()


@dataclass
class _Sensor:
    kind: str                      # "mlx" or "h1"
    tick_us: float
    nibbles: int
    crc_mode: int                  # 0 data only, 1 status+data
    seed: int
    legacy: bool = False
    i: int = 0

    def next(self) -> tuple[int, tuple[int, ...], tuple | None]:
        self.i += 1
        if self.kind == "mlx":
            angle = (self.i * 3) % 4096
            cnt = self.i % 256
            n0 = angle >> 8
            nibs = (n0, (angle >> 4) & 15, angle & 15, cnt >> 4, cnt & 15, 15 - n0)
        else:
            s1 = 1500 + (self.i % 40)
            s2 = 4095 - s1
            nibs = (s1 >> 8, (s1 >> 4) & 15, s1 & 15, s2 & 15, (s2 >> 4) & 15, s2 >> 8)
        slow = ("short", 0x03, 0x42) if self.i % 32 == 0 else None
        return 0, nibs, slow

    def period_s(self) -> float:
        return (56 + 12 + self.nibbles * 19 + 20 + 12) * self.tick_us * 1e-6


def _make_sensor(kind: str) -> _Sensor | None:
    if kind == "mlx":
        return _Sensor("mlx", 3.0, 6, 0, 0x03)
    if kind == "h1":
        return _Sensor("h1", 3.0, 6, 1, 0x05)
    return None


@dataclass
class _Ch:
    mode: str = "off"
    nibbles: int = 6
    crc_mode: int = 0
    seed: int = 3
    legacy: bool = False
    min_x10: int = 24
    max_x10: int = 45
    out_id: int = 0x510
    tx_tick_x10: int = 30
    term: int = 2
    frames: int = 0
    crc: int = 0
    sync: int = 0
    lost: int = 0
    last_diag: tuple = ()
    last_loss: tuple = ()
    busy_until: float = 0.0
    sensor: _Sensor | None = None
    next_frame: float = 0.0
    saved: dict = field(default_factory=dict)
    learn_since: float = 0.0


class Emulator:
    def __init__(self, channels: int = 4, legacy: bool = False, sensors: dict | None = None,
                 loop: list[tuple[int, int]] | None = None, rate: float = 1.0,
                 wire: tuple[str, int] | None = None):
        self.nch = channels
        self.legacy = legacy
        self.version = "0102" if legacy else "0200"
        self.rate = rate
        self.chs = [_Ch(out_id=P.ID_RX + P.ID_STRIDE * i) for i in range(channels)]
        self.peer: dict[int, int] = {}
        for a, b in loop or []:
            self.peer[a - 1], self.peer[b - 1] = b - 1, a - 1
        self._wire = (wire[0], wire[1] - 1) if wire else None
        if self._wire and not 0 <= self._wire[1] < channels:
            raise ValueError(f"wire channel {wire[1]} does not exist on a {channels}-channel board")
        self._inbox: queue.SimpleQueue = queue.SimpleQueue()   # frames from other emulators
        if sensors is None:
            sensors = {1: "mlx", 2: "h1", 3: "mlx"} if channels == 4 else {1: "mlx"}
        for n, kind in sensors.items():
            if 1 <= n <= channels and (n - 1) not in self.peer and                     not (self._wire and self._wire[1] == n - 1):
                self.chs[n - 1].sensor = _make_sensor(kind)
        self.listen_only = False
        self.learn_owner: int | None = None
        self._out = bytearray()
        self._inbuf = b""
        self._lock = threading.Lock()
        self._deliveries: list[tuple[float, int, int, tuple, tuple]] = []
        self.is_open = True
        self.commands: list[str] = []
        self.drop_next_answers = 0          # test hook: lose answer tokens
        self._t_diag = self._t_supply = time.monotonic()
        if self._wire:
            with _WIRES_LOCK:
                _WIRES.setdefault(self._wire[0], []).append((self, self._wire[1]))
        threading.Thread(target=self._run, daemon=True, name="sentusb-emulator").start()

    @classmethod
    def from_url(cls, url: str) -> "Emulator":
        u = urlparse(url)
        kind = (u.netloc or u.path.strip("/")).lower()
        q = parse_qs(u.query)
        loop = None
        if "loop" in q:
            loop = [(int(p[0]), int(p[1])) for p in q["loop"][0].split(",") if len(p) == 2]
        sensors = None
        if "sensors" in q:
            sensors = {int(k): v for k, v in (s.split(":") for s in q["sensors"][0].split(",") if s)}
        rate = float(q.get("rate", ["1.0"])[0])
        wire = None
        if "wire" in q:
            name, _, n = q["wire"][0].partition(":")
            wire = (name, int(n or 1))
        if kind in ("1ch", "legacy"):
            return cls(1, legacy=True, sensors=sensors, rate=rate, wire=wire)
        if kind in ("4ch", ""):          # looped and wired channels get no sensor
            return cls(4, sensors=sensors, loop=loop, rate=rate, wire=wire)
        raise ValueError(f"unknown emulator {url!r}: use emu://4ch or emu://1ch")

    # ── serial-like API ──

    def write(self, data: bytes) -> int:
        self._inbuf += data
        while b"\r" in self._inbuf:
            line, self._inbuf = self._inbuf.split(b"\r", 1)
            with self._lock:
                self._command(line.decode("ascii", "replace"))
        return len(data)

    def read(self, n: int) -> bytes:
        deadline = time.monotonic() + 0.05
        while True:
            if not self.is_open:
                raise OSError("port closed")
            with self._lock:
                if self._out:
                    d = bytes(self._out[:n])
                    del self._out[:n]
                    return d
            if time.monotonic() >= deadline:
                return b""
            time.sleep(0.002)

    def close(self):
        self.is_open = False
        if self._wire:
            with _WIRES_LOCK:
                peers = _WIRES.get(self._wire[0], [])
                peers[:] = [(e, i) for e, i in peers if e is not self]
                if not peers:
                    _WIRES.pop(self._wire[0], None)

    def reset_input_buffer(self):
        with self._lock:
            self._out.clear()

    # ── output ──

    def _emit(self, s: str):
        self._out += s.encode("ascii")

    def _answer(self, s: str):
        if self.drop_next_answers > 0:
            self.drop_next_answers -= 1
            return
        self._emit(s)

    def _frame(self, cid: int, data):
        self._emit(f"t{cid:03X}{len(data):X}{bytes(data).hex().upper()}\r")

    # ── commands ──

    def _command(self, line: str):
        self.commands.append(line)
        if not line:
            return
        c = line[0]
        if c in "Vv":
            return self._answer(f"{c}{self.version}\r")
        if c == "N":
            return self._answer("N1234\r")
        if c == "F":
            return self._answer("F00\r")
        if c in "Ss":
            return self._answer("\r")
        if c == "O" or (c == "L" and not self.legacy):
            self.listen_only = c == "L"
            self._end_learn()
            for ch in self.chs:
                ch.mode = "rx"
            return self._answer("\r")
        if c == "L":
            return self._answer("\r")            # v1.1: not implemented, plain ACK
        if c == "C":
            self.listen_only = False
            self._end_learn()
            for ch in self.chs:
                ch.mode = "off"
            return self._answer("\r")
        if c == "t":
            f = P.parse_frame(line)
            if f is None:
                return self._answer("\a")
            return self._answer("z\r" if self._frame_cmd(*f) else "\a")
        self._answer("\a")

    def _end_learn(self):
        if self.learn_owner is not None:
            ch = self.chs[self.learn_owner]
            for k, v in ch.saved.items():
                setattr(ch, k, v)
            if ch.mode == "learn":
                ch.mode = "off"
            self.learn_owner = None

    def _frame_cmd(self, cid: int, d: bytes) -> bool:
        if P.ID_CONFIG <= cid < P.ID_CONFIG + self.nch:
            ch_i = cid - P.ID_CONFIG
            ch = self.chs[ch_i]
            if self.learn_owner == ch_i:
                self._end_learn()
            if len(d) >= 1 and 1 <= d[0] <= 8:
                ch.nibbles = d[0]
            if len(d) >= 2:
                ch.crc_mode = 1 if d[1] else 0
            if len(d) >= 3 and d[2]:
                ch.legacy = bool(d[2] & 0x80)
                if d[2] & 0x0F:
                    ch.seed = d[2] & 0x0F
            if len(d) >= 4 and d[3]:
                ch.min_x10 = d[3] * 5
            if len(d) >= 5 and d[4]:
                ch.max_x10 = d[4] * 10
            if len(d) >= 7:
                rid = d[5] << 8 | d[6]
                if 1 <= rid <= 0x7FF:
                    ch.out_id = rid
            return ch.min_x10 <= ch.max_x10
        if cid == P.ID_CONTROL and d:
            ch_i, cmd = d[0] >> 4, d[0] & 0x0F
            if self.legacy:
                if d[0] > 0x0F or cmd in (P.CMD_INFO, P.CMD_TERM):
                    return False
            elif cmd == P.CMD_INFO:
                self._info_pending = True
                return True
            if ch_i >= self.nch:
                return False
            ch = self.chs[ch_i]
            if cmd == P.CMD_RX:
                if self.learn_owner == ch_i:
                    self._end_learn()
                ch.mode = "rx"
                return True
            if cmd == P.CMD_TX:
                if self.learn_owner == ch_i:
                    self._end_learn()
                ch.mode = "tx"
                return True
            if cmd == P.CMD_STOP:
                if self.learn_owner == ch_i:
                    self._end_learn()
                ch.mode = "off"
                return True
            if cmd == P.CMD_LEARN:
                if self.learn_owner is not None and self.learn_owner != ch_i:
                    return False
                if self.learn_owner is None:
                    ch.saved = {k: getattr(ch, k) for k in ("nibbles", "crc_mode", "seed", "legacy",
                                                            "min_x10", "max_x10")}
                self.learn_owner = ch_i
                ch.mode = "learn"
                ch.learn_since = time.monotonic()
                return True
            if cmd == P.CMD_TX_TICK:
                if len(d) < 3:
                    return False
                t = d[1] | d[2] << 8
                if not 20 <= t <= 900:
                    return False
                ch.tx_tick_x10 = t
                return True
            if cmd == P.CMD_TERM:
                if len(d) < 2 or d[1] > 2:
                    return False
                ch.term = d[1]
                return True
            return False
        for i, ch in enumerate(self.chs):
            if cid == P.ID_TX + P.ID_STRIDE * i:
                if ch.mode != "tx" or len(d) < 5:
                    return False
                now = time.monotonic()
                if now < ch.busy_until:
                    return False                      # previous frame still on the line
                status = d[0] & 0x0F
                packed = int.from_bytes(d[1:5], "big")
                nibs = tuple((packed >> (4 * (ch.nibbles - 1 - k))) & 0x0F for k in range(ch.nibbles))
                pause = (d[5] | d[6] << 8) if len(d) >= 7 else None
                dur = P.frame_ticks(status, nibs, pause) * ch.tx_tick_x10 * 1e-7
                ch.busy_until = now + dur
                sender = (ch.tx_tick_x10, ch.nibbles, ch.crc_mode, ch.seed, ch.legacy)
                if i in self.peer:
                    self._deliveries.append((now + dur, self.peer[i], status, nibs, sender))
                if self._wire and self._wire[1] == i:
                    with _WIRES_LOCK:
                        others = [(e, j) for e, j in _WIRES.get(self._wire[0], []) if e is not self]
                    for e, j in others:     # no lock on the other emulator: it drains its inbox
                        e._inbox.put((now + dur, j, status, nibs, sender))
                return True
        return True                                   # unknown ID: accepted, ignored

    # The info frame follows the 'z' of command 0x06 (sent_core.c order).
    _info_pending = False

    def _info_frame(self):
        modes = 0
        for i, ch in enumerate(self.chs):
            modes |= {"off": 0, "rx": 1, "tx": 2, "learn": 3}[ch.mode] << (2 * i)
        term = 0
        for i, ch in enumerate(self.chs):
            auto_on = ch.mode in ("rx", "learn") and not self.listen_only
            on = ch.term == 1 or (ch.term == 2 and auto_on)
            term |= (1 << i) if on else 0
            term |= (1 << (4 + i)) if ch.term == 2 else 0
        learn = 0xFF if self.learn_owner is None else self.learn_owner
        self._frame(P.ID_INFO, [self.nch, 0x02, 0x00, modes, learn, term, 0x07, 0])

    # ── background: sensors, deliveries, diagnostics ──

    def _matches(self, ch: _Ch, tick_x10: int, nibbles: int, crc_mode: int, seed: int, legacy: bool) -> str:
        if not (ch.min_x10 <= tick_x10 <= ch.max_x10) or ch.nibbles != nibbles:
            return "sync"
        if (ch.crc_mode, ch.seed, ch.legacy) != (crc_mode, seed, legacy):
            return "crc"
        return "ok"

    def _rx_frame(self, i: int, status: int, nibs: tuple, slow=None):
        ch = self.chs[i]
        n_bytes = (len(nibs) + 1) // 2
        info_idx = max(3, n_bytes)
        data = [0] * (info_idx + 1)
        padded = list(nibs) + [0]
        for b in range(n_bytes):
            data[b] = padded[2 * b] << 4 | padded[2 * b + 1]
        data[info_idx] = status << 4
        if slow:
            data[info_idx] |= P.SLOW_FLAG_NEW
            data += [slow[1], slow[2] & 0xFF, slow[2] >> 8]
        ch.frames += 1
        self._frame(ch.out_id, data)

    def _run(self):
        while self.is_open:
            time.sleep(0.005)
            now = time.monotonic()
            with self._lock:
                if self._info_pending:
                    self._info_pending = False
                    self._info_frame()
                self._sensors(now)
                while True:
                    try:
                        self._deliveries.append(self._inbox.get_nowait())
                    except queue.Empty:
                        break
                self._deliver(now)
                if now - self._t_diag >= 0.1:
                    self._t_diag = now
                    self._diag()
                if not self.legacy and now - self._t_supply >= 0.5:
                    self._t_supply = now
                    active = sum(1 for c in self.chs if c.sensor)
                    mv = 5020 + random.randint(-8, 8)
                    ma = 2 + 8 * active
                    self._frame(P.ID_SUPPLY, [mv & 0xFF, mv >> 8, 0x01, ma & 0xFF, ma >> 8, 0])

    def _sensors(self, now: float):
        for i, ch in enumerate(self.chs):
            s = ch.sensor
            if s is None:
                continue
            if ch.mode == "learn" and self.learn_owner == i and now - ch.learn_since > 0.3:
                tick_x10 = int(round(s.tick_us * 10))
                margin = tick_x10 * 20 // 100
                ch.nibbles, ch.crc_mode, ch.seed, ch.legacy = s.nibbles, s.crc_mode, s.seed, s.legacy
                ch.min_x10, ch.max_x10 = tick_x10 - margin, tick_x10 + margin
                ch.mode = "off"
                self.learn_owner = None
                self._frame(P.ID_LEARN + i, [tick_x10 & 0xFF, tick_x10 >> 8, s.nibbles, s.crc_mode,
                                             s.seed | (0x80 if s.legacy else 0)])
                continue
            period = s.period_s() / max(self.rate, 1e-3)
            if ch.next_frame == 0.0 or ch.mode != "rx":
                ch.next_frame = now + period
                continue
            k = 0
            while ch.next_frame <= now and k < 64:
                ch.next_frame += period
                k += 1
            if ch.next_frame < now:
                ch.next_frame = now + period
            if not k:
                continue
            m = self._matches(ch, int(round(s.tick_us * 10)), s.nibbles, s.crc_mode, s.seed, s.legacy)
            for _ in range(k):
                status, nibs, slow = s.next()
                if m == "ok":
                    self._rx_frame(i, status, nibs, slow)
                elif m == "crc":
                    ch.crc += 1
                else:
                    ch.sync += 1

    def _deliver(self, now: float):
        due = [d for d in self._deliveries if d[0] <= now]
        self._deliveries = [d for d in self._deliveries if d[0] > now]
        for _, rx_i, status, nibs, sender in due:
            ch = self.chs[rx_i]
            if ch.mode != "rx":
                continue
            m = self._matches(ch, *sender)
            if m == "ok":
                self._rx_frame(rx_i, status, nibs)
            elif m == "crc":
                ch.crc += 1
            else:
                ch.sync += 1

    def _diag(self):
        for i, ch in enumerate(self.chs):
            cur = (ch.frames, ch.crc, ch.sync)
            if cur != ch.last_diag:
                ch.last_diag = cur
                self._frame(P.ID_DIAG + P.ID_STRIDE * i,
                            list(ch.frames.to_bytes(4, "little")) +
                            [ch.crc & 0xFF, ch.crc >> 8 & 0xFF, ch.sync & 0xFF, ch.sync >> 8 & 0xFF])
