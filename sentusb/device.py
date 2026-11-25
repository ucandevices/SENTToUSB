"""SentUSB: one connection to a SENT-to-USB converter (1 or 4 channels).

Channels are numbered 1..N everywhere in this API, as on the board (SENT1..SENT4).

    from sentusb import SentUSB, SentFormat
    with SentUSB() as dev:                       # first converter found
        print(dev.info.board, dev.channel_count)
        dev.learn(1)                             # find CH1's format
        print(dev.read(1, timeout=1.0))
        result = dev.capture(2.0, decoder="mlx90377")

How the link works: the firmware answers every command with exactly one token,
in order ("\\r", "z", "\\a" = NACK, or a text reply such as "V0200").  Data
frames ("t...") are interleaved and never count as answers.  A command waits for
its token; a lost token (possible only if the device's USB buffer overflowed)
is detected by timeout and the link re-synchronised with a "V" command.
"""
from __future__ import annotations

import queue
import threading
import time
from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Callable, Iterable, Sequence

from . import protocol as P
from .protocol import (ChannelCounters, CrcMode, DeviceInfo, LearnResult, RxFrame, SentFormat,
                       SupplyStatus)


class SentUSBError(Exception):
    pass


class NackError(SentUSBError):
    pass


class NotSupportedError(SentUSBError):
    pass


# ── Ports ─────────────────────────────────────────────────────────────────────

def find_ports() -> list[dict]:
    """Serial ports; converters (USB 0483:5740) first."""
    import serial.tools.list_ports
    out = []
    for p in serial.tools.list_ports.comports():
        out.append({"port": p.device, "description": p.description or "",
                    "converter": p.vid == P.USB_VID and p.pid == P.USB_PID,
                    "serial_number": p.serial_number})
    out.sort(key=lambda d: (not d["converter"], d["port"]))
    return out


def default_port() -> str | None:
    ports = [p for p in find_ports() if p["converter"]]
    return ports[0]["port"] if ports else None


def open_transport(port: str):
    if port.startswith("emu://"):
        from .emulator import Emulator
        return Emulator.from_url(port)
    import serial
    try:
        return serial.Serial(port, 115200, timeout=0.05, write_timeout=1.0)
    except serial.SerialException as e:
        hint = ""
        if "denied" in str(e).lower() or "busy" in str(e).lower():
            hint = " (another program, e.g. sent_viewer, has the port open: close it first)"
        raise SentUSBError(f"cannot open {port}: {e}{hint}") from e


# ── Internal state ────────────────────────────────────────────────────────────

_LOST = "\x00lost"


class _Waiter:
    __slots__ = ("event", "ok", "text", "data")

    def __init__(self):
        self.event = threading.Event()
        self.ok = False
        self.text = ""
        self.data = b""


@dataclass
class _Channel:
    number: int                          # 1-based
    fmt: SentFormat = field(default_factory=SentFormat)
    rx_id: int = 0
    mode: str = "off"
    termination: str = "auto"
    counters: ChannelCounters = field(default_factory=ChannelCounters)
    latest: RxFrame | None = None
    last_rx: float = 0.0
    tx_job: "TxJob | None" = None

    @property
    def idx(self) -> int:
        return self.number - 1


class TxJob:
    """Background transmission on one channel; see SentUSB.transmit()."""

    def __init__(self, dev: "SentUSB", ch: _Channel, frames, period_s: float, count: int | None,
                 duration_s: float | None, pause_ticks: int | None, frame_s: float | None = None):
        self.channel = ch.number
        self.sent = 0
        self.rejected = 0          # NACKed twice: previous frame still on the line, or not in TX mode
        self.error: str | None = None
        self.period_s = period_s
        # Longest frame: the firmware NACKs a frame while the previous one is still on the line.
        self._frame_s = period_s if frame_s is None else frame_s
        self._dev, self._ch, self._frames = dev, ch, frames
        self._count, self._duration, self._pause = count, duration_s, pause_ticks
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"sentusb-tx{ch.number}", daemon=True)
        self._thread.start()

    def _run(self):
        start = time.perf_counter()
        nxt = start
        i = 0
        try:
            while not self._stop.is_set():
                if self._count is not None and i >= self._count:
                    break
                if self._duration is not None and time.perf_counter() - start >= self._duration:
                    break
                status, nibs = self._frames(i)
                frame = P.tx_frame(self._ch.idx, status, nibs, self._pause)
                try:
                    self._dev._command(frame)
                    self.sent += 1
                except NackError:
                    # Most likely the previous frame is still on the line (it started late
                    # because of USB or scheduling delay): retry once after a frame time.
                    if self._stop.wait(self._frame_s):
                        break
                    try:
                        self._dev._command(frame)
                        self.sent += 1
                    except NackError:
                        self.rejected += 1
                acked = time.perf_counter()         # the frame started before its answer arrived
                i += 1
                # Keep the requested rate, but never send the next frame before this one can
                # have ended: a late wake-up must not be followed by an early (NACKed) send.
                nxt = max(nxt + self.period_s, acked + self._frame_s)
                self._wait_until(nxt)
        except Exception as e:                      # link lost etc.
            self.error = str(e)

    _SPIN_S = 0.020     # Windows sleeps in 1-16 ms steps: sleep until this close, then spin

    def _wait_until(self, deadline: float):
        while not self._stop.is_set():
            left = deadline - time.perf_counter()
            if left <= 0:
                return
            if left > self._SPIN_S:
                self._stop.wait(left - self._SPIN_S)
            else:
                time.sleep(0)                       # yield to other threads, no timer granularity

    @property
    def running(self) -> bool:
        return self._thread.is_alive()

    def stop(self):
        self._stop.set()
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=2.0)

    def wait(self, timeout: float | None = None) -> bool:
        self._thread.join(timeout)
        return not self._thread.is_alive()

    def to_dict(self) -> dict:
        return {"channel": self.channel, "running": self.running, "sent": self.sent,
                "rejected": self.rejected, "period_ms": round(self.period_s * 1000, 3),
                "error": self.error}


def _frame_source(values, status: int, nibble_count: int):
    """Normalise TX values into a function i -> (status, nibbles).

    values: '123456' | [1,2,3,4,5,6] | list of those (cycled) | list of
    (status, nibbles) | callable(i) returning either form.
    """
    def norm(v):
        if isinstance(v, tuple) and len(v) == 2 and not isinstance(v[0], str) and \
                isinstance(v[1], (str, list, tuple)):
            return int(v[0]), P.parse_nibbles(v[1], nibble_count)
        return status, P.parse_nibbles(v, nibble_count)

    if callable(values):
        return lambda i: norm(values(i))
    if isinstance(values, str) or (isinstance(values, (list, tuple)) and values
                                   and all(isinstance(x, int) for x in values)):
        one = norm(values)
        return lambda i: one
    seq = [norm(v) for v in values]
    if not seq:
        raise ValueError("no TX values")
    return lambda i: seq[i % len(seq)]


# ── Device ────────────────────────────────────────────────────────────────────

class SentUSB:
    """Connection to one converter.  Thread-safe; close() or use `with`."""

    RX_HISTORY = 4096

    def __init__(self, port: str | None = None, *, listen_only: bool = False, start_rx: bool = True,
                 formats: SentFormat | dict | Sequence | None = None, timeout: float = 0.5):
        """Open the port, detect the board and (by default) start RX everywhere.

        formats: one SentFormat for every channel, or a list / {channel: SentFormat}.
        Every channel is configured on connect, because the converter cannot report
        its current format and the library must know the nibble count to decode.
        listen_only: 4-channel board, keep termination (10 k + 2.2 nF) off so a live
        ECU-sensor link is not loaded.  A sensor alone on the bench needs it off=False.
        """
        self.port = port or default_port()
        if not self.port:
            raise SentUSBError("no SENT-to-USB converter found (USB 0483:5740); pass port='COMx'")
        self.timeout = timeout
        self._tr = open_transport(self.port)
        self._lock = threading.Lock()
        self._pending: deque[_Waiter] = deque()
        self._resync: _Waiter | None = None
        self._frame_waiters: dict[int, list[_Waiter]] = {}
        self._subscribers: list[Callable[[RxFrame], None]] = []
        self._history: dict[int, deque] = {}
        self._supply: SupplyStatus | None = None
        self._supply_event = threading.Event()
        self._closed = False
        self._error: str | None = None
        self.listen_only = False
        self.channels: list[_Channel] = [_Channel(1, rx_id=P.ID_RX)]
        self._rx_map: dict[int, _Channel] = {P.ID_RX: self.channels[0]}
        self._reader = threading.Thread(target=self._read_loop, name="sentusb-reader", daemon=True)
        self._reader.start()
        try:
            self._detect()
            for ch in self.channels:
                fmt = self._format_for(formats, ch.number)
                if fmt is not None:
                    ch.fmt = fmt
                self.configure(ch.number, ch.fmt)
            if start_rx:
                self.start_rx(listen_only=listen_only)
                self.refresh_info()
        except BaseException:
            self.close(stop=False)
            raise

    # ── context / lifetime ──

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self, stop: bool = True):
        """Stop transmissions, optionally stop all channels ('C'), close the port."""
        if self._closed:
            return
        for ch in self.channels:
            if ch.tx_job:
                ch.tx_job.stop()
                ch.tx_job = None
        if stop and not self._error:
            try:
                self._command(b"C\r")
            except SentUSBError:
                pass
        self._closed = True
        try:
            self._tr.close()
        except Exception:
            pass
        self._reader.join(timeout=1.0)

    @property
    def connected(self) -> bool:
        return not self._closed and self._error is None

    # ── link layer ──

    def _read_loop(self):
        buf = b""
        while not self._closed:
            try:
                # pyserial's read(n) blocks until n bytes or the 50 ms timeout, which would delay
                # every answer by 50 ms: wait for one byte, then take whatever else is queued.
                waiting = getattr(self._tr, "in_waiting", None)
                if waiting is None:
                    chunk = self._tr.read(512)
                else:
                    chunk = self._tr.read(1)
                    if chunk:
                        n = self._tr.in_waiting
                        if n:
                            chunk += self._tr.read(n)
            except Exception as e:
                if not self._closed:
                    self._fail(f"link lost: {e}")
                return
            if not chunk:
                continue
            buf += chunk
            while True:
                ir, ia = buf.find(b"\r"), buf.find(b"\x07")
                if ir < 0 and ia < 0:
                    break
                if ia >= 0 and (ir < 0 or ia < ir):
                    head, buf = buf[:ia].strip(), buf[ia + 1:]
                    if head:
                        self._on_line(head.decode("ascii", "replace"))
                    self._on_token("\a")
                else:
                    line, buf = buf[:ir], buf[ir + 1:]
                    self._on_line(line.decode("ascii", "replace").strip("\n "))

    def _fail(self, msg: str):
        self._error = msg
        with self._lock:
            for w in self._pending:
                w.event.set()
            self._pending.clear()

    def _on_line(self, line: str):
        if line.startswith("t"):
            f = P.parse_frame(line)
            if f is not None:
                self._on_frame(*f)
                return
        self._on_token(line)

    def _on_token(self, tok: str):
        with self._lock:
            if self._resync is not None:
                if tok.startswith("V"):
                    self._resync.text = tok
                    self._resync.ok = True
                    self._resync.event.set()
                    self._resync = None
                return
            if not self._pending:
                return                       # unsolicited; ignore
            w = self._pending.popleft()
        w.ok = tok != "\a"
        w.text = tok
        w.event.set()

    def _command(self, payload: bytes, timeout: float | None = None) -> str:
        """Send one command and wait for its answer token.  NACK -> NackError."""
        if self._error:
            raise SentUSBError(self._error)
        if self._closed:
            raise SentUSBError("connection closed")
        w = _Waiter()
        with self._lock:
            self._pending.append(w)
            try:
                self._tr.write(payload)
            except Exception as e:
                self._pending.remove(w)
                raise SentUSBError(f"write failed: {e}") from e
        if not w.event.wait(timeout or self.timeout):
            self._do_resync()
            raise SentUSBError(f"no answer to {payload.decode(errors='replace').strip()!r} (link resynchronised)")
        if self._error:
            raise SentUSBError(self._error)
        if w.text == _LOST:
            raise SentUSBError(f"answer to {payload.decode(errors='replace').strip()!r} lost (link resynchronised)")
        if not w.ok:
            raise NackError(f"device refused {payload.decode(errors='replace').strip()!r}")
        return w.text

    def _do_resync(self):
        w = _Waiter()
        with self._lock:
            for p in self._pending:
                p.text = _LOST
                p.event.set()
            self._pending.clear()
            self._resync = w
            self._tr.write(b"V\r")
        if not w.event.wait(1.5):
            self._fail("device stopped answering")

    @contextmanager
    def _expect_frame(self, can_id: int):
        w = _Waiter()
        with self._lock:
            self._frame_waiters.setdefault(can_id, []).append(w)
        try:
            yield w
        finally:
            with self._lock:
                lst = self._frame_waiters.get(can_id, [])
                if w in lst:
                    lst.remove(w)

    # ── incoming frames ──

    def _on_frame(self, cid: int, data: bytes):
        with self._lock:
            waiters = self._frame_waiters.pop(cid, None)
        if waiters:
            for w in waiters:
                w.ok, w.data = True, data
                w.event.set()

        ch = self._rx_map.get(cid)
        if ch is not None:
            fr = P.decode_rx_frame(ch.number, cid, data, ch.fmt.nibbles)
            if fr is None:
                return
            ch.latest = fr
            ch.last_rx = fr.timestamp
            self._history.setdefault(ch.number, deque(maxlen=self.RX_HISTORY)).append(fr)
            for cb in list(self._subscribers):
                try:
                    cb(fr)
                except Exception:
                    pass
            return
        if cid == P.ID_SUPPLY and len(data) >= 3:
            self._supply = SupplyStatus.from_frame(data)
            self._supply_event.set()
            return
        if cid == P.ID_INFO:
            return                                   # handled by waiters
        n = len(self.channels)
        off = cid - P.ID_DIAG
        if off >= 0 and off % P.ID_STRIDE == 0 and off // P.ID_STRIDE < n and len(data) >= 8:
            c = self.channels[off // P.ID_STRIDE].counters
            c.frames = int.from_bytes(data[0:4], "little")
            c.crc_errors = data[4] | data[5] << 8
            c.sync_errors = data[6] | data[7] << 8
            return
        off = cid - P.ID_LOSS
        if off >= 0 and off % P.ID_STRIDE == 0 and off // P.ID_STRIDE < n and len(data) >= 8:
            c = self.channels[off // P.ID_STRIDE].counters
            c.lost_edges = int.from_bytes(data[0:4], "little")
            c.usb_overflows = data[4] | data[5] << 8
            c.guard_trips = data[6] | data[7] << 8

    # ── detection ──

    def _detect(self):
        self.version = self._command(b"V\r")[1:]
        self.info = self._query_info()
        if self.info is None:
            self.info = DeviceInfo(firmware=self.version, channels=1, legacy=True, modes=["off"])
        self.channels = [_Channel(i + 1, rx_id=P.ID_RX + P.ID_STRIDE * i) for i in range(self.info.channels)]
        self._rebuild_rx_map()

    def _query_info(self) -> DeviceInfo | None:
        with self._expect_frame(P.ID_INFO) as w:
            try:
                self._command(P.control_frame(0, P.CMD_INFO))
            except NackError:
                return None                  # v1.x firmware: one channel
            if not w.event.wait(self.timeout):
                raise SentUSBError("device acknowledged the info command but sent no info frame")
        return DeviceInfo.from_frame(w.data, getattr(self, "version", ""))

    def _rebuild_rx_map(self):
        self._rx_map = {ch.rx_id: ch for ch in reversed(self.channels)}

    @staticmethod
    def _format_for(formats, number: int) -> SentFormat | None:
        if formats is None:
            return None
        if isinstance(formats, SentFormat):
            return formats
        if isinstance(formats, dict):
            f = formats.get(number)
        else:
            f = formats[number - 1] if number - 1 < len(formats) else None
        if isinstance(f, dict):
            f = SentFormat.from_dict(f)
        return f

    # ── public: information ──

    @property
    def channel_count(self) -> int:
        return len(self.channels)

    def _ch(self, number: int) -> _Channel:
        if not isinstance(number, int) or not 1 <= number <= len(self.channels):
            raise ValueError(f"channel must be 1..{len(self.channels)}, not {number!r}")
        return self.channels[number - 1]

    def refresh_info(self) -> DeviceInfo:
        """Ask the board for modes, learning channel and actual termination."""
        if self.info.legacy:
            self.info.modes = [ch.mode for ch in self.channels]
            return self.info
        info = self._query_info()
        if info is not None:
            info.firmware = self.version or info.firmware
            self.info = info
            for ch, m in zip(self.channels, info.modes):
                if not (ch.tx_job and ch.tx_job.running):
                    ch.mode = m
        return self.info

    def channel_state(self, number: int) -> dict:
        ch = self._ch(number)
        d = {"channel": ch.number, "mode": ch.mode, "format": ch.fmt.to_dict(),
             "rx_can_id": f"0x{ch.rx_id:03X}", "termination_setting": ch.termination,
             "counters": ch.counters.to_dict(),
             "last_frame": ch.latest.to_dict() if ch.latest else None,
             "seconds_since_last_frame": round(time.time() - ch.last_rx, 3) if ch.last_rx else None,
             "transmit": ch.tx_job.to_dict() if ch.tx_job else None}
        if self.info.termination_on is not None:
            d["termination_on"] = self.info.termination_on[ch.idx]
        return d

    def counters(self, number: int) -> ChannelCounters:
        c = self._ch(number).counters
        return ChannelCounters(**c.to_dict())

    def supply(self, timeout: float = 1.2) -> SupplyStatus:
        """Sensor supply (4-channel board): VS, current, source, fault."""
        if not self.info.has_supply:
            raise NotSupportedError("this board does not measure the sensor supply")
        if self._supply and time.time() - self._supply.timestamp < 1.0:
            return self._supply
        self._supply_event.clear()
        if not self._supply_event.wait(timeout):
            if self._supply:
                return self._supply
            raise SentUSBError("no supply report (frame 0x60E) received")
        return self._supply

    # ── public: configuration ──

    def configure(self, number: int, fmt: SentFormat | None = None, *, rx_id: int | None = None,
                  **changes) -> SentFormat:
        """Set a channel's format (and RX CAN ID).  Keyword changes override fields,
        e.g. configure(1, nibbles=4).  Also sets the TX tick.  Ends a learn."""
        ch = self._ch(number)
        base = fmt or ch.fmt
        if changes:
            d = base.to_dict()
            d.update(changes)
            base = SentFormat.from_dict(d)
        new_id = ch.rx_id if rx_id is None else int(rx_id)
        try:
            self._command(P.config_frame(ch.idx, base, new_id))
        except NackError:
            raise NackError(f"CH{number}: format {base.to_dict()} refused by the converter") from None
        ch.fmt = base
        ch.rx_id = new_id
        self._rebuild_rx_map()
        try:
            self._command(P.tx_tick_frame(ch.idx, base.tick_us))
        except NackError:
            pass                             # TX tick outside 2..90 µs: RX still works
        if ch.mode == "learn":
            ch.mode = "off"
        return base

    def set_termination(self, number: int, mode: str):
        """'auto' (on in RX/learn, off in TX), 'on' or 'off'.  4-channel board only."""
        ch = self._ch(number)
        mode = str(mode).lower()
        if mode not in P.TERMINATION_VALUES:
            raise ValueError("termination must be 'auto', 'on' or 'off'")
        if not self.info.has_termination:
            raise NotSupportedError("this board has no switchable termination")
        self._command(P.control_frame(ch.idx, P.CMD_TERM, bytes([P.TERMINATION_VALUES[mode]])))
        ch.termination = mode

    def set_listen_only(self, enabled: bool):
        """Restart RX on every channel with AUTO termination kept off (True) or normal (False)."""
        self.start_rx(listen_only=enabled)

    # ── public: modes ──

    def start_rx(self, number: int | None = None, *, listen_only: bool | None = None):
        """RX on one channel, or on all (number=None, 'O'/'L').  Stops TX on those channels."""
        if number is None:
            for ch in self.channels:
                self._stop_job(ch)
            lo = self.listen_only if listen_only is None else bool(listen_only)
            if lo and not self.info.has_termination:
                lo = False                   # nothing to switch off on this board
            self._command(b"L\r" if lo else b"O\r")
            self.listen_only = lo
            for ch in self.channels:
                ch.mode = "rx"
            return
        ch = self._ch(number)
        self._stop_job(ch)
        self._command(P.control_frame(ch.idx, P.CMD_RX))
        ch.mode = "rx"

    def stop(self, number: int | None = None):
        """Stop one channel, or all ('C')."""
        if number is None:
            for ch in self.channels:
                self._stop_job(ch)
            self._command(b"C\r")
            for ch in self.channels:
                ch.mode = "off"
            return
        ch = self._ch(number)
        self._stop_job(ch)
        self._command(P.control_frame(ch.idx, P.CMD_STOP))
        ch.mode = "off"

    # ── public: learn ──

    def learn(self, number: int, timeout: float = 5.0, apply: bool = True) -> LearnResult:
        """Find the format of the sensor on a channel (tick, nibbles, CRC variant).

        Only one channel can learn at a time.  With apply=True the learned format
        is configured and RX restarted (the firmware stops the channel after a learn).
        """
        ch = self._ch(number)
        self._stop_job(ch)
        with self._expect_frame(P.ID_LEARN + ch.idx) as w:
            try:
                self._command(P.control_frame(ch.idx, P.CMD_LEARN))
            except NackError:
                busy = self.refresh_info().learning_channel
                raise NackError(f"learn refused on CH{number}" +
                                (f": CH{busy} is learning" if busy else "")) from None
            ch.mode = "learn"
            if not w.event.wait(timeout):
                self.configure(number)            # ends the learn, restores the format
                self.start_rx(number)
                raise TimeoutError(
                    f"CH{number}: no SENT format recognised in {timeout:g} s. Check that the sensor is "
                    "connected and powered (supply), and on the 4-channel board that termination is not "
                    "forced off / listen-only is off for a sensor alone on the bench.")
        result = LearnResult.from_frame(w.data)
        ch.mode = "off"
        if apply:
            self.configure(number, result.to_format())
            self.start_rx(number)
        return result

    # ── public: receive ──

    def latest(self, number: int) -> RxFrame | None:
        return self._ch(number).latest

    def history(self, number: int, n: int | None = None) -> list[RxFrame]:
        h = list(self._history.get(self._ch(number).number, ()))
        return h[-n:] if n else h

    def subscribe(self, callback: Callable[[RxFrame], None]):
        """callback(frame) for every received frame, called on the reader thread: keep it short."""
        self._subscribers.append(callback)

    def unsubscribe(self, callback):
        if callback in self._subscribers:
            self._subscribers.remove(callback)

    @contextmanager
    def stream(self, channels: Iterable[int] | int | None = None, maxsize: int = 0):
        """Context manager giving a queue.Queue of RxFrames from the given channels."""
        if isinstance(channels, int):
            channels = [channels]
        wanted = None if channels is None else {self._ch(c).number for c in channels}
        q: queue.Queue = queue.Queue(maxsize)

        def cb(fr: RxFrame):
            if wanted is None or fr.channel in wanted:
                try:
                    q.put_nowait(fr)
                except queue.Full:
                    pass
        self.subscribe(cb)
        try:
            yield q
        finally:
            self.unsubscribe(cb)

    def read(self, number: int, timeout: float = 1.0) -> RxFrame | None:
        """Next frame received on a channel after this call, or None on timeout."""
        with self.stream(number) as q:
            try:
                return q.get(timeout=timeout)
            except queue.Empty:
                return None

    def capture(self, duration_s: float, channels: Iterable[int] | None = None, *,
                decoder=None, csv_path: str | None = None):
        """Record frames for duration_s and return a CaptureResult with per-channel
        statistics (see sentusb.capture).  decoder: name, or {channel: name}."""
        from .capture import run_capture
        return run_capture(self, duration_s, channels, decoder=decoder, csv_path=csv_path)

    # ── public: transmit ──

    def _check_tx_allowed(self, ch: _Channel, force: bool):
        if force or ch.mode == "tx":
            return
        if ch.mode not in ("rx", "learn"):
            return
        recent = ch.last_rx and time.time() - ch.last_rx < 0.5
        before = (ch.counters.sync_errors, ch.counters.crc_errors)
        heard = recent or self.read(ch.number, timeout=0.15) is not None   # listen briefly first
        # A sensor in another format shows up as sync / CRC errors, not frames.
        heard = heard or (ch.counters.sync_errors, ch.counters.crc_errors) != before
        if heard:
            raise SentUSBError(
                f"CH{ch.number} is receiving SENT frames from a sensor right now; transmitting would "
                "drive against it. Disconnect the sensor, or pass force=True.")

    def _enter_tx(self, ch: _Channel, force: bool):
        if ch.mode == "tx":
            return
        self._check_tx_allowed(ch, force)
        self._command(P.control_frame(ch.idx, P.CMD_TX))
        ch.mode = "tx"

    def frame_duration_s(self, number: int, status: int, nibbles, pause_ticks: int | None = None) -> float:
        ch = self._ch(number)
        return P.frame_ticks(status, nibbles, pause_ticks) * ch.fmt.tick_us * 1e-6

    def send(self, number: int, nibbles, status: int = 0, *, pause_ticks: int | None = None,
             force: bool = False):
        """Send one frame.  The channel switches to TX and stays there (start_rx() to go back)."""
        ch = self._ch(number)
        self._stop_job(ch)
        nibs = P.parse_nibbles(nibbles, ch.fmt.nibbles)
        self._enter_tx(ch, force)
        frame = P.tx_frame(ch.idx, status, nibs, pause_ticks)
        try:
            self._command(frame)
        except NackError:                         # previous frame still on the line
            time.sleep(self.frame_duration_s(number, status, nibs, pause_ticks) + 0.002)
            self._command(frame)

    def transmit(self, number: int, values, *, status: int = 0, period_ms: float | None = None,
                 count: int | None = None, duration_s: float | None = None,
                 pause_ticks: int | None = None, force: bool = False) -> TxJob:
        """Transmit frames in the background (the firmware has no TX queue, so the
        host paces them).  values: '123456', a list of such values (cycled), a list of
        (status, nibbles), or a callable i -> value.  period_ms defaults to the
        frame length + 0.5 ms; shorter than the frame is refused.  Returns a TxJob."""
        ch = self._ch(number)
        self._stop_job(ch)
        source = _frame_source(values, status, ch.fmt.nibbles)
        st0, nib0 = source(0)
        longest = P.frame_ticks(15, [15] * ch.fmt.nibbles, pause_ticks) * ch.fmt.tick_us * 1e-6
        if period_ms is None:
            period = longest + 0.0005
        else:
            period = float(period_ms) / 1000.0
            if period < longest:
                raise ValueError(f"period {period_ms} ms is shorter than a CH{number} frame "
                                 f"({longest * 1000:.3f} ms at {ch.fmt.tick_us:g} µs tick)")
        self._enter_tx(ch, force)
        ch.tx_job = TxJob(self, ch, source, period, count, duration_s, pause_ticks, frame_s=longest)
        return ch.tx_job

    def _stop_job(self, ch: _Channel):
        if ch.tx_job:
            ch.tx_job.stop()
            ch.tx_job = None

    def stop_tx(self, number: int, resume_rx: bool = True) -> dict | None:
        """Stop a background transmission; back to RX unless resume_rx=False."""
        ch = self._ch(number)
        job = ch.tx_job
        self._stop_job(ch)
        if resume_rx:
            self.start_rx(number)
        return job.to_dict() if job else None

    def to_dict(self) -> dict:
        return {"port": self.port, "listen_only": self.listen_only, **self.info.to_dict()}
