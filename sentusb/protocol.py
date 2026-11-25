"""Wire protocol of the ucandevices SENT-to-USB converters.

SLCAN-style text over USB CDC.  Every SENT channel is addressed through
CAN-like frames; channel n is 0-based here (the public API is 1-based).

  config        t001+n     nibbles, CRC mode, CRC selector, tick window, RX ID
  control       t600       data[0] = n << 4 | command
  TX frame      t520+0x40n status, nibbles right-aligned in 32 bits, [pause LE16]
  RX frame      t510+0x40n by default (configurable)
  diagnostics   t511+0x40n frames LE32, CRC errors LE16, sync errors LE16
  losses        t512+0x40n lost edges LE32, USB overflows LE16, guard trips LE16
  learn result  t601+n     tick x0.1 us LE16, nibbles, CRC mode, CRC selector
  device info   t60F       answer to control command 0x06 (4-channel firmware)
  supply        t60E       every 500 ms (4-channel board)

Source of truth: SENT_4chanel/firmware Core/Inc/sent_bridge.h, Core/Src/sent_core.c
and sent_bridge.c (pack_rx_can_frame, handle_tx_can_frame).
"""
from __future__ import annotations

import re
import time
from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import Sequence

MAX_CHANNELS = 4
ID_STRIDE = 0x40
ID_CONFIG = 0x001
ID_CONTROL = 0x600
ID_LEARN = 0x601
ID_RX = 0x510
ID_DIAG = 0x511
ID_LOSS = 0x512
ID_TX = 0x520
ID_INFO = 0x60F
ID_SUPPLY = 0x60E

CMD_RX, CMD_TX, CMD_STOP, CMD_LEARN, CMD_TX_TICK, CMD_INFO, CMD_TERM = range(1, 8)

FEATURE_TERMINATION = 0x01
FEATURE_SUPPLY = 0x02
FEATURE_PUSH_PULL = 0x04

MODE_NAMES = ("off", "rx", "tx", "learn")
TERMINATION_VALUES = {"off": 0, "on": 1, "auto": 2}

USB_VID, USB_PID = 0x0483, 0x5740

TX_MIN_TICK_US, TX_MAX_TICK_US = 2.0, 90.0
DEFAULT_PAUSE_TICKS = 12          # firmware pause when a TX frame carries none
SYNC_TICKS = 56

SLOW_FLAG_NEW, SLOW_FLAG_ENHANCED, SLOW_FLAG_16BIT = 0x01, 0x02, 0x04


class CrcMode(str, Enum):
    DATA_ONLY = "data_only"
    STATUS_AND_DATA = "status_and_data"

    @classmethod
    def parse(cls, value) -> "CrcMode":
        if isinstance(value, CrcMode):
            return value
        v = str(value).strip().lower().replace("-", "_").replace(" ", "_")
        aliases = {"data": cls.DATA_ONLY, "data_only": cls.DATA_ONLY, "0": cls.DATA_ONLY,
                   "status": cls.STATUS_AND_DATA, "status_and_data": cls.STATUS_AND_DATA,
                   "status_data": cls.STATUS_AND_DATA, "1": cls.STATUS_AND_DATA}
        if v not in aliases:
            raise ValueError(f"CRC mode {value!r}: use 'data_only' or 'status_and_data'")
        return aliases[v]


@dataclass
class SentFormat:
    """Format of a SENT stream: what the converter needs to decode or send it.

    crc_seed 0x03 is the J2716 recommended start value, 0x05 the variant used by
    several VW/GM sensors.  legacy_crc selects the pre-2010 (FEB2008) CRC.
    """
    tick_us: float = 3.0
    nibbles: int = 6
    crc_mode: CrcMode = CrcMode.DATA_ONLY
    crc_seed: int = 0x03
    legacy_crc: bool = False

    def __post_init__(self):
        self.crc_mode = CrcMode.parse(self.crc_mode)
        self.tick_us = float(self.tick_us)
        self.nibbles = int(self.nibbles)
        self.crc_seed = int(self.crc_seed, 0) if isinstance(self.crc_seed, str) else int(self.crc_seed)
        self.validate()

    def validate(self):
        if not 1 <= self.nibbles <= 8:
            raise ValueError(f"nibbles must be 1..8, not {self.nibbles}")
        if not 1.0 <= self.tick_us <= 150.0:
            raise ValueError(f"tick_us must be 1..150 µs, not {self.tick_us}")
        if not 1 <= self.crc_seed <= 15:
            raise ValueError(f"crc_seed must be 1..15 (0x03 recommended, 0x05 variant), not {self.crc_seed}")

    @property
    def crc_selector(self) -> int:
        return (self.crc_seed & 0x0F) | (0x80 if self.legacy_crc else 0)

    def to_dict(self) -> dict:
        d = asdict(self)
        d["crc_mode"] = self.crc_mode.value
        d["crc_seed"] = f"0x{self.crc_seed:02X}"
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "SentFormat":
        keys = ("tick_us", "nibbles", "crc_mode", "crc_seed", "legacy_crc")
        return cls(**{k: d[k] for k in keys if k in d and d[k] is not None})


# ── Frame text ────────────────────────────────────────────────────────────────

_FRAME_RE = re.compile(r'^t([0-9A-Fa-f]{3})([0-9A-Fa-f])([0-9A-Fa-f]*)$')


def encode_frame(can_id: int, data: bytes) -> bytes:
    if not 0 <= can_id <= 0x7FF or len(data) > 8:
        raise ValueError("bad frame")
    return f"t{can_id:03X}{len(data):X}{bytes(data).hex().upper()}\r".encode("ascii")


def parse_frame(line: str):
    """'tIIIDdd..' -> (id, data) or None."""
    m = _FRAME_RE.match(line)
    if not m:
        return None
    dlc = int(m.group(2), 16)
    if dlc > 8 or len(m.group(3)) < dlc * 2:
        return None
    return int(m.group(1), 16), bytes.fromhex(m.group(3)[:dlc * 2])


def control_frame(ch0: int, cmd: int, extra: bytes = b"") -> bytes:
    return encode_frame(ID_CONTROL, bytes([(ch0 << 4) | cmd]) + bytes(extra))


def config_frame(ch0: int, fmt: SentFormat, rx_id: int) -> bytes:
    """Config frame 0x001+n.  Tick window 80 %..150 % of the tick, as the viewer."""
    min_b = min(255, max(1, round(fmt.tick_us * 0.8 / 0.5)))   # units of 0.5 µs
    max_b = min(255, max(1, int(fmt.tick_us * 1.5 + 0.5)))     # units of 1 µs
    if not 1 <= rx_id <= 0x7FF:
        raise ValueError(f"RX CAN ID 0x{rx_id:X} outside 0x001..0x7FF")
    data = bytes([fmt.nibbles, 1 if fmt.crc_mode is CrcMode.STATUS_AND_DATA else 0,
                  fmt.crc_selector, min_b, max_b, rx_id >> 8, rx_id & 0xFF])
    return encode_frame(ID_CONFIG + ch0, data)


def tx_tick_frame(ch0: int, tick_us: float) -> bytes:
    t = max(20, min(int(round(tick_us * 10)), 900))
    return control_frame(ch0, CMD_TX_TICK, bytes([t & 0xFF, t >> 8]))


def parse_nibbles(value, count: int | None = None) -> tuple[int, ...]:
    """'1A3', [1, 10, 3] or (1, 10, 3) -> (1, 10, 3)."""
    if isinstance(value, str):
        s = value.strip().replace(" ", "").replace("_", "")
        if s.lower().startswith("0x"):
            s = s[2:]
        try:
            nibs = tuple(int(c, 16) for c in s)
        except ValueError:
            raise ValueError(f"nibbles {value!r}: hex digits only, e.g. '123456'") from None
    else:
        nibs = tuple(int(n) for n in value)
    if not nibs or len(nibs) > 8 or any(not 0 <= n <= 15 for n in nibs):
        raise ValueError(f"nibbles {value!r}: 1..8 values of 0..15")
    if count is not None and len(nibs) != count:
        raise ValueError(f"{len(nibs)} nibbles given, the channel is configured for {count}")
    return nibs


def tx_frame(ch0: int, status: int, nibbles: Sequence[int], pause_ticks: int | None = None) -> bytes:
    """TX frame 0x520+0x40n: the firmware unpacks the low `nibbles` nibbles of a
    big-endian 32-bit word, MSB first (sent_unpack_nibbles)."""
    if not 0 <= status <= 15:
        raise ValueError("status must be 0..15")
    packed = 0
    for n in nibbles:
        packed = (packed << 4) | (n & 0x0F)
    data = bytes([status & 0x0F]) + packed.to_bytes(4, "big")
    if pause_ticks is not None:
        if not 0 <= pause_ticks <= 0xFFFF:
            raise ValueError("pause_ticks must be 0..65535")
        data += pause_ticks.to_bytes(2, "little")
    return encode_frame(ID_TX + ID_STRIDE * ch0, data)


def frame_ticks(status: int, nibbles: Sequence[int], pause_ticks: int | None = None) -> int:
    """Length of one transmitted frame in ticks (CRC nibble counted at its maximum)."""
    pause = DEFAULT_PAUSE_TICKS if not pause_ticks else pause_ticks
    return SYNC_TICKS + (12 + status) + sum(12 + n for n in nibbles) + 27 + pause


# ── Decoded device data ──────────────────────────────────────────────────────

@dataclass
class SlowMessage:
    kind: str          # "short", "enhanced" (12-bit data, 8-bit ID), "enhanced16" (16-bit data, 4-bit ID)
    id: int
    value: int


@dataclass
class RxFrame:
    channel: int                  # 1-based
    can_id: int
    timestamp: float
    status: int
    nibbles: tuple[int, ...]
    slow: SlowMessage | None = None
    raw: bytes = b""

    @property
    def hex(self) -> str:
        return "".join(f"{n:X}" for n in self.nibbles)

    def to_dict(self) -> dict:
        d = {"channel": self.channel, "time": round(self.timestamp, 4),
             "status": self.status, "nibbles": self.hex}
        if self.slow:
            d["slow"] = asdict(self.slow)
        return d


def decode_rx_frame(channel: int, can_id: int, data: bytes, nibble_count: int,
                    timestamp: float | None = None) -> RxFrame | None:
    """Inverse of the firmware's pack_rx_can_frame()."""
    n_bytes = (nibble_count + 1) // 2
    info_idx = max(3, n_bytes)
    if len(data) <= info_idx:
        return None
    nibs = []
    for b in data[:n_bytes]:
        nibs += [b >> 4, b & 0x0F]
    info = data[info_idx]
    slow = None
    if info & SLOW_FLAG_NEW and len(data) >= info_idx + 4:
        kind = "enhanced16" if info & SLOW_FLAG_16BIT else ("enhanced" if info & SLOW_FLAG_ENHANCED else "short")
        slow = SlowMessage(kind, data[info_idx + 1], data[info_idx + 2] | (data[info_idx + 3] << 8))
    return RxFrame(channel, can_id, time.time() if timestamp is None else timestamp,
                   info >> 4, tuple(nibs[:nibble_count]), slow, bytes(data))


@dataclass
class LearnResult:
    tick_us: float
    nibbles: int
    crc_mode: CrcMode
    crc_seed: int
    legacy_crc: bool

    @classmethod
    def from_frame(cls, data: bytes) -> "LearnResult":
        if len(data) < 4:
            raise ValueError("short learn result")
        sel = data[4] if len(data) >= 5 else 0x03
        return cls(tick_us=(data[0] | data[1] << 8) / 10.0, nibbles=data[2],
                   crc_mode=CrcMode.STATUS_AND_DATA if data[3] else CrcMode.DATA_ONLY,
                   crc_seed=(sel & 0x0F) or 0x03, legacy_crc=bool(sel & 0x80))

    def to_format(self) -> SentFormat:
        return SentFormat(self.tick_us, self.nibbles, self.crc_mode, self.crc_seed, self.legacy_crc)


@dataclass
class DeviceInfo:
    firmware: str
    channels: int
    legacy: bool                      # v1.x firmware: no info command
    modes: list[str] = field(default_factory=list)
    learning_channel: int | None = None          # 1-based
    termination_on: list[bool] | None = None
    termination_auto: list[bool] | None = None
    features: int = 0

    @property
    def has_termination(self) -> bool:
        return bool(self.features & FEATURE_TERMINATION)

    @property
    def has_supply(self) -> bool:
        return bool(self.features & FEATURE_SUPPLY)

    @property
    def board(self) -> str:
        if self.legacy or self.channels == 1:
            return "SENTToUSB 1-channel (STM32F042)"
        return f"SENT_4chanel {self.channels}-channel (STM32C071)"

    @classmethod
    def from_frame(cls, data: bytes, firmware: str) -> "DeviceInfo":
        n = max(1, min(MAX_CHANNELS, data[0]))
        info = cls(firmware=firmware, channels=n, legacy=False)
        if len(data) >= 3:
            info.firmware = f"{data[1]:02X}{data[2]:02X}"
        if len(data) >= 4:
            info.modes = [MODE_NAMES[(data[3] >> (2 * i)) & 3] for i in range(n)]
        if len(data) >= 5 and data[4] != 0xFF:
            info.learning_channel = data[4] + 1
        if len(data) >= 7:
            info.features = data[6]
        if len(data) >= 6 and info.has_termination:
            info.termination_on = [bool(data[5] >> i & 1) for i in range(n)]
            info.termination_auto = [bool(data[5] >> (4 + i) & 1) for i in range(n)]
        return info

    def to_dict(self) -> dict:
        return {"board": self.board, "firmware": self.firmware, "channels": self.channels,
                "legacy_firmware": self.legacy, "modes": self.modes,
                "learning_channel": self.learning_channel,
                "termination_on": self.termination_on, "termination_auto": self.termination_auto,
                "features": {"switchable_termination": self.has_termination,
                             "supply_measurement": self.has_supply,
                             "push_pull_tx": bool(self.features & FEATURE_PUSH_PULL)}}


@dataclass
class SupplyStatus:
    vs_v: float
    current_ma: int | None
    usb_path_on: bool
    fault: bool
    source: str | None              # "usb", "ext" or None
    timestamp: float

    @classmethod
    def from_frame(cls, data: bytes) -> "SupplyStatus":
        cur = data[3] | data[4] << 8 if len(data) >= 5 else 0xFFFF
        src = data[5] if len(data) >= 6 else 0xFF
        return cls(vs_v=(data[0] | data[1] << 8) / 1000.0,
                   current_ma=None if cur == 0xFFFF else cur,
                   usb_path_on=bool(data[2] & 1), fault=bool(data[2] & 2),
                   source=None if src == 0xFF else ("ext" if src else "usb"),
                   timestamp=time.time())

    def to_dict(self) -> dict:
        d = asdict(self)
        d["timestamp"] = round(self.timestamp, 3)
        warnings = []
        if self.fault:
            warnings.append("USB sensor supply fault: short circuit or overload")
        if self.vs_v > 5.5:
            warnings.append(f"VS is {self.vs_v:.1f} V from EXT: every sensor VS pin gets it; 5 V-only sensors are damaged")
        d["warnings"] = warnings
        return d


@dataclass
class ChannelCounters:
    frames: int = 0
    crc_errors: int = 0
    sync_errors: int = 0
    lost_edges: int = 0
    usb_overflows: int = 0
    guard_trips: int = 0

    def minus(self, other: "ChannelCounters") -> "ChannelCounters":
        return ChannelCounters(**{k: v - getattr(other, k) for k, v in asdict(self).items()})

    def to_dict(self) -> dict:
        return asdict(self)
