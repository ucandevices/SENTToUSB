"""Sensor decoders: turn a frame's nibbles into named values.

The J2716 layouts follow SAE J2716 APR2016 appendix H:
  H.1  two 12-bit fast channels: nibbles 1-3 = channel 1 MSN..LSN,
       nibbles 4-6 = channel 2 LSN..MSN (channel 2 is sent LSN first)
  H.4  secure single sensor: nibbles 1-3 = channel 1, nibbles 4-5 = 8-bit
       rolling counter, nibble 6 = 15 - nibble 1
  H.5  single sensor: nibbles 1-3 = channel 1, nibbles 4-6 = 0
Status nibble bit 0 = channel 1 error flag, bit 1 = channel 2 (H.1).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

from .protocol import RxFrame


@dataclass
class Decoder:
    name: str
    description: str
    nibbles: int | None                  # required nibble count, None = any
    func: Callable[[RxFrame], dict]
    sequence_field: str | None = None    # rolling counter checked for gaps
    sequence_modulus: int = 256
    fields: dict = field(default_factory=dict)   # field -> description / unit

    def decode(self, frame: RxFrame) -> dict:
        if self.nibbles is not None and len(frame.nibbles) != self.nibbles:
            return {"decode_error": f"{self.name} needs {self.nibbles} nibbles, frame has {len(frame.nibbles)}"}
        return self.func(frame)

    def to_dict(self) -> dict:
        return {"name": self.name, "description": self.description,
                "nibbles": self.nibbles, "fields": self.fields}


def _word(nibs) -> int:
    v = 0
    for n in nibs:
        v = (v << 4) | n
    return v


def _raw(f: RxFrame) -> dict:
    return {"value": _word(f.nibbles)}


def _h1(f: RxFrame) -> dict:
    n = f.nibbles
    return {"signal1": _word(n[0:3]), "signal2": _word((n[5], n[4], n[3])),
            "error1": bool(f.status & 1), "error2": bool(f.status & 2)}


def _h4(f: RxFrame) -> dict:
    n = f.nibbles
    return {"signal1": _word(n[0:3]), "counter": n[3] << 4 | n[4],
            "inverted_ok": n[5] == 15 - n[0], "error1": bool(f.status & 1)}


def _h5(f: RxFrame) -> dict:
    n = f.nibbles
    return {"signal1": _word(n[0:3]), "zero_ok": n[3:6] == (0, 0, 0), "error1": bool(f.status & 1)}


def _mlx90377(f: RxFrame) -> dict:
    d = _h4(f)
    d["angle_deg"] = round(d["signal1"] * 360.0 / 4096.0, 3)
    return d


DECODERS: dict[str, Decoder] = {d.name: d for d in [
    Decoder("raw", "All data nibbles as one unsigned integer (MSN first).", None, _raw,
            fields={"value": "integer of all data nibbles"}),
    Decoder("h1_dual12", "SAE J2716 H.1: two 12-bit fast channels (channel 2 sent LSN first).", 6, _h1,
            fields={"signal1": "fast channel 1, 0..4095", "signal2": "fast channel 2, 0..4095",
                    "error1": "status bit 0", "error2": "status bit 1"}),
    Decoder("h4_secure", "SAE J2716 H.4: 12-bit fast channel, 8-bit rolling counter, inverted copy of nibble 1.",
            6, _h4, sequence_field="counter",
            fields={"signal1": "fast channel 1, 0..4095", "counter": "rolling counter 0..255",
                    "inverted_ok": "nibble 6 == 15 - nibble 1", "error1": "status bit 0"}),
    Decoder("h5_single12", "SAE J2716 H.5: one 12-bit fast channel, nibbles 4-6 zero.", 6, _h5,
            fields={"signal1": "fast channel 1, 0..4095", "zero_ok": "nibbles 4-6 are 0",
                    "error1": "status bit 0"}),
    Decoder("mlx90377", "Melexis MLX90377 angle sensor (H.4 format). angle_deg assumes the 12-bit value "
            "spans 0-360 deg; a sensor programmed with another transfer curve needs its own scale.",
            6, _mlx90377, sequence_field="counter",
            fields={"angle_deg": "signal1 x 360 / 4096", "signal1": "0..4095", "counter": "0..255",
                    "inverted_ok": "secure check", "error1": "status bit 0"}),
]}


def get_decoder(name: str | None) -> Decoder | None:
    if name is None or name == "":
        return None
    try:
        return DECODERS[name]
    except KeyError:
        raise ValueError(f"unknown decoder {name!r}; known: {', '.join(DECODERS)}") from None


# Known sensors: format plus decoder.  The two pressure sensors' 12-bit raw values
# are not scaled to kPa: no transfer curve is documented here.
SENSOR_PRESETS = {
    "MLX90377": {"format": {"tick_us": 3.0, "nibbles": 6, "crc_mode": "data_only", "crc_seed": 0x03},
                 "decoder": "mlx90377", "description": "Melexis angle sensor, H.4 single secure"},
    "04L 906 051 L": {"format": {"tick_us": 3.0, "nibbles": 6, "crc_mode": "status_and_data", "crc_seed": 0x05},
                      "decoder": "h1_dual12", "description": "VW/Audi TDI DPF differential pressure sensor"},
    "GM 12643955": {"format": {"tick_us": 3.0, "nibbles": 6, "crc_mode": "data_only", "crc_seed": 0x05},
                    "decoder": "h1_dual12", "description": "GM 4-bar MAP sensor, redundant pressure"},
}
