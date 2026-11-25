"""MCP server for the SENT-to-USB converters (stdio).

Register in Claude Code:
    claude mcp add sentusb -- python -m sentusb.mcp_server
or with a fixed port / the emulator:
    claude mcp add sentusb -e SENTUSB_PORT=COM7 -- python -m sentusb.mcp_server

The server keeps one connection open between tool calls, so a learned format,
termination settings and background transmissions persist.  Tools return
summaries (statistics, last frames) rather than raw streams; capture can also
write every frame to a CSV file for detailed analysis.
"""
from __future__ import annotations

import os
import threading
from typing import Annotated, Literal

import anyio
from pydantic import Field

from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.exceptions import ToolError
from mcp.types import ToolAnnotations

from . import __version__
from .decoders import DECODERS, SENSOR_PRESETS
from .device import SentUSB, SentUSBError, find_ports
from .protocol import SentFormat
from .selftest import run_selftest

INSTRUCTIONS = """\
Controls a ucandevices SENT-to-USB converter: SAE J2716 SENT sensors on 1 channel
(SENTToUSB) or 4 channels (SENT_4chanel board).  Channels are numbered 1..4 as
printed on the board (SENT1..SENT4).

Typical flow: connect -> device_status -> learn_format(channel) -> capture(...)
with a decoder (list_decoders) -> report values, error counters and hints.

Safety:
- connect(listen_only=True) is the default: termination stays off, which is right
  for monitoring a live ECU-sensor link.  A sensor alone on the bench usually
  needs termination: connect(listen_only=False) or set_listen_only(False).
- start_transmit drives the SENT line.  It refuses a channel on which a sensor is
  sending; only use force=True when the user confirms nothing else drives that line.
- With an external supply on the board's EXT input every sensor VS pin gets that
  voltage (supply_status warns above 5.5 V); tell the user before they connect a
  5 V-only sensor.
- The serial port can be open in only one program: close sent_viewer first.
"""

mcp = MCPServer("sentusb", instructions=INSTRUCTIONS, version=__version__)

_dev: SentUSB | None = None
_lock = threading.RLock()

Channel = Annotated[int, Field(ge=1, le=4, description="Channel 1..4 as printed on the board (SENT1..SENT4)")]
RO = ToolAnnotations(readOnlyHint=True)


def _need() -> SentUSB:
    if _dev is None or not _dev.connected:
        raise SentUSBError("not connected: call connect first")
    return _dev


async def _call(fn, *args, **kwargs):
    """Run blocking device code in a worker thread; turn expected errors into tool errors."""
    def work():
        with _lock:
            return fn(*args, **kwargs)
    try:
        return await anyio.to_thread.run_sync(work)
    except (SentUSBError, ValueError, TimeoutError) as e:
        raise ToolError(str(e)) from e


# ── connection ──

@mcp.tool(annotations=RO)
async def list_ports() -> dict:
    """List serial ports; SENT-to-USB converters (USB 0483:5740) are marked converter=true."""
    return {"ports": await _call(find_ports),
            "default_port_env": os.environ.get("SENTUSB_PORT")}


@mcp.tool()
async def connect(
    port: Annotated[str | None, Field(description="COMx, or 'emu://4ch' / 'emu://1ch' for the built-in "
                                                  "emulator. Default: $SENTUSB_PORT, else the first converter")] = None,
    listen_only: Annotated[bool, Field(description="True: termination off (live ECU-sensor link). "
                                                   "False: termination on (sensor alone on the bench)")] = True,
) -> dict:
    """Open the converter, detect 1 or 4 channels, configure the default format
    (3 µs tick, 6 nibbles) on every channel and start receiving."""
    def work():
        global _dev
        if _dev is not None:
            _dev.close()
            _dev = None
        _dev = SentUSB(port or os.environ.get("SENTUSB_PORT") or None, listen_only=listen_only)
        return _dev.to_dict()
    return await _call(work)


@mcp.tool()
async def disconnect() -> dict:
    """Stop all channels and transmissions and release the serial port."""
    def work():
        global _dev
        if _dev is not None:
            _dev.close()
            _dev = None
        return {"connected": False}
    return await _call(work)


@mcp.tool(annotations=RO)
async def device_status() -> dict:
    """Board, firmware, per-channel mode / format / termination / error counters /
    last frame / transmission, and the sensor supply."""
    def work():
        dev = _need()
        dev.refresh_info()
        d = dev.to_dict()
        d["channels_state"] = [dev.channel_state(n) for n in range(1, dev.channel_count + 1)]
        if dev.info.has_supply:
            d["supply"] = dev.supply().to_dict()
        return d
    return await _call(work)


@mcp.tool(annotations=RO)
async def supply_status() -> dict:
    """Sensor supply on the 4-channel board: VS volts, current mA, source usb/ext, fault, warnings."""
    return await _call(lambda: _need().supply().to_dict())


@mcp.tool(annotations=RO)
async def list_decoders() -> dict:
    """Decoders (turn nibbles into named values, e.g. mlx90377 -> angle_deg) and known sensor presets."""
    return {"decoders": [d.to_dict() for d in DECODERS.values()], "presets": SENSOR_PRESETS}


# ── configuration ──

@mcp.tool()
async def configure_channel(
    channel: Channel,
    preset: Annotated[str | None, Field(description=f"Known sensor: {', '.join(SENSOR_PRESETS)}")] = None,
    tick_us: Annotated[float | None, Field(description="Tick time in µs (J2716: 3..90)")] = None,
    nibbles: Annotated[int | None, Field(ge=1, le=8, description="Data nibbles per frame")] = None,
    crc_mode: Annotated[Literal["data_only", "status_and_data"] | None, Field()] = None,
    crc_seed: Annotated[int | None, Field(ge=1, le=15, description="3 = J2716, 5 = VW/GM variant")] = None,
    legacy_crc: Annotated[bool | None, Field(description="Pre-2010 J2716 CRC")] = None,
) -> dict:
    """Set a channel's SENT format.  Unspecified fields keep their current value
    (a preset replaces all).  Use learn_format when the format is unknown."""
    def work():
        dev = _need()
        d = dev._ch(channel).fmt.to_dict()
        if preset:
            if preset not in SENSOR_PRESETS:
                raise ValueError(f"unknown preset {preset!r}")
            d.update(SENSOR_PRESETS[preset]["format"])
        for k, v in (("tick_us", tick_us), ("nibbles", nibbles), ("crc_mode", crc_mode),
                     ("crc_seed", crc_seed), ("legacy_crc", legacy_crc)):
            if v is not None:
                d[k] = v
        fmt = dev.configure(channel, SentFormat.from_dict(d))
        out = {"channel": channel, "format": fmt.to_dict()}
        if preset:
            out["suggested_decoder"] = SENSOR_PRESETS[preset]["decoder"]
        return out
    return await _call(work)


@mcp.tool()
async def learn_format(
    channel: Channel,
    timeout_s: Annotated[float, Field(gt=0, le=30)] = 5.0,
) -> dict:
    """Detect the SENT format of the sensor on a channel (tick, nibbles, CRC
    variant), apply it and restart receiving.  One channel learns at a time."""
    def work():
        r = _need().learn(channel, timeout=timeout_s)
        return {"channel": channel, "format": r.to_format().to_dict(),
                "note": "applied; the channel is receiving with this format"}
    return await _call(work)


@mcp.tool()
async def set_termination(
    channel: Channel,
    mode: Annotated[Literal["auto", "on", "off"], Field(description="auto: on while receiving, off while "
                                                                     "transmitting")],
) -> dict:
    """Termination (10 k pull-up + 2.2 nF) of one channel, 4-channel board only.
    Off for monitoring a live ECU-sensor link, on for a sensor alone on the bench."""
    def work():
        dev = _need()
        dev.set_termination(channel, mode)
        return {"channel": channel, "setting": mode,
                "termination_on": dev.refresh_info().termination_on[channel - 1]}
    return await _call(work)


@mcp.tool()
async def set_listen_only(enabled: bool) -> dict:
    """Restart receiving on all channels with AUTO termination kept off (True) or
    normal (False).  Stops any transmission."""
    def work():
        dev = _need()
        dev.set_listen_only(enabled)
        return {"listen_only": dev.listen_only}
    return await _call(work)


@mcp.tool()
async def start_rx(channel: Annotated[int | None, Field(ge=1, le=4, description="None = all")] = None) -> dict:
    """Put a channel (or all) into receive mode; stops a transmission on it."""
    return await _call(lambda: (_need().start_rx(channel), {"receiving": channel or "all"})[1])


@mcp.tool()
async def stop_channel(channel: Annotated[int | None, Field(ge=1, le=4, description="None = all")] = None) -> dict:
    """Stop a channel (or all): no receive, no transmit."""
    return await _call(lambda: (_need().stop(channel), {"stopped": channel or "all"})[1])


# ── receive ──

@mcp.tool(annotations=RO)
async def capture(
    channels: Annotated[list[int] | None, Field(description="Channels to record; default all")] = None,
    duration_s: Annotated[float, Field(gt=0, le=60)] = 1.0,
    decoder: Annotated[str | None, Field(description="Decoder name from list_decoders, e.g. mlx90377, "
                                                     "h1_dual12, h4_secure")] = None,
    csv_path: Annotated[str | None, Field(description="Also write every frame to this CSV file")] = None,
    include_frames: Annotated[int, Field(ge=0, le=200, description="Include the first N frames per channel")] = 0,
) -> dict:
    """Record for duration_s and return per-channel statistics: frames, fps, device
    error counters (CRC, sync, lost), value ranges from the decoder, rolling-counter
    gaps, slow-channel messages, and hints explaining problems."""
    def work():
        res = _need().capture(duration_s, channels, decoder=decoder, csv_path=csv_path)
        return res.to_dict(include_frames=include_frames)
    return await _call(work)


@mcp.tool(annotations=RO)
async def read_frames(
    channel: Channel,
    count: Annotated[int, Field(ge=1, le=200)] = 10,
    decoder: Annotated[str | None, Field(description="Optional decoder name")] = None,
) -> dict:
    """The most recent frames received on a channel (newest last)."""
    def work():
        dev = _need()
        frames = dev.history(channel, count)
        dec = DECODERS.get(decoder) if decoder else None
        if decoder and dec is None:
            raise ValueError(f"unknown decoder {decoder!r}")
        out = []
        for f in frames:
            d = f.to_dict()
            if dec:
                d["values"] = dec.decode(f)
            out.append(d)
        return {"channel": channel, "frames": out,
                "note": None if out else "no frames yet: check device_status / capture hints"}
    return await _call(work)


# ── transmit ──

@mcp.tool()
async def start_transmit(
    channel: Channel,
    values: Annotated[list[str], Field(min_length=1, max_length=4096,
                                       description="Frames as hex nibbles, e.g. ['123456']; several are cycled. "
                                                   "Length must match the channel's nibble count")],
    status: Annotated[int, Field(ge=0, le=15)] = 0,
    period_ms: Annotated[float | None, Field(gt=0, description="Default: frame length + 0.5 ms")] = None,
    count: Annotated[int | None, Field(ge=1, description="Stop after this many frames")] = None,
    duration_s: Annotated[float | None, Field(gt=0, le=3600)] = None,
    force: Annotated[bool, Field(description="Transmit although a sensor is sending on this channel. "
                                             "Only with the user's confirmation")] = False,
) -> dict:
    """Emulate a sensor: transmit SENT frames on a channel in the background,
    using the channel's configured format.  Runs until count/duration or stop_transmit."""
    def work():
        dev = _need()
        job = dev.transmit(channel, values if len(values) > 1 else values[0], status=status,
                           period_ms=period_ms, count=count, duration_s=duration_s, force=force)
        return job.to_dict()
    return await _call(work)


@mcp.tool()
async def stop_transmit(channel: Channel, resume_rx: bool = True) -> dict:
    """Stop a background transmission; the channel goes back to receiving unless resume_rx=False."""
    return await _call(lambda: _need().stop_tx(channel, resume_rx=resume_rx) or {"channel": channel,
                                                                                 "note": "nothing was running"})


@mcp.tool(annotations=RO)
async def transmit_status() -> dict:
    """Sent / rejected frame counts of every background transmission."""
    def work():
        dev = _need()
        return {"jobs": [ch.tx_job.to_dict() for ch in dev.channels if ch.tx_job]}
    return await _call(work)


# ── self-test ──

@mcp.tool()
async def run_self_test(
    pairs: Annotated[list[list[int]] | None, Field(description="Wired channel pairs, default [[1,2],[3,4]]")] = None,
    quick: bool = True,
    frames: Annotated[int, Field(ge=1, le=500)] = 25,
    loopback: Annotated[bool, Field(description="False: only board, supply and termination checks "
                                                "(no wiring needed)")] = True,
    peer_port: Annotated[str | None, Field(description="Second board's port (e.g. COM9 or emu://1ch?wire=A): "
                                                       "loopback between channel 1 of the connected board and "
                                                       "channel 1 of this one, SENT lines wired together. "
                                                       "Use it with two 1-channel boards.")] = None,
    direction: Annotated[str, Field(description="With peer_port: 'both', 'a-to-b' (connected board TX into "
                                                "the peer's RX) or 'b-to-a'; one wire is enough for one "
                                                "direction")] = "both",
) -> dict:
    """Board self-test.  The loopback part needs SENT1-SENT2 and SENT3-SENT4 wired together
    on a 4-channel board, or (peer_port) a second board wired to the first; no sensors
    connected.  It transmits on those channels."""
    def work():
        dev = _need()
        if peer_port:
            with SentUSB(peer_port, listen_only=dev.listen_only) as peer:
                return run_selftest(dev, quick=quick, frames=frames, loopback=loopback, peer=peer,
                                    direction=direction).to_dict()
        p = [tuple(x) for x in pairs] if pairs else [(1, 2), (3, 4)]
        return run_selftest(dev, pairs=p, quick=quick, frames=frames, loopback=loopback).to_dict()
    return await _call(work)


def main():
    mcp.run("stdio")


if __name__ == "__main__":
    main()
