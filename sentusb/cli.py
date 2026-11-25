"""sentusb command line.

  sentusb ports
  sentusb info
  sentusb capture -c 1 2 -s 2 --preset MLX90377
  sentusb capture -c 1 --learn --decoder h4_secure --csv ch1.csv
  sentusb learn 1
  sentusb monitor -c 1 --decoder mlx90377
  sentusb tx 2 123456 --count 100
  sentusb selftest --quick
  sentusb -p COM8 selftest --peer COM9      two boards, SENT lines wired together
  sentusb --port emu://4ch info          (built-in emulator, no hardware)

Every invocation opens the port, configures the channels (the converter cannot
report its format) and closes it again; use --learn or --preset / format options
on the command that needs the format.  --json prints machine-readable output.
Channels are numbered 1..4 as on the board.  Port: --port, $SENTUSB_PORT, or the
first converter found (USB 0483:5740).
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

from . import __version__
from .decoders import DECODERS, SENSOR_PRESETS
from .device import SentUSB, SentUSBError, find_ports
from .protocol import SentFormat
from .selftest import run_selftest


def _out(args, data, text: str | None = None):
    if args.json:
        print(json.dumps(data, indent=2, default=str))
    else:
        print(text if text is not None else json.dumps(data, indent=2, default=str))


def _format_args(p: argparse.ArgumentParser):
    g = p.add_argument_group("SENT format (default: 3 µs, 6 nibbles, data-only CRC, seed 0x03)")
    g.add_argument("--preset", choices=list(SENSOR_PRESETS), help="known sensor: format + decoder")
    g.add_argument("--tick", type=float, help="tick in µs")
    g.add_argument("--nibbles", type=int, help="data nibbles 1..8")
    g.add_argument("--crc", choices=["data_only", "status_and_data"], help="CRC mode")
    g.add_argument("--seed", type=lambda s: int(s, 0), help="CRC seed, 3 (J2716) or 5 (variant)")
    g.add_argument("--legacy-crc", action="store_true", help="pre-2010 J2716 CRC")
    g.add_argument("--learn", action="store_true", help="learn each selected channel's format first")


def _build_format(args) -> SentFormat | None:
    d = {}
    if getattr(args, "preset", None):
        d.update(SENSOR_PRESETS[args.preset]["format"])
    for key, attr in (("tick_us", "tick"), ("nibbles", "nibbles"), ("crc_mode", "crc"), ("crc_seed", "seed")):
        v = getattr(args, attr, None)
        if v is not None:
            d[key] = v
    if getattr(args, "legacy_crc", False):
        d["legacy_crc"] = True
    return SentFormat.from_dict(d) if d else None


def _decoder(args) -> str | None:
    if getattr(args, "decoder", None):
        return args.decoder
    if getattr(args, "preset", None):
        return SENSOR_PRESETS[args.preset]["decoder"]
    return None


def _open(args, channels=None) -> SentUSB:
    fmt = _build_format(args)
    formats = None
    if fmt is not None:
        formats = {c: fmt for c in channels} if channels else fmt
    dev = SentUSB(args.port or os.environ.get("SENTUSB_PORT") or None,
                  listen_only=args.listen_only, formats=formats)
    try:
        for c in channels or ():
            dev._ch(c)                          # validate against the board's channel count
        if getattr(args, "learn", False):
            for c in channels or range(1, dev.channel_count + 1):
                r = dev.learn(c)
                if not args.json:
                    print(f"CH{c} learned: tick {r.tick_us:g} µs, {r.nibbles} nibbles, {r.crc_mode.value}, "
                          f"seed 0x{r.crc_seed:X}{' legacy' if r.legacy_crc else ''}", file=sys.stderr)
    except BaseException:
        dev.close()
        raise
    return dev


# ── commands ──

def cmd_ports(args):
    ports = find_ports()
    _out(args, ports, "\n".join(f"{p['port']:8} {'converter ' if p['converter'] else '          '}"
                                f"{p['description']}" for p in ports) or "no serial ports")


def cmd_decoders(args):
    data = {"decoders": [d.to_dict() for d in DECODERS.values()], "presets": SENSOR_PRESETS}
    text = "\n".join(f"{d.name:12} {d.description}" for d in DECODERS.values())
    text += "\n\npresets:\n" + "\n".join(f"  {k:14} {v['description']}  ({v['decoder']})"
                                         for k, v in SENSOR_PRESETS.items())
    _out(args, data, text)


def cmd_info(args):
    with _open(args) as dev:
        time.sleep(0.3)
        info = dev.refresh_info()
        data = dev.to_dict()
        data["channels_state"] = [dev.channel_state(n) for n in range(1, dev.channel_count + 1)]
        if info.has_supply:
            data["supply"] = dev.supply().to_dict()
        lines = [f"{info.board}  firmware {info.firmware}  on {dev.port}"]
        for st in data["channels_state"]:
            term = st.get("termination_on")
            lines.append(f"  CH{st['channel']}  {st['mode']:<5}  termination "
                         f"{'—' if term is None else ('on' if term else 'off')}  "
                         f"last {st['last_frame']['nibbles'] if st['last_frame'] else '—'}")
        if "supply" in data:
            s = data["supply"]
            lines.append(f"  supply VS {s['vs_v']:.2f} V  {s['current_ma']} mA  from {s['source']}"
                         + ("  FAULT" if s["fault"] else ""))
            lines += [f"  ! {w}" for w in s["warnings"]]
        _out(args, data, "\n".join(lines))


def cmd_supply(args):
    with _open(args) as dev:
        s = dev.supply().to_dict()
        _out(args, s, f"VS {s['vs_v']:.2f} V  {s['current_ma']} mA  from {s['source']}  "
                      f"USB path {'on' if s['usb_path_on'] else 'off'}" + ("  FAULT" if s["fault"] else "")
             + "".join(f"\n! {w}" for w in s["warnings"]))


def cmd_learn(args):
    args.learn = False
    with _open(args) as dev:
        r = dev.learn(args.channel, timeout=args.timeout)
        fmt = r.to_format()
        flags = (f"--tick {fmt.tick_us:g} --nibbles {fmt.nibbles} --crc {fmt.crc_mode.value} "
                 f"--seed 0x{fmt.crc_seed:X}" + (" --legacy-crc" if fmt.legacy_crc else ""))
        _out(args, {"channel": args.channel, "format": fmt.to_dict(), "cli_flags": flags},
             f"CH{args.channel}: tick {fmt.tick_us:g} µs, {fmt.nibbles} nibbles, {fmt.crc_mode.value}, "
             f"seed 0x{fmt.crc_seed:X}{' (legacy CRC)' if fmt.legacy_crc else ''}\nreuse with: {flags}")


def cmd_capture(args):
    with _open(args, args.channels) as dev:
        res = dev.capture(args.seconds, args.channels, decoder=_decoder(args), csv_path=args.csv)
        _out(args, res.to_dict(include_frames=args.frames), res.summary())


def cmd_monitor(args):
    dec = DECODERS.get(_decoder(args)) if _decoder(args) else None
    with _open(args, args.channels) as dev:
        end = time.time() + args.seconds if args.seconds else None
        try:
            with dev.stream(args.channels) as q:
                while end is None or time.time() < end:
                    try:
                        f = q.get(timeout=0.2)
                    except Exception:
                        continue
                    if args.json:
                        d = f.to_dict()
                        if dec:
                            d["values"] = dec.decode(f)
                        print(json.dumps(d), flush=True)
                    else:
                        line = f"{time.strftime('%H:%M:%S', time.localtime(f.timestamp))}" \
                               f".{int(f.timestamp % 1 * 1000):03d}  CH{f.channel}  st={f.status:X}  {f.hex}"
                        if f.slow:
                            line += f"  slow {f.slow.kind} id=0x{f.slow.id:02X} val=0x{f.slow.value:X}"
                        if dec:
                            line += "  " + " ".join(f"{k}={v}" for k, v in dec.decode(f).items())
                        print(line, flush=True)
        except KeyboardInterrupt:
            pass


def cmd_tx(args):
    with _open(args, [args.channel]) as dev:
        values = args.values if len(args.values) > 1 else args.values[0]
        count = args.count if args.seconds is None else None
        job = dev.transmit(args.channel, values, status=args.status, period_ms=args.period_ms,
                           count=count, duration_s=args.seconds, pause_ticks=args.pause_ticks,
                           force=args.force)
        try:
            job.wait()
        except KeyboardInterrupt:
            job.stop()
        result = dev.stop_tx(args.channel) or job.to_dict()
        _out(args, result, f"CH{args.channel}: sent {result['sent']} frames, rejected {result['rejected']}, "
                           f"period {result['period_ms']} ms")


def cmd_selftest(args):
    pairs = [(int(p[0]), int(p[1])) for p in args.pairs.split(",")] if args.pairs else [(1, 2), (3, 4)]
    with _open(args) as dev:
        if args.peer:
            with SentUSB(args.peer, listen_only=args.listen_only) as peer:
                rep = run_selftest(dev, quick=args.quick, frames=args.frames, loopback=not args.no_loopback,
                                   peer=peer, peer_channels=(args.channel, args.peer_channel),
                                   direction=args.direction)
        else:
            rep = run_selftest(dev, pairs=pairs, quick=args.quick, frames=args.frames,
                               loopback=not args.no_loopback)
        _out(args, rep.to_dict(), rep.summary())
        return 0 if rep.passed else 2


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="sentusb", description="SENT-to-USB converter tool (1 or 4 channels).",
                                epilog="Port 'emu://4ch' or 'emu://1ch' runs the built-in emulator.")
    p.add_argument("-p", "--port", help="COMx, or emu://4ch (default: $SENTUSB_PORT or auto-detect)")
    p.add_argument("--json", action="store_true", help="machine-readable output")
    p.add_argument("--listen-only", action="store_true",
                   help="termination off: monitor a live ECU-sensor link without loading it")
    p.add_argument("--version", action="version", version=f"sentusb {__version__}")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("ports", help="list serial ports, converters first").set_defaults(func=cmd_ports)
    sub.add_parser("decoders", help="list sensor decoders and presets").set_defaults(func=cmd_decoders)
    sub.add_parser("info", help="board, firmware, channel states, supply").set_defaults(func=cmd_info)
    sub.add_parser("supply", help="sensor supply (4-channel board)").set_defaults(func=cmd_supply)

    s = sub.add_parser("learn", help="find the SENT format on a channel")
    s.add_argument("channel", type=int)
    s.add_argument("--timeout", type=float, default=5.0)
    s.set_defaults(func=cmd_learn)

    s = sub.add_parser("capture", help="record for a while, print statistics")
    s.add_argument("-c", "--channels", type=int, nargs="+", help="channels (default: all)")
    s.add_argument("-s", "--seconds", type=float, default=1.0)
    s.add_argument("--decoder", choices=list(DECODERS))
    s.add_argument("--csv", help="write every frame to this CSV file")
    s.add_argument("--frames", type=int, default=0, help="with --json: include the first N frames")
    _format_args(s)
    s.set_defaults(func=cmd_capture)

    s = sub.add_parser("monitor", help="print frames as they arrive (Ctrl+C stops)")
    s.add_argument("-c", "--channels", type=int, nargs="+")
    s.add_argument("-s", "--seconds", type=float, help="stop after this many seconds")
    s.add_argument("--decoder", choices=list(DECODERS))
    _format_args(s)
    s.set_defaults(func=cmd_monitor)

    s = sub.add_parser("tx", help="transmit SENT frames on a channel")
    s.add_argument("channel", type=int)
    s.add_argument("values", nargs="+", help="nibbles in hex, e.g. 123456; several values are cycled")
    s.add_argument("--status", type=lambda x: int(x, 0), default=0)
    s.add_argument("--period-ms", type=float, help="default: frame length + 0.5 ms")
    s.add_argument("--count", type=int, default=1)
    s.add_argument("--seconds", type=float, help="transmit for this long instead of --count frames")
    s.add_argument("--pause-ticks", type=int)
    s.add_argument("--force", action="store_true", help="transmit even if a sensor is sending on the channel")
    _format_args(s)
    s.set_defaults(func=cmd_tx)

    s = sub.add_parser("selftest", help="board self-test; loopback needs SENT1-SENT2 and SENT3-SENT4 wired, "
                                        "or a second board (--peer)")
    s.add_argument("--pairs", help="wired pairs, e.g. 12,34 (default)")
    s.add_argument("--peer", metavar="PORT",
                   help="second board (e.g. two 1-channel boards): loopback between --port and this one; "
                        "wire the SENT lines together plus GND")
    s.add_argument("--channel", type=int, default=1, help="with --peer: channel on the --port board (default 1)")
    s.add_argument("--peer-channel", type=int, default=1, help="with --peer: channel on the peer (default 1)")
    s.add_argument("--direction", choices=["both", "a-to-b", "b-to-a"], default="both",
                   help="with --peer: which way to test; a-to-b = --port TX into the peer's RX "
                        "(one wire is enough for one direction)")
    s.add_argument("--quick", action="store_true", help="2 formats instead of 6")
    s.add_argument("--frames", type=int, default=25, help="frames per format and direction")
    s.add_argument("--no-loopback", action="store_true", help="only board, supply and termination")
    s.set_defaults(func=cmd_selftest)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        rc = args.func(args)
        return rc or 0
    except (SentUSBError, ValueError, TimeoutError) as e:
        if args.json:
            print(json.dumps({"error": str(e)}))
        else:
            print(f"error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
