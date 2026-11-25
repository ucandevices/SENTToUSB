#!/usr/bin/env python3
"""
SENT Sensor Monitor - viewer for the ucandevices SENT-to-USB converters.

One program for both boards:
  * SENTToUSB, 1 channel (STM32F042), firmware V01xx and V02xx
  * SENT_4chanel, 4 channels (STM32C071), firmware V02xx

The channel count is read from the device when it connects (control command
0x06, answered with info frame 0x60F).  Firmware that does not know the
command (v1.x) answers with a NACK and is run as one channel; its frame IDs
are exactly the channel-0 IDs of the newer firmware.

Host protocol: SLCAN text over USB CDC.  Channel n = 0..3 (SENT1..SENT4):
  config        t001+n     data nibbles, CRC mode/selector, tick window, RX ID
  control       t600       data[0] = n << 4 | command
                           1 RX, 2 TX, 3 stop, 4 learn, 5 TX tick,
                           6 device info (0x60F), 7 termination (0 off/1 on/2 auto)
  TX frame      t520+0x40n status, nibbles packed big-endian
  RX frame      t510+0x40n by default (configurable per channel)
  diagnostics   t511+0x40n frames LE32, CRC errors LE16, sync errors LE16
  losses        t512+0x40n lost edges LE32, USB overflows LE16, guard trips LE16
  learn result  t601+n     tick x0.1 us LE16, nibbles, CRC mode, CRC selector
  device info   t60F       channels, version BCD, modes, learning channel,
                           termination state, board features
  supply        t60E       VS mV LE16, flags, current mA LE16, source
                           (4-channel board, every 500 ms)
  'O' starts RX on every channel, 'L' the same with AUTO termination off
  (listen-only, 4-channel firmware), 'C' stops every channel.

Requirements:  pip install pyserial
"""

import tkinter as tk
from tkinter import ttk, font, filedialog, messagebox
import serial
import serial.tools.list_ports
import threading
import queue
import time
import re
from collections import deque
from datetime import datetime
import os

# Release this viewer belongs to (matches the GitHub release tag).
APP_VERSION = "2.0"


def _build_time() -> str:
    """Build time shown in the title bar.

    A packaged exe carries the time it was built (build_viewer.py writes
    sent_viewer_build.py before running PyInstaller).  When run from source
    there is no build, so the script's last-modified time is shown instead.
    """
    try:
        from sent_viewer_build import BUILD_TIME
        return BUILD_TIME
    except ImportError:
        pass
    try:
        mtime = os.path.getmtime(os.path.abspath(__file__))
        return datetime.fromtimestamp(mtime).strftime("%Y-%m-%d %H:%M") + " (source)"
    except OSError:
        return "unknown"


APP_TITLE = f"SENT Sensor Monitor v{APP_VERSION}  -  build {_build_time()}"


# ─── Protocol constants (firmware Core/Inc/sent_bridge.h, sent_core.c) ───────

MAX_CHANNELS = 4
ID_STRIDE    = 0x40
ID_CONFIG    = 0x001
ID_CONTROL   = 0x600
ID_LEARN     = 0x601
ID_RX        = 0x510
ID_DIAG      = 0x511
ID_LOSS      = 0x512
ID_TX        = 0x520
ID_INFO      = 0x60F
ID_SUPPLY    = 0x60E

CMD_RX, CMD_TX, CMD_STOP, CMD_LEARN, CMD_TX_TICK, CMD_INFO, CMD_TERM = range(1, 8)

FEATURE_TERMINATION = 0x01   # switchable termination (command 0x07)
FEATURE_SUPPLY      = 0x02   # supply measurement (frame 0x60E)
FEATURE_PUSH_PULL   = 0x04   # push-pull TX buffer

MODE_NAMES   = ("off", "RX", "TX", "learn")
TERM_CHOICES = {"Auto": 2, "On": 1, "Off": 0}

USB_VID, USB_PID = 0x0483, 0x5740        # ST virtual COM port (both boards)

DISCONNECTED = "\x00disconnected"        # rx thread -> UI: the port went away


def slcan_frame(can_id: int, data: bytes) -> bytes:
    return f"t{can_id:03X}{len(data):X}{data.hex().upper()}\r".encode("ascii")


def parse_frame(line: str):
    """'tIIIDdd..' -> (id, data) or None."""
    m = re.match(r'^t([0-9A-Fa-f]{3})([0-9A-Fa-f])([0-9A-Fa-f]*)$', line)
    if not m:
        return None
    cid = int(m.group(1), 16)
    dlc = int(m.group(2), 16)
    if dlc > 8 or len(m.group(3)) < dlc * 2:
        return None
    try:
        return cid, bytes.fromhex(m.group(3)[:dlc * 2])
    except ValueError:
        return None


def le16(b: bytes, i: int) -> int:
    return b[i] | (b[i + 1] << 8)


def le32(b: bytes, i: int) -> int:
    return b[i] | (b[i + 1] << 8) | (b[i + 2] << 16) | (b[i + 3] << 24)


# ─── Sensor presets ───────────────────────────────────────────────────────────

SENSOR_PRESETS = {
    "MLX90377": {
        "tick_us":     "3",
        "sync_ticks":  "56",
        "sync_min_us": "100",
        "nibbles":     "6",
        "crc_poly":    "0x0D",
        "crc_init":    "0x03",
        "crc_mode":    "DATA_ONLY",
        "info": [
            ("Type:",    "Angle sensor (H.4 Single Secure)"),
            ("Output:",  "12-bit angle  0x000–0xFFF"),
            ("Format:",  "data[0..1]=angle  data[1..2]=mag"),
        ],
    },
    "04L 906 051 L": {
        "tick_us":     "3",
        "sync_ticks":  "56",
        "sync_min_us": "100",
        "nibbles":     "6",
        "crc_poly":    "0x0D",
        "crc_init":    "0x05",
        "crc_mode":    "STATUS_AND_DATA",
        "info": [
            ("Type:",    "DPF diff-pressure sensor (VW/Audi TDI)"),
            ("Output:",  "sig0 + sig1 (12-bit each, 6 nibbles)"),
            ("CRC:",     "SAE J2716 status+data, seed 0x05"),
        ],
    },
    "GM 12643955": {
        "tick_us":     "3",
        "sync_ticks":  "56",
        "sync_min_us": "100",
        "nibbles":     "6",
        "crc_poly":    "0x0D",
        "crc_init":    "0x05",
        "crc_mode":    "DATA_ONLY",
        "info": [
            ("Type:",    "4-bar MAP sensor (GM Duramax / 2.0T)"),
            ("Output:",  "sig0 + sig1 redundant pressure (12-bit)"),
            ("CRC:",     "GM DATA_ONLY variant, seed 0x05"),
        ],
    },
}

PARAM_FIELDS = [
    ("Tick Size (µs):",  "tick_us"),
    ("Sync Ticks:",      "sync_ticks"),
    ("Min Sync (µs):",   "sync_min_us"),
    ("Data Nibbles:",    "nibbles"),
    ("CRC Polynomial:",  "crc_poly"),
    ("CRC Init Seed:",   "crc_init"),
]


# ─── Per-channel state ────────────────────────────────────────────────────────

class Channel:
    """Settings (Tk variables, shown when the channel is selected) and
    live state of one SENT channel."""

    def __init__(self, idx: int):
        self.idx = idx
        self.name = f"CH{idx + 1}"
        preset = SENSOR_PRESETS["MLX90377"]
        self.preset = tk.StringVar(value="MLX90377")
        self.params = {key: tk.StringVar(value=preset[key]) for _, key in PARAM_FIELDS}
        self.params["data_can_id"] = tk.StringVar(value=f"0x{ID_RX + ID_STRIDE * idx:03X}")
        self.crc_mode = tk.StringVar(value=preset["crc_mode"])
        self.term = tk.StringVar(value="Auto")
        self.tx_status = tk.StringVar(value="0")
        self.tx_nibbles = tk.StringVar(value="123456")
        self.tx_period = tk.StringVar(value="100")
        self.learn_result = tk.StringVar(value="")

        self.mode = "—"            # off / RX / TX / learn (from 0x60F, else what we sent)
        self.term_on = None        # effective termination from 0x60F, None = unknown
        self.learning = False
        self.tx_active = False
        self.tx_after = None
        self.reset_stats()

    def reset_stats(self):
        self.frames = self.crc_errors = self.sync_errors = 0
        self.lost_edges = self.usb_overflows = self.guard_trips = 0
        self.times: deque = deque()
        self.last_data = "—"
        self.last_nibbles = "—"
        self.last_meta = ""

    def rx_id(self):
        try:
            return int(self.params["data_can_id"].get(), 0)
        except ValueError:
            return None


# ─── Main application ─────────────────────────────────────────────────────────

class SentMonitor:
    MAX_TABLE_ROWS = 10000  # rows kept in the frame table
    POLL_MS        = 40     # UI update interval
    INFO_POLL_MS   = 1000   # device info refresh (channel modes, termination)
    OPEN_WAIT_MS   = 400    # wait for the info answer before opening (v1.x: none)

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry("1200x840")
        self.root.configure(bg="#1e1e2e")

        self.serial_port: serial.Serial | None = None
        self.rx_thread: threading.Thread | None = None
        self.running = False
        self.paused  = False
        self.rx_queue: queue.Queue[str] = queue.Queue()

        self.channels = [Channel(i) for i in range(MAX_CHANNELS)]
        self.n_ch = 1                  # channels on the connected device
        self.features = 0              # FEATURE_* bits from 0x60F
        self.fw_version = ""
        self.info_seen = False         # device answered command 0x06
        self.opened = False            # 'O' / 'L' sent
        self._open_after = None
        self._info_after = None
        self._silent_info = 0          # info polls whose answer is not logged
        self._silent_acks = 0
        self._learn_sent_at = 0.0

        self.sel_var = tk.IntVar(value=0)
        self.listen_var = tk.BooleanVar(value=False)
        self.filter_var = tk.StringVar(value="All")
        self._rows: deque = deque()          # (channel idx, values) of every row, newest first
        self._tree_iids: deque = deque()     # iids of rows shown, newest first

        self.log_file = None
        self.recording = False

        self._build_ui()
        self._select_channel()
        self._set_channel_count(1)
        self._refresh_ports()
        self._poll_queue()

    @property
    def sel(self) -> Channel:
        return self.channels[self.sel_var.get()]

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except Exception:
            pass
        style.configure("TLabelframe",       background="#1e1e2e", foreground="#AAAACC")
        style.configure("TLabelframe.Label", background="#1e1e2e", foreground="#AAAACC",
                        font=("Consolas", 9, "bold"))
        style.configure("TLabel",  background="#1e1e2e", foreground="#CCCCDD")
        style.configure("TFrame",  background="#1e1e2e")
        style.configure("TEntry",  fieldbackground="#2a2a3e", foreground="#CCCCDD",
                        insertcolor="#AAAACC")
        style.configure("TCombobox", fieldbackground="#2a2a3e", foreground="#CCCCDD",
                        selectbackground="#3a3a5e", selectforeground="#FFFFFF",
                        arrowcolor="#AAAACC")
        style.map("TCombobox",
            fieldbackground=[("readonly", "#2a2a3e"), ("focus", "#2a2a3e"),
                             ("disabled", "#1a1a2e")],
            foreground=[("readonly", "#CCCCDD"), ("focus", "#FFFFFF"),
                        ("disabled", "#555566")],
            selectbackground=[("readonly", "#3a3a5e")],
            selectforeground=[("readonly", "#FFFFFF")])
        style.configure("TButton", background="#2a2a3e", foreground="#CCCCDD",
                        font=("Consolas", 9))
        style.map("TButton",
            background=[("active", "#4a4a6e"), ("pressed", "#5a5a7e"),
                        ("disabled", "#1a1a2e")],
            foreground=[("active", "#FFFFFF"), ("pressed", "#FFFFFF"),
                        ("disabled", "#555566")])
        style.configure("Treeview", background="#181828", foreground="#CCCCDD",
                        fieldbackground="#181828", rowheight=20)
        style.configure("Treeview.Heading", background="#2a2a3e", foreground="#AAAACC",
                        font=("Consolas", 9, "bold"))
        style.map("Treeview", background=[("selected", "#3a3a5e")])
        self.root.option_add("*TCombobox*Listbox.background",       "#2a2a3e")
        self.root.option_add("*TCombobox*Listbox.foreground",       "#CCCCDD")
        self.root.option_add("*TCombobox*Listbox.selectBackground", "#4a4a6e")
        self.root.option_add("*TCombobox*Listbox.selectForeground", "#FFFFFF")
        self.root.option_add("*TCombobox*Listbox.font",             "Consolas 9")

        paned = tk.PanedWindow(self.root, orient=tk.HORIZONTAL,
                               sashwidth=5, sashrelief=tk.FLAT, bg="#1e1e2e")
        paned.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        left  = ttk.Frame(paned, width=250)
        right = ttk.Frame(paned)
        paned.add(left,  minsize=230)
        paned.add(right, minsize=640)

        self._build_left(left)
        self._build_right(right)

    def _check(self, parent, text, var, command=None):
        return tk.Checkbutton(parent, text=text, variable=var, command=command,
                              bg="#1e1e2e", fg="#AAAACC", selectcolor="#2a2a3e",
                              activebackground="#1e1e2e", activeforeground="#CCCCDD",
                              font=("Consolas", 9), anchor=tk.W)

    def _build_left(self, parent):
        # ── Serial port ──
        pf = ttk.LabelFrame(parent, text=" Serial Port ", padding=8)
        pf.pack(fill=tk.X, padx=6, pady=(6, 3))

        ttk.Label(pf, text="Port:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(pf, textvariable=self.port_var, width=11, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky=tk.EW, padx=(4, 0))
        ttk.Button(pf, text="⟳", width=3, command=self._refresh_ports).grid(row=0, column=2, padx=(3, 0))
        pf.columnconfigure(1, weight=1)

        btn_row = ttk.Frame(pf)
        btn_row.grid(row=1, column=0, columnspan=3, sticky=tk.EW, pady=(6, 0))
        self.connect_btn    = ttk.Button(btn_row, text="Connect", command=self._connect)
        self.disconnect_btn = ttk.Button(btn_row, text="Disconnect", command=self._disconnect,
                                         state=tk.DISABLED)
        self.connect_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2))
        self.disconnect_btn.pack(side=tk.LEFT, expand=True, fill=tk.X)

        self.status_lbl = ttk.Label(pf, text="● Disconnected", foreground="#FF6666")
        self.status_lbl.grid(row=2, column=0, columnspan=3, pady=(5, 0))

        self.listen_chk = self._check(pf, "Listen only (termination off)", self.listen_var,
                                      self._on_listen_toggle)
        self.listen_chk.grid(row=3, column=0, columnspan=3, sticky=tk.W, pady=(4, 0))

        # ── Channel selector ──
        cf = ttk.LabelFrame(parent, text=" Channel ", padding=(8, 4))
        cf.pack(fill=tk.X, padx=6, pady=3)
        self.ch_buttons = []
        for i in range(MAX_CHANNELS):
            b = tk.Radiobutton(cf, text=f"CH{i + 1}", value=i, variable=self.sel_var,
                               indicatoron=False, command=self._select_channel,
                               width=5, font=("Consolas", 9, "bold"),
                               bg="#2a2a3e", fg="#CCCCDD", selectcolor="#4a6a9e",
                               activebackground="#4a4a6e", activeforeground="#FFFFFF",
                               disabledforeground="#444455", relief=tk.FLAT, bd=1)
            b.grid(row=0, column=i, padx=1, sticky=tk.EW)
            cf.columnconfigure(i, weight=1)
            self.ch_buttons.append(b)

        # ── SENT parameters of the selected channel ──
        self.param_frame = ttk.LabelFrame(parent, text=" CH1 Parameters ", padding=8)
        sf = self.param_frame
        sf.pack(fill=tk.X, padx=6, pady=3)

        ttk.Label(sf, text="Preset:").grid(row=0, column=0, sticky=tk.W, pady=(1, 4))
        self.preset_cb = ttk.Combobox(sf, width=13, values=list(SENSOR_PRESETS.keys()),
                                      state="readonly")
        self.preset_cb.grid(row=0, column=1, sticky=tk.EW, padx=(4, 0), pady=(1, 4))
        self.preset_cb.bind("<<ComboboxSelected>>", lambda _e: self._apply_preset())

        self.param_entries: dict[str, ttk.Entry] = {}
        for i, (lbl, key) in enumerate(PARAM_FIELDS):
            ttk.Label(sf, text=lbl).grid(row=i + 1, column=0, sticky=tk.W, pady=1)
            e = ttk.Entry(sf, width=9)
            e.grid(row=i + 1, column=1, sticky=tk.EW, padx=(4, 0))
            self.param_entries[key] = e

        n = len(PARAM_FIELDS) + 1
        ttk.Label(sf, text="CRC Mode:").grid(row=n, column=0, sticky=tk.W, pady=1)
        self.crc_mode_cb = ttk.Combobox(sf, width=13, values=["DATA_ONLY", "STATUS_AND_DATA"],
                                        state="readonly")
        self.crc_mode_cb.grid(row=n, column=1, sticky=tk.EW, padx=(4, 0))

        ttk.Label(sf, text="RX CAN ID:").grid(row=n + 1, column=0, sticky=tk.W, pady=1)
        e = ttk.Entry(sf, width=9)
        e.grid(row=n + 1, column=1, sticky=tk.EW, padx=(4, 0))
        self.param_entries["data_can_id"] = e

        ttk.Label(sf, text="Termination:").grid(row=n + 2, column=0, sticky=tk.W, pady=1)
        self.term_cb = ttk.Combobox(sf, width=13, values=list(TERM_CHOICES), state="readonly")
        self.term_cb.grid(row=n + 2, column=1, sticky=tk.EW, padx=(4, 0))
        self.term_cb.bind("<<ComboboxSelected>>", lambda _e: self._send_termination(self.sel))
        sf.columnconfigure(1, weight=1)

        # ── Channel buttons ──
        ctrl = ttk.Frame(parent)
        ctrl.pack(fill=tk.X, padx=6, pady=(6, 4))
        row = ttk.Frame(ctrl)
        row.pack(fill=tk.X, pady=2)
        ttk.Button(row, text="Apply Config",
                   command=lambda: self._send_config(self.sel)).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2))
        self.apply_all_btn = ttk.Button(row, text="Apply to All", command=self._apply_all)
        self.apply_all_btn.pack(side=tk.LEFT, expand=True, fill=tk.X)
        row = ttk.Frame(ctrl)
        row.pack(fill=tk.X, pady=2)
        ttk.Button(row, text="Start RX", command=self._start_rx_selected).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2))
        ttk.Button(row, text="Stop", command=self._stop_selected).pack(
            side=tk.LEFT, expand=True, fill=tk.X)
        self.learn_btn = ttk.Button(ctrl, text="Learn", command=self._learn)
        self.learn_btn.pack(fill=tk.X, pady=2)
        self.learn_result_lbl = ttk.Label(ctrl, foreground="#88FFAA", wraplength=210,
                                          font=("Consolas", 8))
        self.learn_result_lbl.pack(fill=tk.X, pady=(0, 2))

        # ── Sensor info (follows the preset) ──
        self._info_frame = ttk.LabelFrame(parent, text=" Sensor Info ", padding=8)
        self._info_frame.pack(fill=tk.X, padx=6, pady=3)
        self._info_labels: list[tuple[ttk.Label, ttk.Label]] = []
        for i in range(3):
            k_lbl = ttk.Label(self._info_frame, text="")
            v_lbl = ttk.Label(self._info_frame, text="", foreground="#888899")
            k_lbl.grid(row=i, column=0, sticky=tk.W, pady=1)
            v_lbl.grid(row=i, column=1, sticky=tk.W, padx=4)
            self._info_labels.append((k_lbl, v_lbl))

    def _build_right(self, parent):
        # ── Device / supply bar ──
        bar = ttk.Frame(parent)
        bar.pack(fill=tk.X, padx=6, pady=(6, 2))
        self.sv_device = tk.StringVar(value="No device")
        self.sv_supply = tk.StringVar(value="")
        ttk.Label(bar, textvariable=self.sv_device, relief=tk.SUNKEN, padding=(8, 3),
                  font=("Consolas", 10), foreground="#88CCFF").pack(side=tk.LEFT, padx=3)
        self.supply_lbl = ttk.Label(bar, textvariable=self.sv_supply, padding=(8, 3),
                                    font=("Consolas", 10), foreground="#CCCCDD")
        self.supply_lbl.pack(side=tk.LEFT, padx=3)

        # ── Last frame of the selected channel + TX panel ──
        mid = ttk.Frame(parent)
        mid.pack(fill=tk.X, padx=6, pady=(4, 2))

        big_font  = font.Font(family="Consolas", size=22, weight="bold")
        med_font  = font.Font(family="Consolas", size=13)
        info_font = font.Font(family="Consolas", size=10)

        rx_disp = ttk.Frame(mid)
        rx_disp.pack(side=tk.LEFT, fill=tk.Y)
        self.sv_last_data    = tk.StringVar(value="—")
        self.sv_last_nibbles = tk.StringVar(value="—")
        self.sv_last_meta    = tk.StringVar(value="")
        tk.Label(rx_disp, textvariable=self.sv_last_data, font=big_font, fg="#00BFFF",
                 bg="#1e1e2e", anchor=tk.W).pack(fill=tk.X)
        tk.Label(rx_disp, textvariable=self.sv_last_nibbles, font=med_font, fg="#4488CC",
                 bg="#1e1e2e", anchor=tk.W).pack(fill=tk.X)
        tk.Label(rx_disp, textvariable=self.sv_last_meta, font=info_font, fg="#888899",
                 bg="#1e1e2e", anchor=tk.W).pack(fill=tk.X, pady=(2, 0))

        tx_wrapper = ttk.Frame(mid)
        tx_wrapper.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.tx_frame = ttk.LabelFrame(tx_wrapper, text=" TX SENT Frame - CH1 ", padding=(6, 4))
        self.tx_frame.pack(anchor=tk.CENTER, expand=True)
        tx_grid = ttk.Frame(self.tx_frame)
        tx_grid.pack(anchor=tk.W)

        self.tx_entries = {}
        for r, (lbl, key, width, hint) in enumerate([
                ("Status:",  "tx_status",  3,  "(hex nibble)"),
                ("Nibbles:", "tx_nibbles", 10, "(hex, e.g. 123456)"),
                ("Period:",  "tx_period",  6,  "ms  (0 = one-shot)")]):
            ttk.Label(tx_grid, text=lbl, font=("Consolas", 9)).grid(
                row=r, column=0, sticky=tk.E, padx=(0, 4), pady=2)
            e = ttk.Entry(tx_grid, width=width, font=("Consolas", 10))
            e.grid(row=r, column=1, sticky=tk.W)
            self.tx_entries[key] = e
            ttk.Label(tx_grid, text=hint, font=("Consolas", 8), foreground="#666688").grid(
                row=r, column=2, sticky=tk.W, padx=4)

        tx_btn_row = ttk.Frame(self.tx_frame)
        tx_btn_row.pack(anchor=tk.W, pady=(4, 0))
        ttk.Button(tx_btn_row, text="Send", width=7, command=self._tx_send).pack(
            side=tk.LEFT, padx=(0, 4))
        self.tx_toggle_btn = ttk.Button(tx_btn_row, text="Start", width=7, command=self._tx_toggle)
        self.tx_toggle_btn.pack(side=tk.LEFT)

        # ── Channel overview ──
        of = ttk.LabelFrame(parent, text=" Channels ", padding=4)
        of.pack(fill=tk.X, padx=6, pady=2)
        ocols = {
            "ch":      ("Ch",          45, tk.CENTER),
            "mode":    ("Mode",        50, tk.CENTER),
            "term":    ("Termination", 100, tk.CENTER),
            "nibbles": ("Last nibbles", 220, tk.W),
            "frames":  ("Frames",      75, tk.E),
            "crc":     ("CRC err",     70, tk.E),
            "sync":    ("Sync err",    75, tk.E),
            "lost":    ("Lost",        55, tk.E),
            "rate":    ("fps",         60, tk.E),
        }
        self.overview = ttk.Treeview(of, columns=list(ocols), show="headings", height=1,
                                     selectmode="browse")
        for col, (h, w, anchor) in ocols.items():
            self.overview.heading(col, text=h)
            self.overview.column(col, width=w, minwidth=30, anchor=anchor,
                                 stretch=(col == "nibbles"))
        self.overview.pack(fill=tk.X)
        self.overview.bind("<<TreeviewSelect>>", self._on_overview_select)

        # ── Toolbar for the frame table ──
        tb = ttk.Frame(parent)
        tb.pack(fill=tk.X, padx=6, pady=(4, 0))
        ttk.Label(tb, text="Show:").pack(side=tk.LEFT)
        self.filter_cb = ttk.Combobox(tb, textvariable=self.filter_var, width=5,
                                      values=["All"], state="readonly")
        self.filter_cb.pack(side=tk.LEFT, padx=(4, 8))
        self.filter_cb.bind("<<ComboboxSelected>>", lambda _e: self._rebuild_table())
        self.drop_frames_var = tk.BooleanVar(value=True)
        self._check(tb, "Limit frame rate", self.drop_frames_var).pack(side=tk.LEFT)
        self.record_btn = ttk.Button(tb, text="● Record…", command=self._toggle_recording)
        self.record_btn.pack(side=tk.RIGHT, padx=(3, 0))
        ttk.Button(tb, text="Save Log…", command=self._save_log_dialog).pack(side=tk.RIGHT, padx=3)
        ttk.Button(tb, text="Clear", command=self._clear).pack(side=tk.RIGHT, padx=3)
        self.pause_btn = ttk.Button(tb, text="⏸  Pause", command=self._toggle_pause)
        self.pause_btn.pack(side=tk.RIGHT, padx=3)

        # ── Frame table ──
        tf = ttk.LabelFrame(parent, text=" Received Frames ", padding=4)
        tf.pack(fill=tk.BOTH, expand=True, padx=6, pady=2)
        cols = ("time", "ch", "can_id", "dlc", "data_hex", "nibbles")
        self.tree = ttk.Treeview(tf, columns=cols, show="headings", height=8,
                                 selectmode="extended")
        headers = {
            "time":     ("Time",        95, tk.CENTER),
            "ch":       ("Ch",          45, tk.CENTER),
            "can_id":   ("CAN ID",      65, tk.CENTER),
            "dlc":      ("DLC",         35, tk.CENTER),
            "data_hex": ("Data (hex)", 150, tk.CENTER),
            "nibbles":  ("Nibbles",    200, tk.CENTER),
        }
        for col in cols:
            h, w, anchor = headers[col]
            self.tree.heading(col, text=h)
            self.tree.column(col, width=w, minwidth=30, anchor=anchor, stretch=(col == "nibbles"))
        ysb = ttk.Scrollbar(tf, orient=tk.VERTICAL,   command=self.tree.yview)
        xsb = ttk.Scrollbar(tf, orient=tk.HORIZONTAL, command=self.tree.xview)
        self.tree.configure(yscroll=ysb.set, xscroll=xsb.set)
        ysb.pack(side=tk.RIGHT, fill=tk.Y)
        xsb.pack(side=tk.BOTTOM, fill=tk.X)
        self.tree.pack(fill=tk.BOTH, expand=True)
        self.tree.tag_configure("ok", foreground="#88CC88")

        self.tree.bind("<Control-c>", self._copy_tree_selection)
        self.tree.bind("<Control-C>", self._copy_tree_selection)
        self.tree.bind("<Control-a>", self._select_all_tree)
        self.tree.bind("<Control-A>", self._select_all_tree)
        self._tree_menu = tk.Menu(self.root, tearoff=0, bg="#2a2a3e", fg="#CCCCDD",
                                  activebackground="#4a4a6e", activeforeground="#FFFFFF")
        self._tree_menu.add_command(label="Copy",       command=self._copy_tree_selection)
        self._tree_menu.add_command(label="Copy All",   command=self._copy_tree_all)
        self._tree_menu.add_command(label="Select All", command=self._select_all_tree)
        self._tree_menu.add_separator()
        self._tree_menu.add_command(label="Save to File…", command=self._save_log_dialog)
        self.tree.bind("<Button-3>", self._show_tree_menu)

        # ── Raw log ──
        lf = ttk.LabelFrame(parent, text=" Raw Log ", padding=4)
        lf.pack(fill=tk.X, padx=6, pady=(2, 6))
        self.log = tk.Text(lf, height=5, font=("Consolas", 9), bg="#131320", fg="#888899",
                           insertbackground="#AAAACC", wrap=tk.NONE)
        log_ysb = ttk.Scrollbar(lf, orient=tk.VERTICAL,   command=self.log.yview)
        log_xsb = ttk.Scrollbar(lf, orient=tk.HORIZONTAL, command=self.log.xview)
        self.log.configure(yscrollcommand=log_ysb.set, xscrollcommand=log_xsb.set)
        log_ysb.pack(side=tk.RIGHT, fill=tk.Y)
        log_xsb.pack(side=tk.BOTTOM, fill=tk.X)
        self.log.pack(fill=tk.BOTH)

        self.log.tag_config("info",   foreground="#00BFFF")   # V/v/N/F responses
        self.log.tag_config("dbg",    foreground="#FFAA44")   # notes from the viewer
        self.log.tag_config("raw",    foreground="#FF6666")   # NACK (\a)
        self.log.tag_config("slcan",  foreground="#666688")   # data frame t/T
        self.log.tag_config("other",  foreground="#555566")   # z/Z frame ACK
        self.log.tag_config("tx_cmd", foreground="#55FF55")   # outgoing commands

        self.log.bind("<Key>",      self._log_block_edit)
        self.log.bind("<<Paste>>",  lambda e: "break")
        self.log.bind("<Button-2>", lambda e: "break")
        self.log.bind("<Control-a>", self._select_all_log)
        self.log.bind("<Control-A>", self._select_all_log)
        self._log_menu = tk.Menu(self.root, tearoff=0, bg="#2a2a3e", fg="#CCCCDD",
                                 activebackground="#4a4a6e", activeforeground="#FFFFFF")
        self._log_menu.add_command(label="Copy",       command=self._copy_log_selection)
        self._log_menu.add_command(label="Copy All",   command=self._copy_log_all)
        self._log_menu.add_command(label="Select All", command=self._select_all_log)
        self._log_menu.add_separator()
        self._log_menu.add_command(label="Save Raw Log…", command=self._save_raw_log_dialog)
        self.log.bind("<Button-3>", lambda e: self._log_menu.tk_popup(e.x_root, e.y_root))

    # ── Channel selection / count ─────────────────────────────────────────────

    def _select_channel(self):
        """Point the parameter, termination and TX widgets at the selected channel."""
        ch = self.sel
        for key, e in self.param_entries.items():
            e.configure(textvariable=ch.params[key])
        self.preset_cb.configure(textvariable=ch.preset)
        self.crc_mode_cb.configure(textvariable=ch.crc_mode)
        self.term_cb.configure(textvariable=ch.term)
        for key, e in self.tx_entries.items():
            e.configure(textvariable=getattr(ch, key))
        self.learn_result_lbl.configure(textvariable=ch.learn_result)
        self.param_frame.configure(text=f" {ch.name} Parameters ")
        self.tx_frame.configure(text=f" TX SENT Frame - {ch.name} ")
        self.tx_toggle_btn.config(text="Stop" if ch.tx_active else "Start")
        self._show_preset_info()
        self._update_learn_button()
        self._show_last(ch)
        iid = f"ch{ch.idx}"
        if self.overview.exists(iid) and self.overview.selection() != (iid,):
            self.overview.selection_set(iid)

    def _on_overview_select(self, _event=None):
        sel = self.overview.selection()
        if sel:
            idx = int(sel[0][2:])
            if idx != self.sel_var.get():
                self.sel_var.set(idx)
                self._select_channel()

    def _set_channel_count(self, n: int):
        self.n_ch = n
        for i, b in enumerate(self.ch_buttons):
            b.configure(state=tk.NORMAL if i < n else tk.DISABLED)
        if self.sel_var.get() >= n:
            self.sel_var.set(0)
        self.overview.delete(*self.overview.get_children())
        for ch in self.channels[:n]:
            self.overview.insert("", tk.END, iid=f"ch{ch.idx}", values=(ch.name,))
        self.overview.configure(height=n)
        self.filter_cb.configure(values=["All"] + [c.name for c in self.channels[:n]])
        if self.filter_var.get() not in self.filter_cb.cget("values"):
            self.filter_var.set("All")
        self.apply_all_btn.configure(state=tk.NORMAL if n > 1 else tk.DISABLED)
        self._update_termination_widgets()
        self._select_channel()

    def _update_termination_widgets(self):
        has_term = bool(self.features & FEATURE_TERMINATION)
        # Before the device is known, allow presetting; a known board without
        # switchable termination (1-channel) greys the controls out.
        known_without = self.info_seen and not has_term or (self.opened and not self.info_seen)
        self.term_cb.configure(state=tk.DISABLED if known_without else "readonly")
        self.listen_chk.configure(state=tk.DISABLED if known_without else tk.NORMAL)

    # ── Presets ───────────────────────────────────────────────────────────────

    def _apply_preset(self):
        ch = self.sel
        p = SENSOR_PRESETS.get(ch.preset.get())
        if p is None:
            return
        for key, val in p.items():
            if key == "crc_mode":
                ch.crc_mode.set(val)
            elif key in ch.params:
                ch.params[key].set(val)
        self._show_preset_info()

    def _show_preset_info(self):
        p = SENSOR_PRESETS.get(self.sel.preset.get(), {})
        info = p.get("info", [])
        for i, (k_lbl, v_lbl) in enumerate(self._info_labels):
            k_lbl.config(text=info[i][0] if i < len(info) else "")
            v_lbl.config(text=info[i][1] if i < len(info) else "")

    # ── Serial port ───────────────────────────────────────────────────────────

    def _refresh_ports(self):
        infos = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
        ports = [p.device for p in infos]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            preferred = next((p.device for p in infos
                              if p.vid == USB_VID and p.pid == USB_PID), ports[0])
            self.port_var.set(preferred)

    def _connected(self) -> bool:
        return bool(self.serial_port and self.serial_port.is_open)

    def _connect(self):
        port = self.port_var.get()
        try:
            self.serial_port = serial.Serial(port, 115200, timeout=0.1)
            self.serial_port.reset_input_buffer()   # discard stale OS-buffered data
        except Exception as e:
            self.status_lbl.config(text=f"● {e}", foreground="#FF6666")
            self.serial_port = None
            return
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

        self.info_seen = False
        self.opened = False
        self.features = 0
        self.fw_version = ""
        self._silent_info = self._silent_acks = 0
        self.sv_device.set("Detecting…")
        self.sv_supply.set("")
        for ch in self.channels:
            ch.mode, ch.term_on, ch.learning = "—", None, False
        self._set_channel_count(1)

        # Ask for version and device info first; the channels are opened when
        # the info frame arrives, on a NACK (v1.x firmware) or after a timeout.
        self._send(b"V\r")
        self._send_ctrl(self.channels[0], CMD_INFO)
        self._open_after = self.root.after(self.OPEN_WAIT_MS, self._finish_open)

        self.connect_btn.config(state=tk.DISABLED)
        self.disconnect_btn.config(state=tk.NORMAL)
        self.status_lbl.config(text=f"● Connected  {port}", foreground="#66CC66")

    def _finish_open(self):
        """Start RX on every channel and push each channel's settings."""
        if self._open_after is not None:
            self.root.after_cancel(self._open_after)
            self._open_after = None
        if self.opened or not self._connected():
            return
        self.opened = True
        if not self.info_seen:
            self._append_log("(no device info: v1.x firmware, 1 channel)", tag="dbg")
            self.sv_device.set(f"FW {self.fw_version or '?'}  ·  1 channel")
        self._update_termination_widgets()

        listen = self.listen_var.get() and self.features & FEATURE_TERMINATION
        self._send(b"L\r" if listen else b"O\r")
        for ch in self.channels[:self.n_ch]:
            ch.mode = "RX"
            self._send_config(ch)
            if self.features & FEATURE_TERMINATION and ch.term.get() != "Auto":
                self._send_termination(ch)
        if self.info_seen:
            self._schedule_info_poll()

    def _schedule_info_poll(self):
        if self._info_after is not None:
            self.root.after_cancel(self._info_after)
        self._info_after = self.root.after(self.INFO_POLL_MS, self._info_poll)

    def _info_poll(self):
        self._info_after = None
        if not self._connected():
            return
        self._silent_info += 1
        self._silent_acks += 1
        self._send_ctrl(self.channels[0], CMD_INFO, log=False)
        self._schedule_info_poll()

    def _disconnect(self, reason: str | None = None):
        for ch in self.channels:
            self._tx_stop(ch, resume=False)
        self._stop_recording()
        for aid in (self._open_after, self._info_after):
            if aid is not None:
                self.root.after_cancel(aid)
        self._open_after = self._info_after = None
        self.running = False
        if self.serial_port:
            try:
                self._send(b"C\r")   # stop every channel before closing
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
        self.opened = False
        for ch in self.channels:
            ch.learning = False
            ch.learn_result.set("")
            ch.mode = "—"
        self._update_learn_button()
        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)
        self.listen_chk.configure(state=tk.NORMAL)
        self.term_cb.configure(state="readonly")
        self.sv_device.set("No device")
        self.sv_supply.set("")
        if reason:
            self.status_lbl.config(text=f"● {reason}", foreground="#FF6666")
        else:
            self.status_lbl.config(text="● Disconnected", foreground="#FF6666")

    # ── Commands ──────────────────────────────────────────────────────────────

    def _send(self, data: bytes, log: bool = True):
        """Write bytes to the serial port and echo them to the raw log (green)."""
        if not self._connected():
            return
        try:
            self.serial_port.write(data)
        except Exception:
            return   # the rx thread reports the lost port
        if log:
            label = data.decode("ascii", errors="replace").replace("\r", "\\r")
            self._append_log(f"→ {label}", tag="tx_cmd")

    def _send_ctrl(self, ch: Channel, cmd: int, extra: bytes = b"", log: bool = True):
        self._send(slcan_frame(ID_CONTROL, bytes([(ch.idx << 4) | cmd]) + extra), log=log)

    def _send_config(self, ch: Channel):
        """Send the config frame (0x001+n) and the TX tick (control 0x05).

        Config frame (DLC 7):
          B0 : data nibbles (1..8)
          B1 : CRC mode     (0=DATA_ONLY, 1=STATUS_AND_DATA)
          B2 : CRC selector (0x03 recommended, 0x05 GM/VW variant,
                             0x85 SAE J2716 legacy FEB2008)
          B3 : min tick, units 0.5 µs  (= tick * 0.8 / 0.5)
          B4 : max tick, units 1 µs    (= tick * 1.5)
          B5 : output RX CAN ID, high byte (big-endian)
          B6 : output RX CAN ID, low byte
        """
        if not self._connected():
            return
        try:
            nibbles  = int(ch.params["nibbles"].get())
            crc_mode = 1 if ch.crc_mode.get() == "STATUS_AND_DATA" else 0
            seed     = int(ch.params["crc_init"].get(), 0)
            tick     = float(ch.params["tick_us"].get())
            can_id   = int(ch.params["data_can_id"].get(), 0)
        except (ValueError, TypeError):
            self._append_log(f"({ch.name}: invalid parameter, config not sent)", tag="raw")
            return
        min_b  = min(255, max(1, round(tick * 0.8 / 0.5)))
        max_b  = min(255, max(1, int(tick * 1.5 + 0.5)))
        can_id = max(1, min(can_id, 0x7FF))
        data = bytes([nibbles, crc_mode, seed & 0xFF, min_b, max_b,
                      (can_id >> 8) & 0xFF, can_id & 0xFF])
        self._send(slcan_frame(ID_CONFIG + ch.idx, data))

        # The config frame only sets RX validation; the TX tick is separate.
        tick_x10 = max(20, min(int(round(tick * 10.0)), 900))   # device range 2.0-90.0 µs
        self._send_ctrl(ch, CMD_TX_TICK, bytes([tick_x10 & 0xFF, tick_x10 >> 8]))

    def _apply_all(self):
        """Copy the selected channel's settings (not its RX CAN ID) to every channel."""
        src = self.sel
        for ch in self.channels[:self.n_ch]:
            if ch is not src:
                ch.preset.set(src.preset.get())
                for key, var in src.params.items():
                    if key != "data_can_id":
                        ch.params[key].set(var.get())
                ch.crc_mode.set(src.crc_mode.get())
            self._send_config(ch)

    def _send_termination(self, ch: Channel):
        if not (self._connected() and self.opened and self.features & FEATURE_TERMINATION):
            return   # sent in _finish_open once the board is known
        self._send_ctrl(ch, CMD_TERM, bytes([TERM_CHOICES.get(ch.term.get(), 2)]))

    def _on_listen_toggle(self):
        if not (self._connected() and self.opened and self.features & FEATURE_TERMINATION):
            return
        # 'O' / 'L' restart RX on every channel, so periodic TX stops too.
        for ch in self.channels:
            self._tx_stop(ch, resume=False)
        self._send(b"L\r" if self.listen_var.get() else b"O\r")
        for ch in self.channels[:self.n_ch]:
            ch.mode = "RX"

    def _start_rx_selected(self):
        ch = self.sel
        self._tx_stop(ch, resume=False)
        self._send_ctrl(ch, CMD_RX)
        ch.mode = "RX"

    def _stop_selected(self):
        ch = self.sel
        self._tx_stop(ch, resume=False)
        self._send_ctrl(ch, CMD_STOP)
        ch.mode = "off"
        ch.learning = False
        self._update_learn_button()

    # ── Learn ─────────────────────────────────────────────────────────────────

    def _learn(self):
        """Start a learn on the selected channel, or cancel it.

        Only one channel can learn at a time (the firmware shares the search
        state).  A config frame ends a learn and restores the previous config.
        """
        if not self._connected():
            return
        ch = self.sel
        if ch.learning:
            ch.learning = False
            ch.learn_result.set("Learn cancelled")
            self._send_config(ch)
            self._send_ctrl(ch, CMD_RX)
            ch.mode = "RX"
        else:
            self._tx_stop(ch, resume=False)
            ch.learning = True
            ch.mode = "learn"
            ch.learn_result.set("Learning…")
            self._learn_sent_at = time.monotonic()
            self._send_ctrl(ch, CMD_LEARN)
        self._update_learn_button()

    def _update_learn_button(self):
        busy = next((c for c in self.channels if c.learning), None)
        if busy is None:
            self.learn_btn.config(text="Learn", state=tk.NORMAL)
        elif busy is self.sel:
            self.learn_btn.config(text="Cancel Learn", state=tk.NORMAL)
        else:
            self.learn_btn.config(text=f"Learn ({busy.name} learning)", state=tk.DISABLED)

    def _on_learn_result(self, ch: Channel, data: bytes):
        ch.learning = False
        if len(data) < 4:
            ch.learn_result.set("Learn: short result")
        else:
            tick_us  = le16(data, 0) / 10.0
            nib      = data[2]
            mode_str = "STATUS_AND_DATA" if data[3] else "DATA_ONLY"
            crc_str = ""
            if len(data) >= 5:
                # byte 4: CRC selector, same encoding as config byte 2
                ch.params["crc_init"].set(f"0x{data[4]:02X}")
                crc_str = "  crc=" + ("legacy" if data[4] & 0x80 else f"0x{data[4]:02X}")
            ch.params["tick_us"].set(f"{tick_us:g}")
            ch.params["nibbles"].set(str(nib))
            ch.crc_mode.set(mode_str)
            ch.learn_result.set(f"tick={tick_us:.1f}us  nibbles={nib}  {mode_str}{crc_str}")
            # The firmware stops the channel after a learn: push the learned
            # format (with this channel's RX ID) and receive again.
            self._send_config(ch)
            self._send_ctrl(ch, CMD_RX)
            ch.mode = "RX"
        self._update_learn_button()

    # ── TX ────────────────────────────────────────────────────────────────────

    def _tx_build_frame(self, ch: Channel) -> bytes | None:
        """TX frame 0x520+0x40n: status, then the nibbles packed big-endian."""
        try:
            status = int(ch.tx_status.get().strip(), 16) & 0x0F
            nib_str = ch.tx_nibbles.get().strip()
            if len(nib_str) == 0 or len(nib_str) > 8:
                return None
            nibs = [int(c, 16) for c in nib_str]
            count = len(nibs)
            packed = sum(n << ((count - 1 - i) * 4) for i, n in enumerate(nibs))
            b = [(packed >> (24 - k * 8)) & 0xFF for k in range(4)]
            return slcan_frame(ID_TX + ID_STRIDE * ch.idx, bytes([status] + b))
        except (ValueError, TypeError):
            return None

    def _tx_frame_ms(self, ch: Channel) -> float:
        """Longest possible TX frame on this channel in ms (sync, status and CRC
        at 27 ticks, data nibbles as entered, the firmware's 12-tick pause).  The
        firmware NACKs a frame while the previous one is still on the line."""
        try:
            tick = max(2.0, min(float(ch.params["tick_us"].get()), 90.0))
        except (ValueError, TypeError):
            tick = 3.0
        n = len(ch.tx_nibbles.get().strip()) or 8
        return (56 + 27 + 27 * n + 27 + 12) * tick / 1000.0

    def _tx_send(self):
        """One-shot TX on the selected channel: TX mode, one frame, RX again.

        The RX command is sent from root.after(), not with the frame: commands
        that arrive in one USB packet are handled back to back, and RX would
        start before the frame (about 1.5 ms) has gone out.
        """
        ch = self.sel
        if not self._connected() or ch.tx_active:
            return
        frame = self._tx_build_frame(ch)
        if frame is None:
            return
        self._send_ctrl(ch, CMD_TX)
        self._send(frame)
        ch.mode = "TX"
        self.root.after(max(25, int(self._tx_frame_ms(ch)) + 5), lambda: self._tx_resume_rx(ch))

    def _tx_resume_rx(self, ch: Channel):
        if self._connected() and not ch.tx_active:
            self._send_ctrl(ch, CMD_RX)
            ch.mode = "RX"

    def _tx_toggle(self):
        ch = self.sel
        if ch.tx_active:
            self._tx_stop(ch)
            return
        if not self._connected():
            return
        if ch.learning:
            ch.learning = False
            ch.learn_result.set("Learn ended by TX")
            self._update_learn_button()
        ch.tx_active = True
        ch.mode = "TX"
        self.tx_toggle_btn.config(text="Stop")
        self._send_ctrl(ch, CMD_TX)
        self._tx_tick(ch)

    def _tx_tick(self, ch: Channel):
        if not ch.tx_active:
            return
        if self._connected():
            frame = self._tx_build_frame(ch)
            if frame:
                self._send(frame)
        try:
            period = max(10, int(ch.tx_period.get()))
        except (ValueError, TypeError):
            period = 100
        period = max(period, int(self._tx_frame_ms(ch)) + 2)    # slow ticks: frame > 10 ms
        ch.tx_after = self.root.after(period, lambda: self._tx_tick(ch))

    def _tx_stop(self, ch: Channel, resume: bool = True):
        """Stop periodic TX on a channel; resume=True switches it back to RX."""
        was_active = ch.tx_active
        ch.tx_active = False
        if ch.tx_after is not None:
            self.root.after_cancel(ch.tx_after)
            ch.tx_after = None
        if ch is self.sel:
            self.tx_toggle_btn.config(text="Start")
        if was_active and resume and self._connected():
            self._send_ctrl(ch, CMD_RX)
            ch.mode = "RX"

    # ── View controls ─────────────────────────────────────────────────────────

    def _toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.config(text="▶  Resume" if self.paused else "⏸  Pause")

    def _clear(self):
        for ch in self.channels:
            ch.reset_stats()
        self._show_last(self.sel)
        self._rows.clear()
        self.tree.delete(*self._tree_iids)
        self._tree_iids.clear()
        self.log.delete("1.0", tk.END)

    def _row_visible(self, ch_idx: int) -> bool:
        f = self.filter_var.get()
        return f == "All" or f == self.channels[ch_idx].name

    def _rebuild_table(self):
        self.tree.delete(*self._tree_iids)
        self._tree_iids.clear()
        for ch_idx, values in self._rows:            # newest first
            if self._row_visible(ch_idx):
                self._tree_iids.append(self.tree.insert("", tk.END, values=values, tags=("ok",)))

    def _show_last(self, ch: Channel):
        self.sv_last_data.set(ch.last_data)
        self.sv_last_nibbles.set(ch.last_nibbles)
        self.sv_last_meta.set(ch.last_meta or f"{ch.name}   no frame yet")

    # ── RX thread ─────────────────────────────────────────────────────────────

    def _rx_loop(self):
        buf = b""
        while self.running:
            try:
                chunk = self.serial_port.read(256)
            except Exception:
                if self.running:
                    self.rx_queue.put(DISCONNECTED)
                break
            if chunk:
                buf += chunk
                # SLCAN terminators: \r ends ACKs, frames and info answers;
                # \a (0x07) is a NACK without \r.  Normalise both to \n.
                parts = (buf.replace(b"\r\n", b"\n")
                            .replace(b"\r",   b"\n")
                            .replace(b"\x07", b"\x07\n")
                            .split(b"\n"))
                buf = parts[-1]
                for raw_line in parts[:-1]:
                    text = raw_line.decode("ascii", errors="replace").strip()
                    if text:
                        self.rx_queue.put(text)

    # ── UI update loop ────────────────────────────────────────────────────────

    def _poll_queue(self):
        # Drain the queue.  With "Limit frame rate" only the newest data frame
        # of each channel is shown per cycle (tkinter cannot keep up with
        # several kHz of table inserts); counters, learn results, device info
        # and the supply frame are always handled.
        drop = self.drop_frames_var.get()
        rx_map = {}
        for ch in self.channels[:self.n_ch]:
            cid = ch.rx_id()
            if cid is not None:
                rx_map.setdefault(cid, ch)
        latest: dict[int, tuple] = {}
        pending: list[tuple] = []
        now = time.monotonic()
        lost_port = False
        try:
            while True:
                line = self.rx_queue.get_nowait()
                if line == DISCONNECTED:
                    lost_port = True
                    break
                f = parse_frame(line)
                if f is None:
                    self._handle_text(line)
                    continue
                cid, data = f
                ch = rx_map.get(cid)
                if ch is not None:
                    ch.times.append(now)
                    if drop:
                        latest[ch.idx] = (ch, line, cid, data)
                    else:
                        pending.append((ch, line, cid, data))
                else:
                    self._handle_aux_frame(line, cid, data)
        except queue.Empty:
            pass

        for item in (latest.values() if drop else pending):
            self._process_data_frame(*item)
        self._update_overview(now)
        if lost_port:
            self._disconnect("Device disconnected")
        self.root.after(self.POLL_MS, self._poll_queue)

    def _handle_text(self, line: str):
        if line in ("z", "Z") and self._silent_acks > 0:
            self._silent_acks -= 1
            return
        if line.startswith("V") and len(line) >= 5:
            self.fw_version = line[1:]
        if line == "\x07" and not self.opened and self._open_after is not None:
            # v1.x firmware NACKs the device-info command: one channel.
            self._append_log(line)
            self._finish_open()
            return
        self._append_log(line)

    def _channel_from_id(self, cid: int, base: int):
        off = cid - base
        if off >= 0 and off % ID_STRIDE == 0 and off // ID_STRIDE < self.n_ch:
            return self.channels[off // ID_STRIDE]
        return None

    def _handle_aux_frame(self, line: str, cid: int, data: bytes):
        """Frames that are not SENT data: counters, learn, info, supply."""
        if cid == ID_INFO:
            if self._silent_info > 0:
                self._silent_info -= 1
            else:
                self._append_log(line)
            self._on_info(data)
            return
        if cid == ID_SUPPLY:
            self._on_supply(data)          # every 500 ms: not logged
            return
        self._append_log(line)
        ch = self._channel_from_id(cid, ID_DIAG)
        if ch is not None and len(data) >= 8:
            ch.frames      = le32(data, 0)
            ch.crc_errors  = le16(data, 4)
            ch.sync_errors = le16(data, 6)
            return
        ch = self._channel_from_id(cid, ID_LOSS)
        if ch is not None and len(data) >= 8:
            ch.lost_edges    = le32(data, 0)
            ch.usb_overflows = le16(data, 4)
            ch.guard_trips   = le16(data, 6)
            return
        if ID_LEARN <= cid < ID_LEARN + self.n_ch:
            self._on_learn_result(self.channels[cid - ID_LEARN], data)

    def _on_info(self, data: bytes):
        if len(data) < 1:
            return
        first = not self.info_seen
        self.info_seen = True
        n = max(1, min(MAX_CHANNELS, data[0]))
        if len(data) >= 7:
            self.features = data[6]
        if len(data) >= 3:
            self.fw_version = f"{data[1]:02X}{data[2]:02X}"
        if n != self.n_ch or first:
            self._set_channel_count(n)
        if len(data) >= 4:
            for ch in self.channels[:n]:
                if not ch.tx_active:   # periodic TX flips between frames
                    ch.mode = MODE_NAMES[(data[3] >> (2 * ch.idx)) & 0x03]
        if len(data) >= 5 and time.monotonic() - self._learn_sent_at > 1.5:
            for ch in self.channels[:n]:
                learning = data[4] == ch.idx
                if ch.learning and not learning:
                    ch.learn_result.set("Learn ended")
                ch.learning = learning
            self._update_learn_button()
        if len(data) >= 6 and self.features & FEATURE_TERMINATION:
            for ch in self.channels[:n]:
                ch.term_on = bool(data[5] & (1 << ch.idx))
        self.sv_device.set(f"FW {self.fw_version}  ·  {n} channel{'s' if n > 1 else ''}")
        if not self.opened:
            self._finish_open()
        elif first:
            # The answer came after the open timeout: the device was opened
            # as one channel.  'O' already started every channel; push the
            # other channels' settings and start polling.
            self._append_log("(device info arrived late: configuring all channels)", tag="dbg")
            if self.listen_var.get() and self.features & FEATURE_TERMINATION:
                self._send(b"L\r")
            for ch in self.channels[1:n]:
                ch.mode = "RX"
                self._send_config(ch)
            for ch in self.channels[:n]:
                if self.features & FEATURE_TERMINATION and ch.term.get() != "Auto":
                    self._send_termination(ch)
            self._schedule_info_poll()

    def _on_supply(self, data: bytes):
        """0x60E: VS, USB-path enable, load-switch fault, current, source."""
        if len(data) < 3:
            return
        vs = le16(data, 0) / 1000.0
        flags = data[2]
        parts = [f"Sensor VS {vs:5.2f} V"]
        if len(data) >= 5 and le16(data, 3) != 0xFFFF:
            parts.append(f"{le16(data, 3)} mA")
        if len(data) >= 6 and data[5] != 0xFF:
            parts.append("from EXT" if data[5] else "from USB")
        if flags & 0x02:
            parts.append("USB SUPPLY FAULT (short / overload)")
            color = "#FF6666"
        elif vs > 5.5:
            parts.append("EXT > 5 V on every VS pin")
            color = "#FFAA44"
        else:
            color = "#88DD88"
        self.sv_supply.set("   ".join(parts))
        self.supply_lbl.configure(foreground=color)

    def _process_data_frame(self, ch: Channel, line: str, cid: int, data: bytes):
        self._append_log(line)
        if self.paused:
            return

        # RX layout: data nibbles packed in ceil(n/2) bytes, then an info byte
        # at index max(3, ceil(n/2)) (status nibble in bits 7..4, slow-channel
        # flags in bits 2..0), optionally slow-channel ID and 16-bit data.
        try:
            n_cfg = int(ch.params["nibbles"].get())
        except ValueError:
            n_cfg = 6
        n_bytes = (n_cfg + 1) // 2
        info_idx = max(3, n_bytes)
        nibbles = []
        for b in data[:n_bytes]:
            nibbles.append((b >> 4) & 0x0F)
            nibbles.append(b & 0x0F)
        nibbles = nibbles[:n_cfg]

        data_hex    = "  ".join(f"{b:02X}" for b in data)
        nibbles_str = "  ".join(f"{n:X}" for n in nibbles)
        if len(data) > info_idx:
            info = data[info_idx]
            nibbles_str += f"   st={info >> 4:X}"
            if (info & 0x01) and len(data) >= info_idx + 4:
                kind = "ESM16" if info & 0x04 else ("ESM" if info & 0x02 else "SSM")
                slow_val = data[info_idx + 2] | (data[info_idx + 3] << 8)
                nibbles_str += f"   {kind} id={data[info_idx + 1]:02X} data={slow_val:X}"

        ch.last_data = data_hex
        ch.last_nibbles = nibbles_str
        ch.last_meta = f"{ch.name}   ID: {cid:03X}    DLC: {len(data)}"
        if ch is self.sel:
            self._show_last(ch)

        ts = datetime.now().strftime("%H:%M:%S.%f")[:-4]
        values = (ts, ch.name, f"{cid:03X}", len(data), data_hex, nibbles_str)
        self._rows.appendleft((ch.idx, values))
        if len(self._rows) > self.MAX_TABLE_ROWS:
            self._rows.pop()
        if self._row_visible(ch.idx):
            self._tree_iids.appendleft(self.tree.insert("", 0, values=values, tags=("ok",)))
            if len(self._tree_iids) > self.MAX_TABLE_ROWS:
                self.tree.delete(self._tree_iids.pop())

        if self.recording and self.log_file:
            try:
                self.log_file.write(",".join(self._csv_quote(v) for v in values) + "\n")
            except OSError:
                self._stop_recording()

    def _update_overview(self, now: float):
        for ch in self.channels[:self.n_ch]:
            while ch.times and now - ch.times[0] > 2.0:
                ch.times.popleft()
            setting = ch.term.get()
            if ch.term_on is None:
                term = "—" if self.info_seen or self.opened else setting.lower()
            else:
                term = ("on" if ch.term_on else "off") + ("" if setting == "Auto" else " (forced)")
            lost = ch.lost_edges + ch.usb_overflows
            iid = f"ch{ch.idx}"
            if self.overview.exists(iid):
                self.overview.item(iid, values=(
                    ch.name, ch.mode, term, ch.last_nibbles,
                    ch.frames, ch.crc_errors, ch.sync_errors, lost,
                    f"{len(ch.times) / 2.0:.1f}"))

    # ── Export / record / copy helpers ────────────────────────────────────────

    CSV_HEADER = "time,channel,can_id,dlc,data_hex,nibbles\n"

    @staticmethod
    def _csv_quote(value) -> str:
        s = str(value)
        if any(c in s for c in (",", '"', "\n", "\r")):
            return '"' + s.replace('"', '""') + '"'
        return s

    def _save_log_dialog(self):
        """Write the frames shown in the table to a CSV file (oldest first)."""
        if not self._tree_iids:
            messagebox.showinfo("Save Log", "No frames to save.")
            return
        path = filedialog.asksaveasfilename(
            title="Save received frames", defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("Text files", "*.txt"), ("All files", "*.*")],
            initialfile=f"sent_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write(self.CSV_HEADER)
                for iid in reversed(self._tree_iids):
                    vals = self.tree.item(iid, "values")
                    f.write(",".join(self._csv_quote(v) for v in vals) + "\n")
        except OSError as e:
            messagebox.showerror("Save Log", str(e))

    def _save_raw_log_dialog(self):
        text = self.log.get("1.0", "end-1c")
        if not text:
            messagebox.showinfo("Save Raw Log", "Log is empty.")
            return
        path = filedialog.asksaveasfilename(
            title="Save raw log", defaultextension=".log",
            filetypes=[("Log files", "*.log"), ("Text files", "*.txt"), ("All files", "*.*")],
            initialfile=f"sent_raw_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write(text)
        except OSError as e:
            messagebox.showerror("Save Raw Log", str(e))

    def _toggle_recording(self):
        """Start or stop streaming every decoded frame (all channels) to a CSV file."""
        if self.recording:
            self._stop_recording()
            return
        path = filedialog.asksaveasfilename(
            title="Record frames to file (streaming)", defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialfile=f"sent_record_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        if not path:
            return
        try:
            self.log_file = open(path, "w", encoding="utf-8", buffering=1)
            self.log_file.write(self.CSV_HEADER)
            self.recording = True
            self.record_btn.config(text="■ Stop Recording")
        except OSError as e:
            messagebox.showerror("Record", str(e))
            self.log_file = None

    def _stop_recording(self):
        if self.log_file:
            try:
                self.log_file.close()
            except OSError:
                pass
        self.log_file = None
        if self.recording:
            self.recording = False
            self.record_btn.config(text="● Record…")

    def _copy_tree_selection(self, event=None):
        sel = self.tree.selection()
        if not sel:
            return "break"
        ordered = sorted(sel, key=self.tree.index)
        lines = ["\t".join(str(v) for v in self.tree.item(iid, "values")) for iid in ordered]
        self.root.clipboard_clear()
        self.root.clipboard_append("\n".join(lines))
        return "break"

    def _copy_tree_all(self):
        if not self._tree_iids:
            return
        lines = ["\t".join(str(v) for v in self.tree.item(iid, "values"))
                 for iid in reversed(self._tree_iids)]
        self.root.clipboard_clear()
        self.root.clipboard_append("\n".join(lines))

    def _select_all_tree(self, event=None):
        self.tree.selection_set(*self.tree.get_children())
        return "break"

    def _show_tree_menu(self, event):
        iid = self.tree.identify_row(event.y)
        if iid and iid not in self.tree.selection():
            self.tree.selection_set(iid)
        self._tree_menu.tk_popup(event.x_root, event.y_root)

    def _log_block_edit(self, event):
        if event.state & 0x4:   # Control held: allow Ctrl+C / Ctrl+A
            return None
        nav_keys = {"Left", "Right", "Up", "Down", "Home", "End", "Prior", "Next",
                    "Shift_L", "Shift_R", "Control_L", "Control_R",
                    "Alt_L", "Alt_R", "Tab", "ISO_Left_Tab"}
        if event.keysym in nav_keys:
            return None
        return "break"

    def _select_all_log(self, event=None):
        self.log.tag_add("sel", "1.0", "end-1c")
        return "break"

    def _copy_log_selection(self):
        try:
            text = self.log.get("sel.first", "sel.last")
        except tk.TclError:
            return
        self.root.clipboard_clear()
        self.root.clipboard_append(text)

    def _copy_log_all(self):
        text = self.log.get("1.0", "end-1c")
        if text:
            self.root.clipboard_clear()
            self.root.clipboard_append(text)

    def _append_log(self, text: str, tag: str | None = None):
        if tag is None:
            if re.match(r'^t[0-9A-Fa-f]', text):
                tag = "slcan"
            elif re.match(r'^[zZ]$', text):
                tag = "other"   # frame ACK
            elif re.match(r'^[VvNF]', text):
                tag = "info"    # version / serial / status response
            elif '\x07' in text:
                tag = "raw"     # NACK
                text = "\\a (NACK)"
            else:
                tag = "other"
        self.log.insert(tk.END, text + "\n", tag)
        self.log.see(tk.END)
        lines = int(self.log.index(tk.END).split(".")[0])
        if lines > 600:
            self.log.delete("1.0", f"{lines - 500}.0")


# ─── Entry point ──────────────────────────────────────────────────────────────

def main():
    root = tk.Tk()
    app = SentMonitor(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app._disconnect(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
