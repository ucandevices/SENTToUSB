#!/usr/bin/env python3
"""
SENT Sensor Monitor — generic SLCAN/SENT frame viewer
Connects to STM32F042 USB CDC and displays decoded SENT frames.

Output format from firmware:
  tXXXDHH…  — SLCAN standard frame (CAN ID 3 hex, DLC 1 hex, data bytes)
  z\r / Z\r  — SLCAN frame ACK
  \r         — SLCAN command ACK
  \a         — SLCAN NACK

Requirements:  pip install pyserial
"""

import tkinter as tk
from tkinter import ttk, font
import serial
import serial.tools.list_ports
import threading
import queue
import time
import re
from collections import deque
from datetime import datetime


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
        "data_can_id": "0x510",
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
        "data_can_id": "0x510",
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
        "data_can_id": "0x510",
        "info": [
            ("Type:",    "4-bar MAP sensor (GM Duramax / 2.0T)"),
            ("Output:",  "sig0 + sig1 redundant pressure (12-bit)"),
            ("CRC:",     "GM DATA_ONLY variant, seed 0x05"),
        ],
    },
}


# ─── Main application ─────────────────────────────────────────────────────────

class SentMonitor:
    MAX_TABLE_ROWS = 100   # rows kept in the frame table
    POLL_MS        = 40    # UI update interval

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("SENT Sensor Monitor")
        self.root.geometry("1100x680")
        self.root.configure(bg="#1e1e2e")

        self.serial_port: serial.Serial | None = None
        self.rx_thread: threading.Thread | None = None
        self.running = False
        self.paused  = False
        self.rx_queue: queue.Queue[str] = queue.Queue()

        self.frame_count  = 0
        self.crc_errors   = 0
        self.sync_errors  = 0
        self._frame_times: list[float] = []
        self._tree_iids: deque = deque()   # iids of rows in the frame table, newest first

        self._build_ui()
        self._refresh_ports()
        self._poll_queue()

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
        style.map("Treeview", background=[("selected","#3a3a5e")])
        # Combobox dropdown listbox colours (Tk classic widget inside ttk)
        self.root.option_add("*TCombobox*Listbox.background",       "#2a2a3e")
        self.root.option_add("*TCombobox*Listbox.foreground",       "#CCCCDD")
        self.root.option_add("*TCombobox*Listbox.selectBackground", "#4a4a6e")
        self.root.option_add("*TCombobox*Listbox.selectForeground", "#FFFFFF")
        self.root.option_add("*TCombobox*Listbox.font",             "Consolas 9")

        paned = tk.PanedWindow(self.root, orient=tk.HORIZONTAL,
                               sashwidth=5, sashrelief=tk.FLAT,
                               bg="#1e1e2e")
        paned.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        left  = ttk.Frame(paned, width=240)
        right = ttk.Frame(paned)
        paned.add(left,  minsize=220)
        paned.add(right, minsize=600)

        self._build_left(left)
        self._build_right(right)

    def _build_left(self, parent):
        # ── Serial port ──
        pf = ttk.LabelFrame(parent, text=" Serial Port ", padding=8)
        pf.pack(fill=tk.X, padx=6, pady=(6,3))

        ttk.Label(pf, text="Port:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(pf, textvariable=self.port_var, width=11, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky=tk.EW, padx=(4,0))
        ttk.Button(pf, text="⟳", width=3, command=self._refresh_ports).grid(row=0, column=2, padx=(3,0))
        pf.columnconfigure(1, weight=1)

        btn_row = ttk.Frame(pf)
        btn_row.grid(row=1, column=0, columnspan=3, sticky=tk.EW, pady=(6,0))
        self.connect_btn    = ttk.Button(btn_row, text="Connect",    command=self._connect)
        self.disconnect_btn = ttk.Button(btn_row, text="Disconnect", command=self._disconnect,
                                         state=tk.DISABLED)
        self.connect_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0,2))
        self.disconnect_btn.pack(side=tk.LEFT, expand=True, fill=tk.X)

        self.status_lbl = ttk.Label(pf, text="● Disconnected", foreground="#FF6666")
        self.status_lbl.grid(row=2, column=0, columnspan=3, pady=(5,0))

        # ── SENT Parameters ──
        sf = ttk.LabelFrame(parent, text=" SENT Parameters ", padding=8)
        sf.pack(fill=tk.X, padx=6, pady=3)

        # Preset selector (row 0)
        ttk.Label(sf, text="Preset:").grid(row=0, column=0, sticky=tk.W, pady=(1,4))
        self.preset_var = tk.StringVar(value="MLX90377")
        preset_cb = ttk.Combobox(sf, textvariable=self.preset_var, width=13,
                                  values=list(SENSOR_PRESETS.keys()), state="readonly")
        preset_cb.grid(row=0, column=1, sticky=tk.EW, padx=(4,0), pady=(1,4))
        preset_cb.bind("<<ComboboxSelected>>", lambda _e: self._apply_preset())

        params = [
            ("Tick Size (µs):",  "tick_us",     "3"),
            ("Sync Ticks:",      "sync_ticks",  "56"),
            ("Min Sync (µs):",   "sync_min_us", "100"),
            ("Data Nibbles:",    "nibbles",     "6"),
            ("CRC Polynomial:",  "crc_poly",    "0x0D"),
            ("CRC Init Seed:",   "crc_init",    "0x03"),
        ]
        self.param_vars: dict[str, tk.StringVar] = {}
        for i, (lbl, key, default) in enumerate(params):
            ttk.Label(sf, text=lbl).grid(row=i + 1, column=0, sticky=tk.W, pady=1)
            v = tk.StringVar(value=default)
            self.param_vars[key] = v
            ttk.Entry(sf, textvariable=v, width=9).grid(row=i + 1, column=1,
                                                         sticky=tk.EW, padx=(4,0))

        n = len(params) + 1
        ttk.Label(sf, text="CRC Mode:").grid(row=n, column=0, sticky=tk.W, pady=1)
        self.crc_mode_var = tk.StringVar(value="DATA_ONLY")
        ttk.Combobox(sf, textvariable=self.crc_mode_var, width=13,
                     values=["DATA_ONLY", "STATUS_AND_DATA"],
                     state="readonly").grid(row=n, column=1, sticky=tk.EW, padx=(4,0))
        sf.columnconfigure(1, weight=1)

        # ── Sensor info (updates when preset changes) ──
        self._info_frame = ttk.LabelFrame(parent, text=" Sensor Info ", padding=8)
        self._info_frame.pack(fill=tk.X, padx=6, pady=3)
        self._info_labels: list[tuple[ttk.Label, ttk.Label]] = []
        for i in range(3):
            k_lbl = ttk.Label(self._info_frame, text="")
            v_lbl = ttk.Label(self._info_frame, text="", foreground="#888899")
            k_lbl.grid(row=i, column=0, sticky=tk.W, pady=1)
            v_lbl.grid(row=i, column=1, sticky=tk.W, padx=4)
            self._info_labels.append((k_lbl, v_lbl))
        self._apply_preset()   # populate from default preset

        # ── RX CAN ID ──
        rf = ttk.LabelFrame(parent, text=" RX CAN ID ", padding=8)
        rf.pack(fill=tk.X, padx=6, pady=3)
        self.param_vars["data_can_id"] = tk.StringVar(value="0x510")
        ttk.Entry(rf, textvariable=self.param_vars["data_can_id"],
                  width=9).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Label(rf, text=" (hex)", foreground="#888899").pack(side=tk.LEFT)

        # ── Control buttons ──
        ctrl = ttk.Frame(parent)
        ctrl.pack(fill=tk.X, padx=6, pady=(6,4))
        ttk.Button(ctrl, text="Apply Config", command=self._send_config).pack(fill=tk.X, pady=2)
        self.learn_btn = ttk.Button(ctrl, text="Learn", command=self._learn)
        self.learn_btn.pack(fill=tk.X, pady=2)
        self.learn_result_var = tk.StringVar(value="")
        self.learn_result_lbl = ttk.Label(ctrl, textvariable=self.learn_result_var,
                                          foreground="#88FFAA", wraplength=190,
                                          font=("Consolas", 8))
        self.learn_result_lbl.pack(fill=tk.X, pady=(0,4))
        self.pause_btn = ttk.Button(ctrl, text="Pause", command=self._toggle_pause)
        self.pause_btn.pack(fill=tk.X, pady=2)
        ttk.Button(ctrl, text="Clear", command=self._clear).pack(fill=tk.X, pady=2)

    def _apply_preset(self):
        name = self.preset_var.get()
        p = SENSOR_PRESETS.get(name)
        if p is None:
            return
        for key, val in p.items():
            if key == "crc_mode":
                self.crc_mode_var.set(val)
            elif key == "info":
                for i, (k_lbl, v_lbl) in enumerate(self._info_labels):
                    if i < len(val):
                        k_lbl.config(text=val[i][0])
                        v_lbl.config(text=val[i][1])
                    else:
                        k_lbl.config(text="")
                        v_lbl.config(text="")
            elif key in self.param_vars:
                self.param_vars[key].set(val)

    def _build_right(self, parent):
        # ── Top: stats bar ──
        stats = ttk.Frame(parent)
        stats.pack(fill=tk.X, padx=6, pady=(6,2))

        self.sv_frames = tk.StringVar(value="Frames: 0")
        self.sv_crc    = tk.StringVar(value="CRC Errors: 0")
        self.sv_sync   = tk.StringVar(value="Sync Errors: 0")
        self.sv_rate   = tk.StringVar(value="Rate: — fps")

        lbl_style = {"relief": tk.SUNKEN, "padding": (8,3),
                     "font": ("Consolas", 10)}
        for var, fg in [(self.sv_frames, "#CCCCDD"),
                        (self.sv_crc,    "#FF8888"),
                        (self.sv_sync,   "#FFAA66"),
                        (self.sv_rate,   "#88CCFF")]:
            ttk.Label(stats, textvariable=var, foreground=fg, **lbl_style).pack(
                side=tk.LEFT, padx=3)

        # ── Middle: RX display left, TX panel centred in remaining space ──
        mid = ttk.Frame(parent)
        mid.pack(fill=tk.X, padx=6, pady=(4,2))

        big_font  = font.Font(family="Consolas", size=28, weight="bold")
        med_font  = font.Font(family="Consolas", size=13)
        info_font = font.Font(family="Consolas", size=10)

        # ── Last received frame — anchored left ──
        rx_disp = ttk.Frame(mid)
        rx_disp.pack(side=tk.LEFT, fill=tk.Y)

        self.tx_periodic_active = False
        self._tx_after_id: str | None = None

        # ── TX LabelFrame — centred in remaining space ──
        tx_wrapper = ttk.Frame(mid)
        tx_wrapper.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        tx_lf = ttk.LabelFrame(tx_wrapper, text=" TX SENT Frame ", padding=(6,4))
        tx_lf.pack(anchor=tk.CENTER, expand=True)

        tx_grid = ttk.Frame(tx_lf)
        tx_grid.pack(anchor=tk.W)

        ttk.Label(tx_grid, text="Status:", font=("Consolas", 9)).grid(
            row=0, column=0, sticky=tk.E, padx=(0,4), pady=2)
        self.tx_status_var = tk.StringVar(value="0")
        ttk.Entry(tx_grid, textvariable=self.tx_status_var, width=3,
                  font=("Consolas", 10)).grid(row=0, column=1, sticky=tk.W)
        ttk.Label(tx_grid, text="(hex nibble)", font=("Consolas", 8),
                  foreground="#666688").grid(row=0, column=2, sticky=tk.W, padx=4)

        ttk.Label(tx_grid, text="Nibbles:", font=("Consolas", 9)).grid(
            row=1, column=0, sticky=tk.E, padx=(0,4), pady=2)
        self.tx_nibbles_var = tk.StringVar(value="123456")
        ttk.Entry(tx_grid, textvariable=self.tx_nibbles_var, width=10,
                  font=("Consolas", 10)).grid(row=1, column=1, sticky=tk.W)
        ttk.Label(tx_grid, text="(hex, e.g. 123456)", font=("Consolas", 8),
                  foreground="#666688").grid(row=1, column=2, sticky=tk.W, padx=4)

        ttk.Label(tx_grid, text="Period:", font=("Consolas", 9)).grid(
            row=2, column=0, sticky=tk.E, padx=(0,4), pady=2)
        self.tx_period_var = tk.StringVar(value="100")
        ttk.Entry(tx_grid, textvariable=self.tx_period_var, width=6,
                  font=("Consolas", 10)).grid(row=2, column=1, sticky=tk.W)
        ttk.Label(tx_grid, text="ms  (0 = one-shot)", font=("Consolas", 8),
                  foreground="#666688").grid(row=2, column=2, sticky=tk.W, padx=4)

        tx_btn_row = ttk.Frame(tx_lf)
        tx_btn_row.pack(anchor=tk.W, pady=(4,0))
        self.tx_send_btn = ttk.Button(tx_btn_row, text="Send", width=7,
                                      command=self._tx_send)
        self.tx_send_btn.pack(side=tk.LEFT, padx=(0,4))
        self.tx_toggle_btn = ttk.Button(tx_btn_row, text="Start", width=7,
                                        command=self._tx_toggle)
        self.tx_toggle_btn.pack(side=tk.LEFT)

        self.sv_last_data    = tk.StringVar(value="—  —  —")
        self.sv_last_nibbles = tk.StringVar(value="—  —  —  —  —  —")
        self.sv_last_meta    = tk.StringVar(value="ID: —    DLC: —")

        tk.Label(rx_disp, textvariable=self.sv_last_data,
                 font=big_font, fg="#00BFFF", bg="#1e1e2e",
                 anchor=tk.W).pack(fill=tk.X)
        tk.Label(rx_disp, textvariable=self.sv_last_nibbles,
                 font=med_font, fg="#4488CC", bg="#1e1e2e",
                 anchor=tk.W).pack(fill=tk.X)
        tk.Label(rx_disp, textvariable=self.sv_last_meta,
                 font=info_font, fg="#888899", bg="#1e1e2e",
                 anchor=tk.W).pack(fill=tk.X, pady=(2,0))

        # ── Frame table ──
        tf = ttk.LabelFrame(parent, text=" Received Frames ", padding=4)
        tf.pack(fill=tk.BOTH, expand=True, padx=6, pady=2)

        cols = ("time", "can_id", "dlc", "data_hex", "nibbles", "slcan")
        self.tree = ttk.Treeview(tf, columns=cols, show="headings", height=8,
                                  selectmode="browse")
        headers = {
            "time":     ("Time",       75, tk.CENTER),
            "can_id":   ("CAN ID",     65, tk.CENTER),
            "dlc":      ("DLC",        35, tk.CENTER),
            "data_hex": ("Data (hex)", 110, tk.CENTER),
            "nibbles":  ("Nibbles",   155, tk.CENTER),
            "slcan":    ("SLCAN",     160, tk.W),
        }
        for col in cols:
            h, w, anchor = headers[col]
            self.tree.heading(col, text=h)
            self.tree.column(col, width=w, minwidth=30, anchor=anchor, stretch=(col=="slcan"))

        ysb = ttk.Scrollbar(tf, orient=tk.VERTICAL,   command=self.tree.yview)
        xsb = ttk.Scrollbar(tf, orient=tk.HORIZONTAL, command=self.tree.xview)
        self.tree.configure(yscroll=ysb.set, xscroll=xsb.set)

        ysb.pack(side=tk.RIGHT, fill=tk.Y)
        xsb.pack(side=tk.BOTTOM, fill=tk.X)
        self.tree.pack(fill=tk.BOTH, expand=True)

        self.tree.tag_configure("error", foreground="#FF6666")
        self.tree.tag_configure("ok",    foreground="#88CC88")

        # ── Raw log ──
        lf = ttk.LabelFrame(parent, text=" Raw Log ", padding=4)
        lf.pack(fill=tk.X, padx=6, pady=(2,6))

        self.log = tk.Text(lf, height=5, font=("Consolas", 9),
                           bg="#131320", fg="#888899",
                           insertbackground="#AAAACC", state=tk.DISABLED,
                           wrap=tk.NONE)
        log_ysb = ttk.Scrollbar(lf, orient=tk.VERTICAL,   command=self.log.yview)
        log_xsb = ttk.Scrollbar(lf, orient=tk.HORIZONTAL, command=self.log.xview)
        self.log.configure(yscrollcommand=log_ysb.set, xscrollcommand=log_xsb.set)
        log_ysb.pack(side=tk.RIGHT, fill=tk.Y)
        log_xsb.pack(side=tk.BOTTOM, fill=tk.X)
        self.log.pack(fill=tk.BOTH)

        # Color tags for log
        self.log.tag_config("info",   foreground="#00BFFF")   # V/v/N/F responses
        self.log.tag_config("dbg",   foreground="#FFAA44")   # command ACK (\r)
        self.log.tag_config("raw",   foreground="#FF6666")   # NACK (\a)
        self.log.tag_config("slcan", foreground="#666688")   # data frame t/T
        self.log.tag_config("other", foreground="#555566")   # z/Z frame ACK
        self.log.tag_config("tx_cmd", foreground="#55FF55")  # outgoing commands → host

    # ── Serial port helpers ───────────────────────────────────────────────────

    def _refresh_ports(self):
        ports = [p.device for p in sorted(serial.tools.list_ports.comports(),
                                           key=lambda p: p.device)]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            preferred = next((p for p in ports if "COM8" in p), ports[0])
            self.port_var.set(preferred)

    def _connect(self):
        port = self.port_var.get()
        try:
            self.serial_port = serial.Serial(port, 115200, timeout=0.1)
            self.serial_port.reset_input_buffer()  # discard stale OS-buffered data
            self.running = True
            self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.rx_thread.start()
            # SLCAN: send 'O\r' to open channel — firmware starts SENT RX only after this
            self._send(b"O\r")
            self._send_config()
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            self.status_lbl.config(text=f"● Connected  {port}", foreground="#66CC66")
        except Exception as e:
            self.status_lbl.config(text=f"● {e}", foreground="#FF6666")

    def _learn(self):
        """Send learn command (0x600 byte=0x04) and wait for 0x601 result."""
        if not self.serial_port or not self.serial_port.is_open:
            return
        self.learn_result_var.set("Learning...")
        self.learn_btn.config(state=tk.DISABLED)
        self._send(b"t600104\r")

    def _send_config(self):
        """Build and send a 0x001 config frame with the current UI parameters.

        Frame layout (7 bytes, DLC=7):
          B0 : data nibbles (4 / 6 / 8)
          B1 : CRC mode     (0=DATA_ONLY, 1=STATUS_AND_DATA)
          B2 : CRC init seed (e.g. 0x03 / 0x05)
          B3 : min tick, units 0.5 µs  (= tick * 0.8 / 0.5)
          B4 : max tick, units 1 µs    (= tick * 1.5)
          B5 : output RX CAN ID, low byte  (little-endian)
          B6 : output RX CAN ID, high byte
        """
        if not self.serial_port or not self.serial_port.is_open:
            return
        try:
            nibbles  = int(self.param_vars["nibbles"].get())
            crc_mode = 1 if self.crc_mode_var.get() == "STATUS_AND_DATA" else 0
            seed     = int(self.param_vars["crc_init"].get(), 0)
            tick     = float(self.param_vars["tick_us"].get())
            can_id   = int(self.param_vars["data_can_id"].get(), 0)

            min_b = max(1, round(tick * 0.8 / 0.5))   # 80 % of tick in 0.5 µs units
            max_b = max(1, round(tick * 1.5))           # 150 % of tick in 1 µs units
            min_b  = min(min_b, 255)
            max_b  = min(max_b, 255)
            can_id = max(1, min(can_id, 0x7FF))

            data = bytes([nibbles, crc_mode, seed, min_b, max_b,
                          (can_id >> 8) & 0xFF, can_id & 0xFF])
            slcan = "t001" + "7" + data.hex().upper() + "\r"
            self._send(slcan.encode("ascii"))
        except (ValueError, TypeError):
            pass

    def _tx_build_frame(self) -> bytes | None:
        """Build a 0x520 TX SLCAN frame from current UI values, or None on error."""
        try:
            status = int(self.tx_status_var.get().strip(), 16) & 0x0F
            nib_str = self.tx_nibbles_var.get().strip()
            if len(nib_str) == 0 or len(nib_str) > 8:
                return None
            nibs = [int(c, 16) for c in nib_str]
            count = len(nibs)
            packed = sum(n << ((count - 1 - i) * 4) for i, n in enumerate(nibs))
            b = [(packed >> (24 - k * 8)) & 0xFF for k in range(4)]
            data = bytes([status, b[0], b[1], b[2], b[3]])
            frame = "t520" + "5" + data.hex().upper() + "\r"
            return frame.encode("ascii")
        except (ValueError, TypeError):
            return None

    def _tx_send(self):
        """One-shot TX: switch to TX mode, send one frame, then resume RX after a delay.

        The three commands (start TX / frame / start RX) must NOT be sent back-to-back
        in the same call: they would land in one USB packet, be processed inside a single
        USB interrupt, and the "resume RX" command would arrive before TIM14 has had a
        chance to fire and drain the TX queue.  The TIM14 ISR runs at lower priority than
        USB, so it can only start after the USB interrupt returns.

        Fix: send the resume-RX command via root.after() so it is deferred by at least
        one Tkinter event-loop iteration (25 ms), well past the ~1.5 ms a SENT frame takes.
        """
        if not self.serial_port or not self.serial_port.is_open:
            return
        if self.tx_periodic_active:
            return   # periodic TX is already running; ignore one-shot
        frame = self._tx_build_frame()
        if frame is None:
            return
        self._send(b"t600102\r")   # switch to TX mode
        self._send(frame)           # queue one SENT frame
        # Resume RX after 25 ms — enough for TIM14 to finish the frame (~1.5 ms max)
        self.root.after(25, self._tx_resume_rx)

    def _tx_resume_rx(self):
        """Deferred RX resume after a one-shot TX send."""
        if self.serial_port and self.serial_port.is_open and not self.tx_periodic_active:
            self._send(b"t600101\r")   # switch back to RX mode

    def _tx_toggle(self):
        """Start or stop periodic TX."""
        if not self.tx_periodic_active:
            if not self.serial_port or not self.serial_port.is_open:
                return
            self.tx_periodic_active = True
            self.tx_toggle_btn.config(text="Stop")
            self._send(b"t600102\r")   # switch to TX mode
            self._tx_tick()
        else:
            self._tx_stop()

    def _tx_tick(self):
        """Periodic callback — sends one TX frame then reschedules itself."""
        if not self.tx_periodic_active:
            return
        if self.serial_port and self.serial_port.is_open:
            frame = self._tx_build_frame()
            if frame:
                self._send(frame)
        try:
            period = max(10, int(self.tx_period_var.get()))
        except (ValueError, TypeError):
            period = 100
        self._tx_after_id = self.root.after(period, self._tx_tick)

    def _tx_stop(self):
        """Stop periodic TX and resume RX mode."""
        self.tx_periodic_active = False
        self.tx_toggle_btn.config(text="Start")
        if self._tx_after_id is not None:
            self.root.after_cancel(self._tx_after_id)
            self._tx_after_id = None
        if self.serial_port and self.serial_port.is_open:
            self._send(b"t600101\r")   # resume RX mode

    def _disconnect(self):
        self._tx_stop()
        self.running = False
        if self.serial_port:
            try:
                # SLCAN: send 'C\r' to close channel before disconnecting
                self._send(b"C\r")
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)
        self.learn_btn.config(state=tk.NORMAL)
        self.learn_result_var.set("")
        self.status_lbl.config(text="● Disconnected", foreground="#FF6666")

    def _toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.config(text="▶  Resume" if self.paused else "⏸  Pause")

    def _clear(self):
        self.frame_count = self.crc_errors = self.sync_errors = 0
        self._frame_times.clear()
        self.sv_frames.set("Frames: 0")
        self.sv_crc.set("CRC Errors: 0")
        self.sv_sync.set("Sync Errors: 0")
        self.sv_rate.set("Rate: — fps")
        self.sv_last_data.set("—  —  —")
        self.sv_last_nibbles.set("—  —  —  —  —  —")
        self.sv_last_meta.set("ID: —    DLC: —")
        self.tree.delete(*self._tree_iids)
        self._tree_iids.clear()
        self.log.config(state=tk.NORMAL)
        self.log.delete("1.0", tk.END)
        self.log.config(state=tk.DISABLED)

    # ── RX thread ─────────────────────────────────────────────────────────────

    def _rx_loop(self):
        buf = b""
        while self.running:
            try:
                chunk = self.serial_port.read(256)
                if chunk:
                    buf += chunk
                    # SLCAN terminators:
                    #   \r  — ACK / data frame / info response end
                    #   \a  — NACK (bell, 0x07), no trailing \r
                    # Normalise all to \n so split() works uniformly.
                    parts = (buf.replace(b"\r\n", b"\n")
                               .replace(b"\r",   b"\n")
                               .replace(b"\x07", b"\x07\n")  # keep \a as its own token
                               .split(b"\n"))
                    buf = parts[-1]  # keep incomplete trailing fragment
                    for raw_line in parts[:-1]:
                        text = raw_line.decode("ascii", errors="replace").strip()
                        if text:
                            self.rx_queue.put(text)
            except Exception:
                break

    # ── UI update loop ────────────────────────────────────────────────────────

    def _poll_queue(self):
        try:
            # Drain the entire queue. For data frames keep only the LATEST to avoid
            # tkinter display lag at 300+ fps. All other SLCAN lines are logged as-is.
            # Count ALL data frames for fps/frame-count; display only newest.
            last_data: str | None = None
            now = time.monotonic()
            while True:
                line = self.rx_queue.get_nowait()
                if re.match(r'^t[0-9A-Fa-f]', line):
                    last_data = line   # keep only newest data frame for display
                    self.frame_count += 1
                    self._frame_times.append(now)
                else:
                    # ACKs (\r → shown as blank), NACKs (\a), z/Z, V/v/N/F responses
                    self._append_log(line)
        except queue.Empty:
            pass

        # Update rate display using all received frames (not just displayed ones)
        now = time.monotonic()
        self._frame_times = [t for t in self._frame_times if now - t <= 2.0]
        rate = len(self._frame_times) / 2.0
        self.sv_frames.set(f"Frames: {self.frame_count}")
        self.sv_rate.set(f"Rate: {rate:.1f} fps")

        if last_data is not None:
            self._process_line(last_data)
        self.root.after(self.POLL_MS, self._poll_queue)

    def _process_line(self, line: str):
        self._append_log(line)

        if self.paused:
            return

        # ── Parse SLCAN standard frame (t only; T extended not used by firmware) ──
        fm = re.match(r'^t([0-9A-Fa-f]{3})([0-9A-Fa-f])([0-9A-Fa-f]*)', line)
        if not fm:
            return

        try:
            cid = int(fm.group(1), 16)
            dlc = int(fm.group(2), 16)
        except ValueError:
            return

        # ── 0x601 learn result ────────────────────────────────────────────────
        if cid == 0x601:
            expected_hex = dlc * 2
            if len(fm.group(3)) >= expected_hex and dlc >= 4:
                try:
                    data = bytes.fromhex(fm.group(3)[:expected_hex])
                    tick_x10 = data[0] | (data[1] << 8)
                    tick_us  = tick_x10 / 10.0
                    nib      = data[2]
                    mode_str = "STATUS_AND_DATA" if data[3] else "DATA_ONLY"
                    self.learn_result_var.set(
                        f"tick={tick_us:.1f}us  nibbles={nib}  {mode_str}")
                    # Push learned values into param fields
                    self.param_vars["tick_us"].set(str(tick_us))
                    self.param_vars["nibbles"].set(str(nib))
                    self.crc_mode_var.set(mode_str)
                except (ValueError, IndexError):
                    self.learn_result_var.set("Learn: parse error")
            self.learn_btn.config(state=tk.NORMAL)
            return

        # Filter to configured CAN ID when set; accept all if field is blank/invalid
        try:
            cfg_id = int(self.param_vars["data_can_id"].get(), 0)
            if cid != cfg_id:
                return
        except ValueError:
            pass

        expected_hex = dlc * 2
        if len(fm.group(3)) < expected_hex:
            return

        try:
            data = bytes.fromhex(fm.group(3)[:expected_hex])
        except ValueError:
            return

        # Unpack nibbles (high then low from each byte)
        nibbles = []
        for b in data:
            nibbles.append((b >> 4) & 0x0F)
            nibbles.append(b & 0x0F)

        data_hex     = "  ".join(f"{b:02X}" for b in data)
        nibbles_str  = "  ".join(f"{n:X}" for n in nibbles)

        self.sv_last_data.set(data_hex)
        self.sv_last_nibbles.set(nibbles_str)
        self.sv_last_meta.set(f"ID: {cid:03X}    DLC: {dlc}")

        ts = datetime.now().strftime("%H:%M:%S.%f")[:-4]
        iid = self.tree.insert("", 0, values=(
            ts,
            f"{cid:03X}",
            dlc,
            data_hex,
            nibbles_str,
            line,
        ), tags=("ok",))
        self._tree_iids.appendleft(iid)
        if len(self._tree_iids) > self.MAX_TABLE_ROWS:
            self.tree.delete(self._tree_iids.pop())

    def _send(self, data: bytes):
        """Write bytes to the serial port and echo them to the raw log (green)."""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(data)
            label = data.decode("ascii", errors="replace").replace("\r", "\\r")
            self._append_log(f"→ {label}", tag="tx_cmd")

    def _append_log(self, text: str, tag: str | None = None):
        # Tag each line by SLCAN response type (caller may override with explicit tag):
        #   tx_cmd — outgoing command  → ...
        #   slcan  — data frame        t/T...
        #   other  — frame ACK         z / Z
        #   info   — info resp         V.../v.../N.../F...
        #   raw    — NACK              \a (bell)
        #   dbg    — command ACK       \r (empty line after strip)
        if tag is None:
            if re.match(r'^t[0-9A-Fa-f]', text):
                tag = "slcan"
            elif re.match(r'^[zZ]$', text):
                tag = "other"   # frame ACK
            elif re.match(r'^[VvNF]', text):
                tag = "info"    # version / serial / status response
            elif '\x07' in text or text == '\a':
                tag = "raw"     # NACK (\a = bell)
            elif text == "":
                tag = "dbg"     # command ACK (\r stripped to empty)
            else:
                tag = "other"

        self.log.config(state=tk.NORMAL)
        self.log.insert(tk.END, text + "\n", tag)
        self.log.see(tk.END)
        # Keep log bounded
        lines = int(self.log.index(tk.END).split(".")[0])
        if lines > 600:
            self.log.delete("1.0", f"{lines - 500}.0")
        self.log.config(state=tk.DISABLED)


# ─── Entry point ──────────────────────────────────────────────────────────────

def main():
    root = tk.Tk()
    app  = SentMonitor(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app._disconnect(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
