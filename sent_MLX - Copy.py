#!/usr/bin/env python3
"""
SENT Sensor Monitor — MLX90377 angle sensor viewer
Connects to STM32F042 USB CDC and displays decoded SENT frames.

Output format from firmware:
  tXXX3AABBCC                — SLCAN compact frame (3 bytes = 6 nibbles, CAN ID configurable)
  z\r / Z\r                  — SLCAN frame ACK (ignored)
  \r                         — SLCAN command ACK (ignored)

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
import math
from datetime import datetime


# ─── Angle gauge (canvas) ────────────────────────────────────────────────────

class AngleGauge(tk.Canvas):
    """Simple circular gauge showing a 12-bit angle (0x000–0xFFF = 0°–360°)."""

    RADIUS   = 70
    SIZE     = 160

    def __init__(self, parent, **kwargs):
        super().__init__(parent, width=self.SIZE, height=self.SIZE,
                         bg="#1e1e2e", highlightthickness=0, **kwargs)
        self._angle_raw = None
        self._draw_base()

    def _draw_base(self):
        cx = cy = self.SIZE // 2
        r = self.RADIUS
        # Outer ring
        self.create_oval(cx-r, cy-r, cx+r, cy+r, outline="#444466", width=2)
        # Tick marks
        for deg in range(0, 360, 30):
            rad = math.radians(deg - 90)
            x1 = cx + (r - 6) * math.cos(rad)
            y1 = cy + (r - 6) * math.sin(rad)
            x2 = cx + r * math.cos(rad)
            y2 = cy + r * math.sin(rad)
            self.create_line(x1, y1, x2, y2, fill="#555577", width=1)
        # Center dot
        self.create_oval(cx-4, cy-4, cx+4, cy+4, fill="#888899", outline="")
        # Labels
        for deg, label in [(0,"N"),(90,"E"),(180,"S"),(270,"W")]:
            rad = math.radians(deg - 90)
            lx = cx + (r - 16) * math.cos(rad)
            ly = cy + (r - 16) * math.sin(rad)
            self.create_text(lx, ly, text=label, fill="#666688",
                             font=("Consolas", 8))
        self._needle = self.create_line(cx, cy, cx, cy - r + 8,
                                        fill="#00BFFF", width=2, arrow=tk.LAST)
        self._arc = self.create_arc(cx-r+8, cy-r+8, cx+r-8, cy+r-8,
                                    start=90, extent=0,
                                    style=tk.ARC, outline="#005BBB", width=3)
        self._label = self.create_text(cx, cy + r + 14, text="—",
                                       fill="#AAAACC", font=("Consolas", 10))

    def set_angle(self, angle_raw, is_init=False):
        """Update needle to angle_raw (0x000–0xFFF). Pass is_init=True for 0xFFF init frames."""
        cx = cy = self.SIZE // 2
        r = self.RADIUS
        if is_init or angle_raw is None:
            angle_deg = 0.0
            color = "#555566"
            label_text = "INIT"
        else:
            angle_deg = angle_raw * 360.0 / 4096.0
            color = "#00BFFF"
            label_text = f"{angle_deg:.1f}°"

        rad = math.radians(angle_deg - 90)
        nx = cx + (r - 8) * math.cos(rad)
        ny = cy + (r - 8) * math.sin(rad)
        self.coords(self._needle, cx, cy, nx, ny)
        self.itemconfig(self._needle, fill=color)
        self.itemconfig(self._arc, start=90, extent=-(angle_deg % 360),
                        outline="#005BBB" if not is_init else "#333344")
        self.itemconfig(self._label, text=label_text, fill=color)


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
    MAX_TABLE_ROWS = 300
    POLL_MS        = 40   # UI update interval

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
        self.edge_count   = 0
        self._frame_times: list[float] = []
        self._last_slcan  = ""

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
        style.configure("TCombobox", fieldbackground="#2a2a3e", foreground="#CCCCDD")
        style.configure("TButton", background="#2a2a3e", foreground="#CCCCDD")
        style.configure("Treeview", background="#181828", foreground="#CCCCDD",
                        fieldbackground="#181828", rowheight=20)
        style.configure("Treeview.Heading", background="#2a2a3e", foreground="#AAAACC",
                        font=("Consolas", 9, "bold"))
        style.map("Treeview", background=[("selected","#3a3a5e")])

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
            ("Data CAN ID:",     "data_can_id", "0x510"),
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

        # ── Control buttons ──
        ctrl = ttk.Frame(parent)
        ctrl.pack(fill=tk.X, padx=6, pady=(6,4))
        self.pause_btn = ttk.Button(ctrl, text="⏸  Pause", command=self._toggle_pause)
        self.pause_btn.pack(fill=tk.X, pady=2)
        ttk.Button(ctrl, text="🗑  Clear", command=self._clear).pack(fill=tk.X, pady=2)

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

        # ── Middle: gauge + current value ──
        mid = ttk.Frame(parent)
        mid.pack(fill=tk.X, padx=6, pady=2)

        self.gauge = AngleGauge(mid)
        self.gauge.pack(side=tk.LEFT, padx=(0,12))

        val_frame = ttk.Frame(mid)
        val_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        big_font  = font.Font(family="Consolas", size=32, weight="bold")
        med_font  = font.Font(family="Consolas", size=14)
        info_font = font.Font(family="Consolas", size=10)

        self.sv_angle_hex = tk.StringVar(value="A: ———")
        self.sv_angle_deg = tk.StringVar(value="———°")
        self.sv_mag       = tk.StringVar(value="M: ———")
        self.sv_roll      = tk.StringVar(value="S: —  E: —")

        self.angle_lbl = tk.Label(val_frame, textvariable=self.sv_angle_hex,
                                  font=big_font, fg="#00BFFF", bg="#1e1e2e",
                                  anchor=tk.W)
        self.angle_lbl.pack(fill=tk.X)
        tk.Label(val_frame, textvariable=self.sv_angle_deg,
                 font=med_font, fg="#4488CC", bg="#1e1e2e", anchor=tk.W).pack(fill=tk.X)
        tk.Label(val_frame, textvariable=self.sv_mag,
                 font=info_font, fg="#AAAACC", bg="#1e1e2e", anchor=tk.W).pack(fill=tk.X, pady=(4,0))
        tk.Label(val_frame, textvariable=self.sv_roll,
                 font=info_font, fg="#888899", bg="#1e1e2e", anchor=tk.W).pack(fill=tk.X)

        # ── Frame table ──
        tf = ttk.LabelFrame(parent, text=" Received Frames ", padding=4)
        tf.pack(fill=tk.BOTH, expand=True, padx=6, pady=2)

        cols = ("time", "angle_hex", "angle_deg", "counter", "roll", "err", "slcan")
        self.tree = ttk.Treeview(tf, columns=cols, show="headings", height=8,
                                  selectmode="browse")
        headers = {
            "time":      ("Time",        75, tk.CENTER),
            "angle_hex": ("Angle (hex)",  85, tk.CENTER),
            "angle_deg": ("Angle (°)",   80, tk.CENTER),
            "counter":   ("Counter",      70, tk.CENTER),
            "roll":      ("Roll",         45, tk.CENTER),
            "err":       ("Err",          40, tk.CENTER),
            "slcan":     ("SLCAN Frame", 200, tk.W),
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

        self.tree.tag_configure("init",  foreground="#666688")
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
        self.log.tag_config("mlx",  foreground="#00BFFF")
        self.log.tag_config("dbg",  foreground="#FFAA44")
        self.log.tag_config("raw",  foreground="#FF6666")
        self.log.tag_config("slcan",foreground="#666688")
        self.log.tag_config("other",foreground="#555566")

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
            self.serial_port.write(b"O\r")
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            self.status_lbl.config(text=f"● Connected  {port}", foreground="#66CC66")
        except Exception as e:
            self.status_lbl.config(text=f"● {e}", foreground="#FF6666")

    def _disconnect(self):
        self.running = False
        if self.serial_port:
            try:
                # SLCAN: send 'C\r' to close channel before disconnecting
                self.serial_port.write(b"C\r")
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)
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
        self.sv_angle_hex.set("A: ———")
        self.sv_angle_deg.set("———°")
        self.sv_mag.set("M: ———")
        self.sv_roll.set("S: —  E: —")
        self.gauge.set_angle(None)
        for child in self.tree.get_children():
            self.tree.delete(child)
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
            last_data: str | None = None
            while True:
                line = self.rx_queue.get_nowait()
                if re.match(r'^t[0-9A-Fa-f]', line):
                    last_data = line   # keep only newest data frame
                else:
                    # ACKs (\r → shown as blank), NACKs (\a), z/Z, V/v/N/F responses
                    self._append_log(line)
        except queue.Empty:
            pass
        if last_data is not None:
            self._process_line(last_data)
        self.root.after(self.POLL_MS, self._poll_queue)

    def _process_line(self, line: str):
        self._append_log(line)

        if self.paused:
            return

        # ── SLCAN data frame ──────────────────────────────────────────────────
        fm = re.match(r'^t([0-9A-Fa-f]{3})([0-9A-Fa-f])([0-9A-Fa-f]*)', line)
        if not fm:
            return

        try:
            cid = int(fm.group(1), 16)
            dlc = int(fm.group(2), 16)
            cfg_id = int(self.param_vars["data_can_id"].get(), 0)
        except ValueError:
            return

        if cid != cfg_id or dlc < 3 or len(fm.group(3)) < 6:
            return

        try:
            data = bytes.fromhex(fm.group(3)[:6])
        except ValueError:
            return

        angle = (data[0] << 4) | (data[1] >> 4)
        mag   = ((data[1] & 0x0F) << 8) | data[2]
        is_init   = (angle == 0xFFF)
        angle_deg = angle * 360.0 / 4096.0

        self.frame_count += 1
        now = time.monotonic()
        self._frame_times.append(now)
        self._frame_times = [t for t in self._frame_times if now - t <= 2.0]
        rate = len(self._frame_times) / 2.0

        self.sv_frames.set(f"Frames: {self.frame_count}")
        self.sv_rate.set(f"Rate: {rate:.1f} fps")

        self.gauge.set_angle(angle, is_init=is_init)
        if is_init:
            self.sv_angle_hex.set(f"A: {angle:03X}  (init)")
            self.sv_angle_deg.set("Sensor initialising…")
            self.angle_lbl.config(fg="#555577")
        else:
            self.sv_angle_hex.set(f"A: {angle:03X}")
            self.sv_angle_deg.set(f"{angle_deg:.2f}°")
            self.angle_lbl.config(fg="#00BFFF")

        self.sv_mag.set(f"M: {mag:03X}")
        self.sv_roll.set("—")

        tag = "init" if is_init else "ok"
        ts  = datetime.now().strftime("%H:%M:%S.%f")[:-4]
        self.tree.insert("", 0, values=(
            ts,
            f"0x{angle:03X}",
            "init" if is_init else f"{angle_deg:.2f}",
            f"0x{mag:03X}",
            "—", "—",
            line,
        ), tags=(tag,))

        children = self.tree.get_children()
        if len(children) > self.MAX_TABLE_ROWS:
            for child in children[self.MAX_TABLE_ROWS:]:
                self.tree.delete(child)

    def _append_log(self, text: str):
        # Tag each line by SLCAN response type:
        #   slcan  — data frame  t/T...
        #   other  — frame ACK   z / Z
        #   mlx    — info resp   V.../v.../N.../F...
        #   raw    — NACK        \a (bell)
        #   dbg    — command ACK \r (empty line after strip)
        if re.match(r'^t[0-9A-Fa-f]', text):
            tag = "slcan"
        elif re.match(r'^[zZ]$', text):
            tag = "other"   # frame ACK
        elif re.match(r'^[VvNF]', text):
            tag = "mlx"     # version / serial / status response
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
