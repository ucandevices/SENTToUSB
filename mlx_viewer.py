#!/usr/bin/env python3
"""
MLX90377 SENT Sensor Viewer
Live visualization of angle from a Melexis MLX90377 read via the SENTToUSB
SLCAN bridge.

Angle decoding (datasheet section 11.3.3, Figure 15 — H.4 Single Secure):
    nibbles 0..2 -> 12-bit angle (0x000..0xFFF maps to 0..360 deg)

Auto-applies MLX90377 defaults on connect (3 us tick, DATA_ONLY CRC, seed 0x03,
CAN ID 0x510), so just pick the COM port and hit Connect.

Requirements: pyserial (tkinter is bundled with Python).
"""
import math
import queue
import re
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import ttk

import serial
import serial.tools.list_ports


# ─── MLX90377 default SENT params ─────────────────────────────────────────────
TICK_US      = 3.0
NIBBLES      = 6
CRC_MODE     = 0          # DATA_ONLY
CRC_SEED     = 0x03
CAN_ID       = 0x510

ANGLE_FULL_SCALE = 0x1000  # 12 bits -> 360 deg

HISTORY_SECONDS = 10.0
DIAL_SIZE       = 360
PLOT_HEIGHT     = 220
SLOW_HISTORY_MAX = 128
SLOW_PLOT_COLORS = ["#88FFAA", "#FFCC66", "#FF88CC", "#AA99FF", "#66E0FF", "#FF8866"]


def slcan_config_frame(nibbles: int, crc_mode: int, seed: int,
                       tick_us: float, can_id: int) -> bytes:
    """0x001 config frame, layout matching sent_test.py / sent_viewer.py."""
    min_b  = max(1, min(round(tick_us * 0.8 / 0.5), 255))
    max_b  = max(1, min(int(tick_us * 1.5 + 0.5), 255))
    can_id = max(1, min(can_id, 0x7FF))
    data = bytes([
        nibbles, crc_mode, seed, min_b, max_b,
        (can_id >> 8) & 0xFF, can_id & 0xFF,
    ])
    return f"t001{len(data):X}{data.hex().upper()}\r".encode("ascii")


def slcan_set_tx_tick(tick_us: float) -> bytes:
    tick_x10 = max(20, min(int(round(tick_us * 10.0)), 900))
    payload = bytes([0x05, tick_x10 & 0xFF, (tick_x10 >> 8) & 0xFF])
    return f"t600{len(payload):X}{payload.hex().upper()}\r".encode("ascii")


class MLXViewer:
    POLL_MS   = 20   # serial queue drain interval
    RENDER_MS = 33   # ~30 fps redraw

    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("MLX90377 SENT Viewer")
        root.geometry("1080x740")
        root.configure(bg="#1e1e2e")

        self.serial: serial.Serial | None = None
        self.rx_thread: threading.Thread | None = None
        self.running = False
        self.rx_queue: queue.Queue[str] = queue.Queue()

        self.frame_count = 0
        self.crc_errors  = 0
        self.sync_errors = 0
        self._frame_times: deque[float] = deque()

        self.angle_raw    = 0
        self.angle_deg    = 0.0
        self.last_rx_time = 0.0
        self.zero_offset  = 0.0
        self.slow_count   = 0
        self.slow_id      = None
        self.slow_data    = None
        self.slow_format  = ""
        self.slow_by_id: dict[int, dict] = {}
        self._slow_plot_ids: list[int] = []

        # (timestamp, angle_deg)
        self.history: deque[tuple[float, float]] = deque()

        self._build_ui()
        self._refresh_ports()
        self._poll_queue()
        self._redraw()

    # ── UI ────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        style = ttk.Style()
        try: style.theme_use("clam")
        except Exception: pass
        style.configure("TLabelframe", background="#1e1e2e", foreground="#AAAACC")
        style.configure("TLabelframe.Label", background="#1e1e2e", foreground="#AAAACC",
                        font=("Consolas", 9, "bold"))
        style.configure("TLabel",  background="#1e1e2e", foreground="#CCCCDD")
        style.configure("TFrame",  background="#1e1e2e")
        style.configure("TEntry",  fieldbackground="#2a2a3e", foreground="#CCCCDD",
                        insertcolor="#AAAACC")
        style.configure("TCombobox", fieldbackground="#2a2a3e", foreground="#CCCCDD",
                        selectbackground="#3a3a5e", arrowcolor="#AAAACC")
        style.map("TCombobox",
            fieldbackground=[("readonly", "#2a2a3e")],
            foreground=[("readonly", "#CCCCDD")])
        style.configure("TButton", background="#2a2a3e", foreground="#CCCCDD",
                        font=("Consolas", 9))
        style.map("TButton",
            background=[("active","#4a4a6e"),("pressed","#5a5a7e"),("disabled","#1a1a2e")],
            foreground=[("active","#FFFFFF"),("disabled","#555566")])
        style.configure("Treeview", background="#131320", foreground="#CCCCDD",
                        fieldbackground="#131320", rowheight=19,
                        font=("Consolas", 9))
        style.configure("Treeview.Heading", background="#2a2a3e", foreground="#AAAACC",
                        font=("Consolas", 9, "bold"))
        style.map("Treeview", background=[("selected", "#3a3a5e")])
        self.root.option_add("*TCombobox*Listbox.background",       "#2a2a3e")
        self.root.option_add("*TCombobox*Listbox.foreground",       "#CCCCDD")
        self.root.option_add("*TCombobox*Listbox.selectBackground", "#4a4a6e")
        self.root.option_add("*TCombobox*Listbox.selectForeground", "#FFFFFF")
        self.root.option_add("*TCombobox*Listbox.font",             "Consolas 9")

        paned = tk.PanedWindow(self.root, orient=tk.HORIZONTAL, sashwidth=5,
                               sashrelief=tk.FLAT, bg="#1e1e2e")
        paned.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        left  = ttk.Frame(paned, width=240)
        right = ttk.Frame(paned)
        paned.add(left,  minsize=220)
        paned.add(right, minsize=600)
        self._build_left(left)
        self._build_right(right)

    def _build_left(self, parent):
        pf = ttk.LabelFrame(parent, text=" Serial Port ", padding=8)
        pf.pack(fill=tk.X, padx=6, pady=(6,3))
        ttk.Label(pf, text="Port:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(pf, textvariable=self.port_var, width=11,
                                        state="readonly")
        self.port_combo.grid(row=0, column=1, sticky=tk.EW, padx=(4,0))
        ttk.Button(pf, text="⟳", width=3, command=self._refresh_ports).grid(
            row=0, column=2, padx=(3,0))
        pf.columnconfigure(1, weight=1)

        btn = ttk.Frame(pf)
        btn.grid(row=1, column=0, columnspan=3, sticky=tk.EW, pady=(6,0))
        self.connect_btn    = ttk.Button(btn, text="Connect",    command=self._connect)
        self.disconnect_btn = ttk.Button(btn, text="Disconnect", command=self._disconnect,
                                          state=tk.DISABLED)
        self.connect_btn.pack(   side=tk.LEFT, expand=True, fill=tk.X, padx=(0,2))
        self.disconnect_btn.pack(side=tk.LEFT, expand=True, fill=tk.X)

        self.status_lbl = ttk.Label(pf, text="● Disconnected", foreground="#FF6666")
        self.status_lbl.grid(row=2, column=0, columnspan=3, pady=(5,0))

        sf = ttk.LabelFrame(parent, text=" MLX90377 ", padding=8)
        sf.pack(fill=tk.X, padx=6, pady=3)
        info = [
            ("Type:",   "Magnetic angle sensor"),
            ("Format:", "H.4 Single Secure"),
            ("Tick:",   f"{TICK_US:.0f} µs / Seed 0x{CRC_SEED:02X}"),
            ("Range:",  "12-bit, 0..360°"),
            ("CAN ID:", f"0x{CAN_ID:03X}"),
        ]
        for i,(k,v) in enumerate(info):
            ttk.Label(sf, text=k).grid(row=i, column=0, sticky=tk.W, pady=1)
            ttk.Label(sf, text=v, foreground="#888899").grid(
                row=i, column=1, sticky=tk.W, padx=4)

        st = ttk.LabelFrame(parent, text=" Stats ", padding=8)
        st.pack(fill=tk.X, padx=6, pady=3)
        self.sv_frames = tk.StringVar(value="—")
        self.sv_rate   = tk.StringVar(value="—")
        self.sv_crc    = tk.StringVar(value="—")
        self.sv_sync   = tk.StringVar(value="—")
        self.sv_slow   = tk.StringVar(value="—")
        rows = [("Frames:",   self.sv_frames, "#CCCCDD"),
                ("Rate:",     self.sv_rate,   "#88CCFF"),
                ("CRC err:",  self.sv_crc,    "#FF8888"),
                ("Sync err:", self.sv_sync,   "#FFAA66"),
                ("Slow:",     self.sv_slow,   "#88FFAA")]
        for i,(k,v,fg) in enumerate(rows):
            ttk.Label(st, text=k).grid(row=i, column=0, sticky=tk.W, pady=1)
            ttk.Label(st, textvariable=v, foreground=fg).grid(
                row=i, column=1, sticky=tk.W, padx=4)

        cf = ttk.LabelFrame(parent, text=" Controls ", padding=8)
        cf.pack(fill=tk.X, padx=6, pady=3)
        self.zero_pending = tk.BooleanVar(value=False)
        tk.Checkbutton(cf, text="Zero on next frame", variable=self.zero_pending,
                       bg="#1e1e2e", fg="#CCCCDD", selectcolor="#2a2a3e",
                       activebackground="#1e1e2e", activeforeground="#CCCCDD",
                       font=("Consolas", 9)).pack(anchor=tk.W)
        ttk.Button(cf, text="Reset zero",    command=self._reset_zero).pack(
            fill=tk.X, pady=(4,2))
        ttk.Button(cf, text="Clear history", command=self._clear).pack(fill=tk.X)

    def _build_right(self, parent):
        top = ttk.Frame(parent)
        top.pack(fill=tk.X, padx=6, pady=(6,3))

        self.dial = tk.Canvas(top, width=DIAL_SIZE, height=DIAL_SIZE,
                              bg="#131320", highlightthickness=0)
        self.dial.pack(side=tk.LEFT)

        readout = ttk.Frame(top)
        readout.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(12,0))

        self.sv_angle     = tk.StringVar(value="—°")
        self.sv_angle_raw = tk.StringVar(value="—")
        self.sv_age       = tk.StringVar(value="no data")

        big = ("Consolas", 32, "bold")
        sm  = ("Consolas", 10)

        tk.Label(readout, text="Angle", font=sm, fg="#888899", bg="#1e1e2e",
                 anchor=tk.W).pack(fill=tk.X)
        tk.Label(readout, textvariable=self.sv_angle, font=big, fg="#00BFFF",
                 bg="#1e1e2e", anchor=tk.W).pack(fill=tk.X)
        tk.Label(readout, textvariable=self.sv_angle_raw, font=sm, fg="#666688",
                 bg="#1e1e2e", anchor=tk.W).pack(fill=tk.X)

        tk.Label(readout, text=" ", bg="#1e1e2e").pack()
        tk.Label(readout, textvariable=self.sv_age, font=sm, fg="#666688",
                 bg="#1e1e2e", anchor=tk.W).pack(fill=tk.X)

        slow_lf = ttk.LabelFrame(readout, text=" Slow Channel ", padding=6)
        slow_lf.pack(fill=tk.BOTH, expand=True, pady=(14,0))
        self.sv_slow_latest = tk.StringVar(value="no slow message")
        tk.Label(slow_lf, textvariable=self.sv_slow_latest, font=("Consolas", 11, "bold"),
                 fg="#88FFAA", bg="#1e1e2e", anchor=tk.W).pack(fill=tk.X)
        cols = ("id", "fmt", "value", "dec", "range", "count", "age")
        self.slow_table = ttk.Treeview(slow_lf, columns=cols, show="headings", height=9)
        headings = [("id", "ID", 42), ("fmt", "Fmt", 54), ("value", "Value", 86),
                    ("dec", "Dec", 72), ("range", "Range", 92),
                    ("count", "N", 46), ("age", "Age", 58)]
        for col, text, width in headings:
            self.slow_table.heading(col, text=text)
            self.slow_table.column(col, width=width, anchor=tk.E if col != "fmt" else tk.W,
                                   stretch=(col == "value"))
        self.slow_table.pack(fill=tk.BOTH, expand=True, pady=(6,0))
        for i, color in enumerate(SLOW_PLOT_COLORS):
            self.slow_table.tag_configure(f"plot{i}", foreground=color)

        plot_lf = ttk.LabelFrame(parent,
            text=f" Angle + slow history (last {HISTORY_SECONDS:.0f} s) ", padding=4)
        plot_lf.pack(fill=tk.BOTH, expand=True, padx=6, pady=3)
        self.plot = tk.Canvas(plot_lf, bg="#131320", highlightthickness=0,
                              height=PLOT_HEIGHT)
        self.plot.pack(fill=tk.BOTH, expand=True)

    # ── Drawing ───────────────────────────────────────────────────────────────
    def _draw_dial(self):
        c = self.dial
        c.delete("all")
        cx = cy = DIAL_SIZE // 2
        r_outer = DIAL_SIZE // 2 - 8
        r_inner = r_outer - 18

        c.create_oval(cx-r_outer, cy-r_outer, cx+r_outer, cy+r_outer,
                      outline="#3a3a5e", width=2)
        c.create_oval(cx-r_inner, cy-r_inner, cx+r_inner, cy+r_inner,
                      outline="#2a2a3e", width=1)

        for deg in range(0, 360, 10):
            ang = math.radians(deg - 90)   # 0° at top, clockwise
            major = (deg % 30) == 0
            r1 = r_outer
            r2 = r_outer - (12 if major else 6)
            color = "#AAAACC" if major else "#555577"
            x1 = cx + r1 * math.cos(ang); y1 = cy + r1 * math.sin(ang)
            x2 = cx + r2 * math.cos(ang); y2 = cy + r2 * math.sin(ang)
            c.create_line(x1, y1, x2, y2, fill=color, width=2 if major else 1)
            if major:
                lr = r_outer - 28
                lx = cx + lr * math.cos(ang); ly = cy + lr * math.sin(ang)
                c.create_text(lx, ly, text=str(deg), fill="#888899",
                              font=("Consolas", 9))

        a = (self.angle_deg - self.zero_offset) % 360.0
        ang_rad = math.radians(a - 90)
        nr = r_inner - 14
        nx = cx + nr * math.cos(ang_rad)
        ny = cy + nr * math.sin(ang_rad)
        tx = cx - nr * 0.18 * math.cos(ang_rad)
        ty = cy - nr * 0.18 * math.sin(ang_rad)
        c.create_line(tx, ty, nx, ny, fill="#00BFFF", width=4, capstyle=tk.ROUND)
        c.create_oval(cx-7, cy-7, cx+7, cy+7, fill="#00BFFF", outline="")

        c.create_text(cx, cy + r_inner//2 + 12, text=f"{a:6.2f}°",
                      fill="#00BFFF", font=("Consolas", 18, "bold"))

    def _draw_plot(self):
        p = self.plot
        p.delete("all")
        w = p.winfo_width()
        h = p.winfo_height()
        if w <= 4 or h <= 4:
            return

        m_l, m_r, m_t, m_b = 38, 94, 8, 18
        x0, x1 = m_l, w - m_r
        y0, y1 = m_t, h - m_b
        p.create_rectangle(x0, y0, x1, y1, outline="#3a3a5e")

        for deg in (0, 90, 180, 270, 360):
            y = y1 - (y1 - y0) * (deg / 360.0)
            p.create_line(x0, y, x1, y, fill="#222238", dash=(2,3))
            p.create_text(x0-4, y, text=f"{deg}°", anchor=tk.E,
                          fill="#666688", font=("Consolas", 8))

        for sec in range(0, int(HISTORY_SECONDS) + 1):
            x = x1 - (x1 - x0) * (sec / HISTORY_SECONDS)
            p.create_line(x, y1, x, y1+3, fill="#666688")
            p.create_text(x, y1+10, text=f"-{sec}s", anchor=tk.N,
                          fill="#666688", font=("Consolas", 8))

        now = time.monotonic()
        cutoff = now - HISTORY_SECONDS
        while self.history and self.history[0][0] < cutoff:
            self.history.popleft()

        for idx, sid in enumerate(self._slow_plot_ids):
            entry = self.slow_by_id.get(sid)
            if not entry:
                continue
            samples = [(t, v) for t, v in entry["history"] if t >= cutoff]
            if len(samples) < 2:
                continue
            values = [v for _t, v in samples]
            lo = min(values)
            hi = max(values)
            if lo == hi:
                continue

            color = SLOW_PLOT_COLORS[idx % len(SLOW_PLOT_COLORS)]
            pts: list[float] = []
            for t, value in samples:
                x = x1 - (x1 - x0) * ((now - t) / HISTORY_SECONDS)
                y = y1 - (y1 - y0) * ((value - lo) / (hi - lo))
                pts.extend([x, y])
            if len(pts) >= 4:
                p.create_line(*pts, fill=color, width=1)

            ly = y0 + 12 + idx * 14
            p.create_line(x1 + 8, ly, x1 + 22, ly, fill=color, width=2)
            p.create_text(x1 + 26, ly, text=f"ID {sid:02X}", anchor=tk.W,
                          fill=color, font=("Consolas", 8))

        if len(self.history) < 2:
            return

        # Angle trace, broken at wrap-around so 359 -> 0 doesn't draw a vertical streak
        prev_y = None
        seg: list[float] = []
        wrap_threshold = (y1 - y0) * 0.5
        for t, ang in self.history:
            x = x1 - (x1 - x0) * ((now - t) / HISTORY_SECONDS)
            ya = y1 - (y1 - y0) * (((ang - self.zero_offset) % 360.0) / 360.0)
            if prev_y is not None and abs(ya - prev_y) > wrap_threshold:
                if len(seg) >= 4:
                    p.create_line(*seg, fill="#00BFFF", width=2)
                seg = []
            seg.extend([x, ya])
            prev_y = ya
        if len(seg) >= 4:
            p.create_line(*seg, fill="#00BFFF", width=2)

    # ── Serial ────────────────────────────────────────────────────────────────
    def _refresh_ports(self):
        ports = [p.device for p in sorted(serial.tools.list_ports.comports(),
                                          key=lambda p: p.device)]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            preferred = next((p for p in ports if "COM8" in p), ports[0])
            self.port_var.set(preferred)

    def _connect(self):
        port = self.port_var.get()
        if not port:
            self.status_lbl.config(text="● No port selected", foreground="#FF6666")
            return
        try:
            self.serial = serial.Serial(port, 115200, timeout=0.1)
            self.serial.reset_input_buffer()
            self.running = True
            self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.rx_thread.start()
            self._send(b"O\r")
            self._send(slcan_config_frame(NIBBLES, CRC_MODE, CRC_SEED, TICK_US, CAN_ID))
            self._send(slcan_set_tx_tick(TICK_US))
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            self.status_lbl.config(text=f"● Connected  {port}", foreground="#66CC66")
        except Exception as e:
            self.status_lbl.config(text=f"● {e}", foreground="#FF6666")

    def _disconnect(self):
        self.running = False
        if self.serial:
            try:
                self._send(b"C\r")
                self.serial.close()
            except Exception:
                pass
            self.serial = None
        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)
        self.status_lbl.config(text="● Disconnected", foreground="#FF6666")

    def _send(self, data: bytes):
        if self.serial and self.serial.is_open:
            try: self.serial.write(data)
            except Exception: pass

    def _slow_hex_width(self, fmt: str) -> int:
        return 4 if fmt == "ESM16" else (3 if fmt == "ESM12" else 2)

    def _record_slow_message(self, when: float, slow_id: int, data: int, fmt: str):
        entry = self.slow_by_id.get(slow_id)
        if entry is None:
            entry = {
                "format": fmt,
                "data": data,
                "count": 0,
                "first": when,
                "last": when,
                "min": data,
                "max": data,
                "history": deque(maxlen=SLOW_HISTORY_MAX),
            }
            self.slow_by_id[slow_id] = entry

        entry["format"] = fmt
        entry["data"] = data
        entry["count"] += 1
        entry["last"] = when
        entry["min"] = min(entry["min"], data)
        entry["max"] = max(entry["max"], data)
        entry["history"].append((when, data))

    def _select_slow_plot_ids(self, now: float) -> list[int]:
        cutoff = now - HISTORY_SECONDS
        candidates: list[tuple[float, int]] = []
        for sid, entry in self.slow_by_id.items():
            hist = entry["history"]
            while hist and hist[0][0] < cutoff:
                hist.popleft()
            if len(hist) < 2:
                continue
            values = [v for _t, v in hist]
            if min(values) == max(values):
                continue
            candidates.append((entry["last"], sid))
        candidates.sort(reverse=True)
        return [sid for _last, sid in candidates[:len(SLOW_PLOT_COLORS)]]

    def _reset_zero(self):
        self.zero_offset = 0.0
        self.zero_pending.set(False)

    def _clear(self):
        self.history.clear()
        self.slow_by_id.clear()
        self._slow_plot_ids = []
        self._frame_times.clear()
        self.frame_count = self.crc_errors = self.sync_errors = 0
        self.slow_count = 0
        self.slow_id = None
        self.slow_data = None
        self.slow_format = ""

    def _rx_loop(self):
        buf = b""
        while self.running:
            try:
                chunk = self.serial.read(256)
            except Exception:
                break
            if not chunk:
                continue
            buf += chunk
            parts = buf.replace(b"\r\n", b"\n").replace(b"\r", b"\n").split(b"\n")
            buf = parts[-1]
            for raw in parts[:-1]:
                s = raw.decode("ascii", errors="replace").strip()
                if s:
                    self.rx_queue.put(s)

    def _poll_queue(self):
        try:
            while True:
                self._process_line(self.rx_queue.get_nowait())
        except queue.Empty:
            pass
        self.root.after(self.POLL_MS, self._poll_queue)

    def _process_line(self, line: str):
        # Diagnostic frame from firmware: t511...
        m = re.match(r"^t511([0-9A-Fa-f])([0-9A-Fa-f]+)$", line)
        if m:
            try:
                dlc = int(m.group(1), 16)
                if dlc >= 8 and len(m.group(2)) >= 16:
                    d = bytes.fromhex(m.group(2)[:16])
                    self.frame_count = d[0] | (d[1]<<8) | (d[2]<<16) | (d[3]<<24)
                    self.crc_errors  = d[4] | (d[5]<<8)
                    self.sync_errors = d[6] | (d[7]<<8)
            except (ValueError, IndexError):
                pass
            return

        m = re.match(r"^t([0-9A-Fa-f]{3})([0-9A-Fa-f])([0-9A-Fa-f]*)$", line)
        if not m:
            return
        cid = int(m.group(1), 16)
        dlc = int(m.group(2), 16)
        if cid != CAN_ID or dlc < 3:
            return
        try:
            d = bytes.fromhex(m.group(3)[:dlc * 2])
        except ValueError:
            return
        if len(d) < 3:
            return

        angle_raw = (d[0] << 4) | (d[1] >> 4)
        angle_deg = (angle_raw * 360.0) / ANGLE_FULL_SCALE
        now = time.monotonic()

        if dlc >= 7 and len(d) >= 7 and (d[3] & 0x01):
            slow_id = d[4]
            slow_data = d[5] | (d[6] << 8)
            slow_format = "ESM16" if (d[3] & 0x04) else ("ESM12" if (d[3] & 0x02) else "SSM")
            self.slow_id = slow_id
            self.slow_data = slow_data
            self.slow_format = slow_format
            self.slow_count += 1
            self._record_slow_message(now, slow_id, slow_data, slow_format)

        if self.zero_pending.get():
            self.zero_offset = angle_deg
            self.zero_pending.set(False)

        self.angle_raw    = angle_raw
        self.angle_deg    = angle_deg
        self.last_rx_time = now
        self.history.append((now, angle_deg))
        self._frame_times.append(now)

    # ── Render loop ───────────────────────────────────────────────────────────
    def _redraw(self):
        now = time.monotonic()
        while self._frame_times and now - self._frame_times[0] > 2.0:
            self._frame_times.popleft()
        fps = len(self._frame_times) / 2.0

        self.sv_frames.set(f"{self.frame_count}")
        self.sv_rate.set(f"{fps:.1f} fps")
        self.sv_crc.set(f"{self.crc_errors}")
        self.sv_sync.set(f"{self.sync_errors}")
        self.sv_slow.set(f"{self.slow_count}")
        self._slow_plot_ids = self._select_slow_plot_ids(now)

        a = (self.angle_deg - self.zero_offset) % 360.0
        self.sv_angle.set(f"{a:6.2f}°")
        self.sv_angle_raw.set(
            f"raw 0x{self.angle_raw:03X}  ({self.angle_raw}/{ANGLE_FULL_SCALE-1})")

        if self.last_rx_time > 0:
            age = now - self.last_rx_time
            if age < 0.5:
                self.sv_age.set(f"live  ({age*1000:.0f} ms ago)")
            else:
                self.sv_age.set(f"stale  ({age:.1f} s ago)")
        else:
            self.sv_age.set("no data")

        if self.slow_id is None:
            self.sv_slow_latest.set("no slow message")
        else:
            width = self._slow_hex_width(self.slow_format)
            self.sv_slow_latest.set(
                f"{self.slow_format}  ID 0x{self.slow_id:02X}  data 0x{self.slow_data:0{width}X}")

        for iid in self.slow_table.get_children():
            self.slow_table.delete(iid)
        plotted = {sid: i for i, sid in enumerate(self._slow_plot_ids)}
        for sid, entry in sorted(self.slow_by_id.items()):
            fmt = entry["format"]
            width = self._slow_hex_width(fmt)
            age = now - entry["last"]
            age_text = f"{age*1000:.0f}ms" if age < 1.0 else f"{age:.1f}s"
            tags = ()
            if sid in plotted:
                tags = (f"plot{plotted[sid] % len(SLOW_PLOT_COLORS)}",)
            range_text = f"{entry['min']:0{width}X}-{entry['max']:0{width}X}"
            self.slow_table.insert("", tk.END, iid=f"{sid:02X}", tags=tags,
                                   values=(f"{sid:02X}", fmt, f"0x{entry['data']:0{width}X}",
                                           f"{entry['data']}", range_text,
                                           f"{entry['count']}", age_text))

        self._draw_dial()
        self._draw_plot()
        self.root.after(self.RENDER_MS, self._redraw)


def main():
    root = tk.Tk()
    app = MLXViewer(root)
    root.protocol("WM_DELETE_WINDOW",
                  lambda: (app._disconnect(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
