#!/usr/bin/env python3
"""
MAX2870 control panel for the my2870 demo sketch (serial protocol v2.1).

Requires Python 3 with tkinter and pyserial (pip install pyserial).
Run: python max2870_gui.py

Protocol summary (see my2870.ino): one command per line, each command is answered by
"OK" or "ERR <code> <message>", optionally preceded by a status line
"S key=value ..."; lines starting with '#' are informational.
"""

import os
import queue
import re
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from tkinter.scrolledtext import ScrolledText

import serial
import serial.tools.list_ports

BAUD = 115200
F_MIN_HZ = 23_437_500
F_MAX_HZ = 6_000_000_000
REPLY_TIMEOUT_S = 3.0
BOOT_TIMEOUT_S = 3.0  # the Due programming port resets the board when opened

POWER_LEVELS = [("Off", 0), ("-4 dBm", 1), ("-1 dBm", 2), ("+2 dBm", 3), ("+5 dBm", 4)]
STEPS = [("1 Hz", 1), ("10 Hz", 10), ("100 Hz", 100), ("1 kHz", 1e3), ("10 kHz", 1e4),
         ("100 kHz", 1e5), ("1 MHz", 1e6), ("10 MHz", 1e7), ("100 MHz", 1e8)]
CP_CURRENTS = [f"{0.32 * k:.2f}" for k in range(1, 17)]
# VCO table saved after each calibration and loaded into the board at connection
VCO_TABLE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "max2870_vco_table.txt")
LONG_COMMANDS = ("SWEEP", "BURST", "CAL", "VSCAN")  # no reply timeout
# VTUNE ADC read back from register 6 (datasheet table 3)
ADC_TEXT = {0: "<0.5V", 1: "0.5-0.7V", 2: "0.7-1.3V", 3: "?", 4: "?", 5: "1.3-2.1V", 6: "2.1-2.5V", 7: ">2.5V"}
SWEEP_LOG = {"No log": "", "Log anomalies": " LOG", "Log all steps": " LOGALL"}

UNIT = {"": 1e6, "HZ": 1, "K": 1e3, "KHZ": 1e3, "M": 1e6, "MHZ": 1e6, "G": 1e9, "GHZ": 1e9}


def parse_freq(text, default_unit=1e6):
    """'3000.5' (MHz by default), '3.0005G', '3000500 Hz', '500k' -> Hz (int)."""
    m = re.fullmatch(r"\s*([-+]?\d+(?:[.,]\d*)?|[-+]?[.,]\d+)\s*([a-zA-Z]*)\s*", text)
    if not m:
        raise ValueError(f"invalid frequency: {text!r}")
    unit = m.group(2).upper()
    if unit not in UNIT:
        raise ValueError(f"invalid unit: {m.group(2)!r}")
    mult = UNIT[unit] if unit else default_unit
    return int(round(float(m.group(1).replace(",", ".")) * mult))


def fmt_mhz(hz):
    """3000000123 -> '3 000.000 123'"""
    sign = "-" if hz < 0 else ""
    hz = abs(int(hz))
    mhz, rest = divmod(hz, 1_000_000)
    return f"{sign}{mhz:,}".replace(",", " ") + f".{rest // 1000:03d} {rest % 1000:03d}"


class SerialLink:
    """Serial port with a reader thread and a one-command-at-a-time queue."""

    def __init__(self, on_line):
        self.on_line = on_line          # callback(line, quiet), called from the GUI thread via poll()
        self.port = None
        self.rx = queue.Queue()
        self.pending = None             # (command, send time, quiet) waiting for OK/ERR
        self.current = None             # command of the line being processed
        self.queue = []                 # commands waiting to be sent
        self.reader = None

    @property
    def connected(self):
        return self.port is not None

    def open(self, name):
        self.port = serial.Serial(name, BAUD, timeout=0.1)
        self.queue.clear()
        self.pending = ("<boot>", time.time(), False)  # wait for the ready message or timeout
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()

    def close(self):
        port, self.port = self.port, None
        if port:
            port.close()
        self.pending = None
        self.queue.clear()

    def _read_loop(self):
        buf = b""
        while self.port is not None:
            try:
                data = self.port.read(256)
            except (serial.SerialException, OSError, TypeError, AttributeError):
                self.rx.put(None)  # port lost
                return
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.rx.put(line.decode("ascii", "replace").strip())

    def send(self, cmd, coalesce=None, quiet=False):
        """Queue a command. coalesce: drop queued commands starting with this prefix.
        quiet: command and its replies are not shown in the console."""
        if not self.connected:
            return
        if coalesce:
            self.queue = [q for q in self.queue if not q[0].startswith(coalesce)]
        self.queue.append((cmd, quiet))
        self._pump()

    def send_now(self, text):
        """Write bypassing the queue (used to stop sweeps and bursts)."""
        if self.connected:
            self.port.write(text.encode("ascii"))

    def _pump(self):
        if self.pending is None and self.queue and self.connected:
            cmd, quiet = self.queue.pop(0)
            self.port.write((cmd + "\n").encode("ascii"))
            self.pending = (cmd, time.time(), quiet)
            self.on_line(f"> {cmd}", quiet)

    @property
    def busy_with(self):
        return self.pending[0] if self.pending else None

    def poll(self):
        """Process received lines; returns False if the port was lost."""
        while True:
            try:
                line = self.rx.get_nowait()
            except queue.Empty:
                break
            if line is None:
                self.close()
                return False
            if not line:
                continue
            quiet = bool(self.pending and self.pending[2]) and not line.startswith("ERR")
            # command the line belongs to (also for its final OK/ERR)
            self.current = self.pending[0] if self.pending else None
            if line == "OK" or line.startswith("ERR"):
                self.pending = None
            elif line.startswith("# MAX2870 ready"):
                if self.pending and self.pending[0] == "<boot>":
                    self.pending = None
            self.on_line(line, quiet)
        if self.pending:
            cmd, t0, _ = self.pending
            long_running = cmd.startswith(LONG_COMMANDS)
            timeout = BOOT_TIMEOUT_S if cmd == "<boot>" else REPLY_TIMEOUT_S
            if not long_running and time.time() - t0 > timeout:
                if cmd != "<boot>":
                    self.on_line(f"# timeout waiting reply to '{cmd}'")
                self.pending = None
        self._pump()
        return True


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("MAX2870 control")
        self.minsize(820, 600)
        self.link = SerialLink(self.log_line)
        self.status = {}
        self.slider_job = None
        self.last_user_move = 0.0  # status updates don't move the controls just after user input
        self.last_auto = 0.0

        self._build()
        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(50, self._poll)

    def _on_close(self):
        self.link.close()
        self.destroy()

    # ------------------------------------------------------------------ UI
    def _build(self):
        pad = dict(padx=4, pady=3)

        # connection
        top = ttk.Frame(self)
        top.pack(fill="x", **pad)
        ttk.Label(top, text="Port").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=40, state="readonly")
        self.port_box.pack(side="left", padx=4)
        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side="left")
        self.conn_btn = ttk.Button(top, text="Connect", command=self.toggle_connect)
        self.conn_btn.pack(side="left", padx=4)
        self.auto_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="Auto refresh status (1 s)", variable=self.auto_var).pack(side="left", padx=10)
        self.busy_lbl = ttk.Label(top, text="", foreground="gray")
        self.busy_lbl.pack(side="right")

        body = ttk.Frame(self)
        body.pack(fill="both", expand=True)
        left = ttk.Frame(body)
        left.pack(side="left", fill="both", expand=True)
        right = ttk.Frame(body)
        right.pack(side="left", fill="y")

        # status display
        st = ttk.LabelFrame(left, text="Output")
        st.pack(fill="x", **pad)
        row = ttk.Frame(st)
        row.pack(fill="x")
        self.freq_lbl = ttk.Label(row, text="--- MHz", font=("Consolas", 26, "bold"))
        self.freq_lbl.pack(side="left", padx=8)
        self.led = tk.Canvas(row, width=26, height=26, highlightthickness=0)
        self.led_item = self.led.create_oval(3, 3, 23, 23, fill="gray")
        self.led.pack(side="left", padx=4)
        self.lock_lbl = ttk.Label(row, text="LOCK ?", width=10)
        self.lock_lbl.pack(side="left")
        self.detail_lbl = ttk.Label(st, text="", font=("Consolas", 10))
        self.detail_lbl.pack(anchor="w", padx=8, pady=(0, 4))

        # frequency control
        fc = ttk.LabelFrame(left, text="Frequency")
        fc.pack(fill="x", **pad)
        r1 = ttk.Frame(fc)
        r1.pack(fill="x", **pad)
        ttk.Label(r1, text="Set (MHz, or with unit Hz/k/M/G):").pack(side="left")
        self.freq_var = tk.StringVar()
        self.freq_entry = ttk.Entry(r1, textvariable=self.freq_var, width=20, font=("Consolas", 12))
        self.freq_entry.pack(side="left", padx=4)
        self.freq_entry.bind("<Return>", lambda e: self.set_freq_from_entry())
        self.freq_entry.bind("<Up>", lambda e: self.step_freq(+1))
        self.freq_entry.bind("<Down>", lambda e: self.step_freq(-1))
        self.freq_entry.bind("<MouseWheel>", self._wheel)
        ttk.Button(r1, text="Set", command=self.set_freq_from_entry).pack(side="left")
        ttk.Label(r1, text="  Step").pack(side="left")
        self.step_var = tk.StringVar(value="1 MHz")
        ttk.Combobox(r1, textvariable=self.step_var, values=[s for s, _ in STEPS],
                     width=8, state="readonly").pack(side="left", padx=4)
        ttk.Button(r1, text="−", width=3, command=lambda: self.step_freq(-1)).pack(side="left")
        ttk.Button(r1, text="+", width=3, command=lambda: self.step_freq(+1)).pack(side="left")

        r2 = ttk.Frame(fc)
        r2.pack(fill="x", **pad)
        self.smin_var = tk.StringVar(value="2000")
        self.smax_var = tk.StringVar(value="6000")
        e = ttk.Entry(r2, textvariable=self.smin_var, width=8)
        e.pack(side="left")
        e.bind("<Return>", lambda ev: self.apply_slider_range())
        self.slider_var = tk.DoubleVar(value=3000.0)
        self.slider = ttk.Scale(r2, from_=2000, to=6000, variable=self.slider_var,
                                orient="horizontal", command=self._slider_moved)
        self.slider.pack(side="left", fill="x", expand=True, padx=4)
        self.slider.bind("<MouseWheel>", self._wheel)
        e = ttk.Entry(r2, textvariable=self.smax_var, width=8)
        e.pack(side="left")
        e.bind("<Return>", lambda ev: self.apply_slider_range())
        ttk.Label(r2, text="MHz").pack(side="left", padx=2)
        ttk.Button(r2, text="Range", command=self.apply_slider_range).pack(side="left", padx=4)

        r3 = ttk.Frame(fc)
        r3.pack(fill="x", **pad)
        ttk.Label(r3, text="Power:").pack(side="left")
        self.pwr_var = tk.IntVar(value=1)
        for label, val in POWER_LEVELS:
            ttk.Radiobutton(r3, text=label, value=val, variable=self.pwr_var,
                            command=lambda: self.cmd(f"P {self.pwr_var.get()}")).pack(side="left", padx=4)

        # sweep
        sw = ttk.LabelFrame(left, text="Sweep")
        sw.pack(fill="x", **pad)
        self.sw_vars = {}
        for name, default in (("Start", "2000"), ("Stop", "3000"), ("Step", "1"), ("Dwell ms", "50")):
            ttk.Label(sw, text=name).pack(side="left", padx=(6, 2))
            v = tk.StringVar(value=default)
            ttk.Entry(sw, textvariable=v, width=10).pack(side="left")
            self.sw_vars[name] = v
        ttk.Label(sw, text="(MHz unless unit given)").pack(side="left", padx=4)
        self.sweep_log_var = tk.StringVar(value="Log anomalies")
        ttk.Combobox(sw, textvariable=self.sweep_log_var, values=list(SWEEP_LOG), width=13,
                     state="readonly").pack(side="left", padx=4)
        self.sweep_btn = ttk.Button(sw, text="Start", command=self.toggle_sweep)
        self.sweep_btn.pack(side="right", padx=6, pady=4)

        # console
        lg = ttk.LabelFrame(left, text="Console")
        lg.pack(fill="both", expand=True, **pad)
        self.log = ScrolledText(lg, height=10, font=("Consolas", 9), state="disabled")
        self.log.pack(fill="both", expand=True)
        rr = ttk.Frame(lg)
        rr.pack(fill="x")
        self.raw_var = tk.StringVar()
        raw = ttk.Entry(rr, textvariable=self.raw_var)
        raw.pack(side="left", fill="x", expand=True)
        raw.bind("<Return>", lambda e: self.send_raw())
        ttk.Button(rr, text="Send", command=self.send_raw).pack(side="left")
        ttk.Button(rr, text="Clear", command=self.clear_log).pack(side="left")
        ttk.Button(rr, text="Save...", command=self.save_log).pack(side="left")

        # settings
        se = ttk.LabelFrame(right, text="Reference / PLL")
        se.pack(fill="x", **pad)
        self.ref_var = tk.StringVar(value="100")
        self.r_var = tk.IntVar(value=5)
        self.refmode_var = tk.StringVar(value="HALF")
        g = dict(sticky="w", padx=4, pady=2)
        ttk.Label(se, text="Ref (MHz)").grid(row=0, column=0, **g)
        ttk.Entry(se, textvariable=self.ref_var, width=10).grid(row=0, column=1, **g)
        ttk.Label(se, text="R").grid(row=1, column=0, **g)
        ttk.Spinbox(se, from_=1, to=1023, textvariable=self.r_var, width=8).grid(row=1, column=1, **g)
        ttk.Label(se, text="Ref mode").grid(row=2, column=0, **g)
        ttk.Combobox(se, textvariable=self.refmode_var, values=["NONE", "HALF", "DOUBLE"],
                     width=8, state="readonly").grid(row=2, column=1, **g)
        self.pfd_lbl = ttk.Label(se, text="fPFD = ? MHz")
        self.pfd_lbl.grid(row=3, column=0, columnspan=2, **g)
        ttk.Button(se, text="Apply reference", command=self.apply_ref).grid(row=4, column=0, columnspan=2, pady=4)
        for v in (self.ref_var, self.r_var, self.refmode_var):
            v.trace_add("write", lambda *a: self._update_pfd_preview())

        ttk.Separator(se).grid(row=5, column=0, columnspan=2, sticky="ew", pady=4)
        ttk.Label(se, text="CP current (mA)").grid(row=6, column=0, **g)
        self.cp_var = tk.StringVar(value="5.12")
        cb = ttk.Combobox(se, textvariable=self.cp_var, values=CP_CURRENTS, width=8, state="readonly")
        cb.grid(row=6, column=1, **g)
        cb.bind("<<ComboboxSelected>>", lambda e: self.cmd(f"CP {self.cp_var.get()}"))
        ttk.Label(se, text="PD polarity").grid(row=7, column=0, **g)
        self.pd_var = tk.StringVar(value="NONINV")
        cb = ttk.Combobox(se, textvariable=self.pd_var, values=["NONINV", "INV"], width=8, state="readonly")
        cb.grid(row=7, column=1, **g)
        cb.bind("<<ComboboxSelected>>", lambda e: self.cmd(f"PD {self.pd_var.get()}"))
        self.ce_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(se, text="Chip enable (CE)", variable=self.ce_var,
                        command=lambda: self.cmd("CE " + ("ON" if self.ce_var.get() else "OFF"))
                        ).grid(row=8, column=0, columnspan=2, **g)
        ttk.Button(se, text="Read status", command=lambda: self.cmd("S")).grid(row=9, column=0, columnspan=2, pady=4)

        vc = ttk.LabelFrame(right, text="VCO")
        vc.pack(fill="x", **pad)
        self.vmode_var = tk.StringVar(value="AUTO")
        rb = ttk.Frame(vc)
        rb.grid(row=0, column=0, columnspan=2, **g)
        for mode in ("AUTO", "TABLE", "MANUAL"):
            ttk.Radiobutton(rb, text=mode.capitalize(), value=mode, variable=self.vmode_var,
                            command=self.set_vmode).pack(side="left")
        ttk.Label(vc, text="Manual VCO").grid(row=1, column=0, **g)
        self.vco_var = tk.IntVar(value=0)
        sb = ttk.Spinbox(vc, from_=0, to=63, textvariable=self.vco_var, width=5,
                         command=lambda: self.cmd(f"VCO {self.vco_var.get()}"))
        sb.grid(row=1, column=1, **g)
        sb.bind("<Return>", lambda e: self.cmd(f"VCO {self.vco_var.get()}"))
        self.ffrac_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(vc, text="Force FRAC mode", variable=self.ffrac_var,
                        command=lambda: self.cmd("FRAC " + ("ON" if self.ffrac_var.get() else "OFF"))
                        ).grid(row=2, column=0, columnspan=2, **g)
        self.vco_lbl = ttk.Label(vc, text="VCO ?", font=("Consolas", 10))
        self.vco_lbl.grid(row=3, column=0, columnspan=2, **g)
        bf = ttk.Frame(vc)
        bf.grid(row=4, column=0, columnspan=2, pady=4)
        ttk.Button(bf, text="Calibrate", command=self.calibrate).pack(side="left", padx=2)
        ttk.Button(bf, text="Registers", command=lambda: self.cmd("REG")).pack(side="left", padx=2)
        self.table_lbl = ttk.Label(vc, text="no VCO table", foreground="gray")
        self.table_lbl.grid(row=5, column=0, columnspan=2, **g)
        self.table_lines = []      # T lines received from CAL/VTAB
        self.collect_table = False

    # ------------------------------------------------------------ helpers
    def log_line(self, line, quiet=False):
        if self.collect_table and (self.link.current or "").startswith("CAL"):
            if line.startswith("T "):
                self.table_lines.append(line)
            elif line == "OK" or line.startswith("ERR"):
                self._table_received()
        if line.startswith("S "):
            self._parse_status(line)
        elif line.startswith("ERR"):
            self.busy_lbl.configure(text=line, foreground="red")
        if quiet:
            return
        self.log.configure(state="normal")
        self.log.insert("end", line + "\n")
        n = int(self.log.index("end-1c").split(".")[0])
        if n > 2000:
            self.log.delete("1.0", f"{n - 2000}.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def save_log(self):
        name = filedialog.asksaveasfilename(defaultextension=".txt", initialfile="max2870_log.txt",
                                            filetypes=[("Text", "*.txt"), ("All files", "*.*")])
        if name:
            with open(name, "w", encoding="utf-8") as f:
                f.write(self.log.get("1.0", "end"))

    def clear_log(self):
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def cmd(self, text, coalesce=None):
        if not self.link.connected:
            self.busy_lbl.configure(text="not connected", foreground="red")
            return
        if str(self.busy_lbl["foreground"]) == "red":
            self.busy_lbl.configure(text="", foreground="gray")
        self.link.send(text, coalesce)

    def step_hz(self):
        return dict(STEPS)[self.step_var.get()]

    def current_req(self):
        try:
            return int(self.status.get("req", 0)) or parse_freq(self.freq_var.get())
        except ValueError:
            return 3_000_000_000

    def send_freq(self, hz):
        hz = max(F_MIN_HZ, min(F_MAX_HZ, int(hz)))
        self.freq_var.set(f"{hz / 1e6:.6f}")
        self.cmd(f"F {hz}", coalesce="F ")

    # ------------------------------------------------------------ actions
    def refresh_ports(self):
        ports = list(serial.tools.list_ports.comports())
        self.port_box["values"] = [f"{p.device} - {p.description}" for p in ports]
        if ports and not self.port_var.get():
            due = [i for i, p in enumerate(ports) if "arduino" in p.description.lower()]
            self.port_box.current(due[0] if due else 0)

    def toggle_connect(self):
        if self.link.connected:
            self.link.close()
            self.conn_btn.configure(text="Connect")
            self.busy_lbl.configure(text="disconnected", foreground="gray")
            return
        name = self.port_var.get().split(" - ")[0]
        if not name:
            return
        try:
            self.link.open(name)
        except serial.SerialException as e:
            messagebox.showerror("Serial port", str(e))
            return
        self.conn_btn.configure(text="Disconnect")
        self.log_line(f"# connected to {name}")
        self.load_table_file()
        self.cmd("S")

    def set_freq_from_entry(self):
        try:
            hz = parse_freq(self.freq_var.get())
        except ValueError as e:
            messagebox.showerror("Frequency", str(e))
            return
        self.send_freq(hz)

    def step_freq(self, direction):
        try:
            base = parse_freq(self.freq_var.get())
        except ValueError:
            base = self.current_req()
        self.send_freq(base + direction * self.step_hz())
        return "break"

    def _wheel(self, event):
        return self.step_freq(+1 if event.delta > 0 else -1)

    def _slider_moved(self, value):
        # snap to the selected step, send after a short pause (dragging generates many events)
        step = self.step_hz()
        hz = round(float(value) * 1e6 / step) * step
        self.freq_var.set(f"{hz / 1e6:.6f}")
        self.last_user_move = time.time()
        if self.slider_job:
            self.after_cancel(self.slider_job)
        self.slider_job = self.after(80, lambda: self._slider_send(hz))

    def _slider_send(self, hz):
        self.slider_job = None
        self.send_freq(hz)

    def apply_slider_range(self):
        try:
            lo = parse_freq(self.smin_var.get()) / 1e6
            hi = parse_freq(self.smax_var.get()) / 1e6
        except ValueError as e:
            messagebox.showerror("Slider range", str(e))
            return
        if lo >= hi:
            messagebox.showerror("Slider range", "min must be lower than max")
            return
        lo = max(lo, F_MIN_HZ / 1e6)
        hi = min(hi, F_MAX_HZ / 1e6)
        self.slider.configure(from_=lo, to=hi)
        self.slider_var.set(min(max(self.slider_var.get(), lo), hi))

    def _ref_values(self):
        ref = parse_freq(self.ref_var.get())
        r = int(self.r_var.get())
        mode = self.refmode_var.get()
        return ref, r, mode

    def _update_pfd_preview(self):
        try:
            ref, r, mode = self._ref_values()
            pfd = ref * {"NONE": 1, "HALF": 0.5, "DOUBLE": 2}[mode] / r
            self.pfd_lbl.configure(text=f"fPFD = {pfd / 1e6:.6g} MHz")
        except (ValueError, tk.TclError, ZeroDivisionError):
            self.pfd_lbl.configure(text="fPFD = ? MHz")

    def apply_ref(self):
        try:
            ref, r, mode = self._ref_values()
        except (ValueError, tk.TclError) as e:
            messagebox.showerror("Reference", str(e))
            return
        self.cmd(f"REF {ref} {r} {mode}")

    def toggle_sweep(self):
        busy = self.link.busy_with
        if busy and busy.startswith(LONG_COMMANDS):
            self.link.send_now("\n")  # any line stops it
            return
        try:
            start = parse_freq(self.sw_vars["Start"].get())
            stop = parse_freq(self.sw_vars["Stop"].get())
            step = parse_freq(self.sw_vars["Step"].get())
            dwell = int(self.sw_vars["Dwell ms"].get())
        except ValueError as e:
            messagebox.showerror("Sweep", str(e))
            return
        if start >= stop or step <= 0 or dwell < 0:
            messagebox.showerror("Sweep", "start < stop, step > 0 and dwell >= 0 required")
            return
        self.cmd(f"SWEEP {start} {stop} {step} {dwell}" + SWEEP_LOG[self.sweep_log_var.get()])

    def set_vmode(self):
        mode = self.vmode_var.get()
        if mode == "MANUAL":
            self.cmd(f"VCO {self.vco_var.get()}")
        else:
            self.cmd(f"VMODE {mode}")

    def calibrate(self):
        if not messagebox.askokcancel(
                "VCO calibration",
                "The calibration takes about 40 s, RF outputs are muted meanwhile.\nStart?"):
            return
        self.table_lines = []
        self.collect_table = True
        self.cmd("CAL")

    def _table_received(self):
        """End of CAL/VTAB output: save the table to the default file."""
        self.collect_table = False
        if not self.table_lines:
            return
        with open(VCO_TABLE_FILE, "w", encoding="utf-8") as f:
            f.write("\n".join(self.table_lines) + "\n")
        kind = self.table_lines[0].split("kind=")[1].split()[0] if "kind=" in self.table_lines[0] else "LOCK"
        self.table_lbl.configure(text=f"{kind} table: {len(self.table_lines)} VCOs, saved", foreground="")

    def load_table_file(self):
        """Send the saved VCO table to the board (after connection, the board starts without table)."""
        if not os.path.exists(VCO_TABLE_FILE):
            return
        n = 0
        kind = "LOCK"
        with open(VCO_TABLE_FILE, encoding="utf-8") as f:
            for line in f:
                if line.startswith("T "):
                    d = dict(x.split("=") for x in line.split()[1:])
                    kind = d.get("kind", "LOCK")
                    self.link.send(f"VTAB {d['vco']} {d['min']} {d['max']} {kind}", quiet=True)
                    n += 1
        if n:
            self.table_lbl.configure(text=f"{kind} table: {n} VCOs, loaded from file", foreground="")

    def send_raw(self):
        text = self.raw_var.get().strip()
        if not text:
            return
        busy = self.link.busy_with
        if busy and busy.startswith(LONG_COMMANDS):
            self.link.send_now("\n")
        self.cmd(text)
        self.raw_var.set("")

    # ------------------------------------------------------------ status
    def _parse_status(self, line):
        st = dict(kv.split("=", 1) for kv in line[2:].split() if "=" in kv)
        self.status = st
        try:
            f = int(st["f"])
            self.freq_lbl.configure(text=f"{fmt_mhz(f)} MHz")
            mode = "FRAC" if int(st.get("frac", 0)) else "INT"
            self.detail_lbl.configure(text=(
                f"req {fmt_mhz(int(st['req']))} MHz   err {int(st['err']):+d} Hz   "
                f"{mode}  N={st['int']}  F/M={st['frac']}/{st['mod']}  div={st['div']}\n"
                f"fPFD {int(st['pfd']) / 1e6:g} MHz (ref {int(st['ref']) / 1e6:g} MHz / R {st['r']} {st['refmode']})"
                f"   CP {st['cp']} mA  PD {st['pd']}  CE {st['ce']}  aux pwr {st['aux']}"))
            locked = st.get("lock") == "1"
            self.led.itemconfigure(self.led_item, fill="#2c2" if locked else "#d22")
            self.lock_lbl.configure(text="LOCKED" if locked else "UNLOCKED")
            # update controls without echoing commands
            if time.time() - self.last_user_move > 0.5 and not self.slider_job:
                if self.focus_get() is not self.freq_entry:
                    self.freq_var.set(f"{int(st['req']) / 1e6:.6f}")
                self.slider_var.set(int(st["req"]) / 1e6)
            self.pwr_var.set(int(st["pwr"]))
            self.cp_var.set(st["cp"])
            self.pd_var.set(st["pd"])
            self.ce_var.set(st["ce"] == "1")
            if "vmode" in st:
                vco = int(st["vco"])
                adc = int(st.get("adc", -1))
                text = f"{st['vmode']}  VCO {vco if vco >= 0 else 'auto'}"
                color = ""
                if adc >= 0:
                    text += f"  VTUNE {ADC_TEXT[adc]}"
                    color = "red" if adc in (0, 7) else ("darkorange" if adc in (1, 6) else "darkgreen")
                self.vco_lbl.configure(text=text, foreground=color)
                self.vmode_var.set(st["vmode"])
                self.ffrac_var.set(st["ffrac"] == "1")
                if vco >= 0:
                    self.vco_var.set(vco)
        except (KeyError, ValueError):
            pass  # partial or old format status line, shown in the console anyway

    def _poll(self):
        if self.link.connected and not self.link.poll():
            self.conn_btn.configure(text="Connect")
            self.busy_lbl.configure(text="port lost", foreground="red")
        busy = self.link.busy_with
        if self.link.connected:
            if busy and busy.startswith(LONG_COMMANDS):
                self.busy_lbl.configure(text=f"{busy.split()[0].lower()} running...", foreground="blue")
                self.sweep_btn.configure(text="Stop")
            else:
                if self.sweep_btn["text"] != "Start":
                    self.sweep_btn.configure(text="Start")
                if busy is None and str(self.busy_lbl["foreground"]) == "blue":
                    self.busy_lbl.configure(text="", foreground="gray")
                now = time.time()
                if (self.auto_var.get() and busy is None and not self.link.queue
                        and now - self.last_auto > 1.0):
                    self.last_auto = now
                    self.link.send("S", quiet=True)
        self.after(50, self._poll)


if __name__ == "__main__":
    App().mainloop()
