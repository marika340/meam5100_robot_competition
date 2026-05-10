#!/usr/bin/env python3
"""
robot_gui.py — Desktop controller for MEAM5100 robot via ESP-NOW bridge.

Sends serial commands to the bridge_espnow ESP32, which relays them to the
robot over ESP-NOW.  Mirrors the web interface (PIDandTOF_web.h) exactly.

Requirements:
    pip install pyserial

Usage:
    1. Flash bridge_espnow.ino to a second ESP32 (with ROBOT_AP_MAC filled in).
    2. Plug that bridge ESP32 into your laptop via USB.
    3. Run:  python robot_gui.py
    4. Select the bridge COM port from the dropdown and click Connect.
    5. Use the buttons and sliders — works identically to the web UI.
"""

import tkinter as tk
from tkinter import ttk
import threading
import queue
import time

try:
    import serial
    import serial.tools.list_ports
    SERIAL_OK = True
except ImportError:
    SERIAL_OK = False


# ─────────────────────────────────────────────────────────────────────────────
# Serial worker — runs in a background thread so the GUI never blocks.
# ─────────────────────────────────────────────────────────────────────────────

class BridgeSerial:
    def __init__(self, rx_queue: queue.Queue):
        self._port = None
        self._rx_q = rx_queue
        self._tx_q: queue.Queue = queue.Queue()
        self._running = False
        self._thread = None

    def connect(self, port: str, baud: int = 115200) -> tuple[bool, str]:
        try:
            self._port = serial.Serial(port, baud, timeout=0.05)
            time.sleep(0.1)          # let USB-CDC settle
            self._running = True
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
            return True, ""
        except Exception as e:
            self._port = None
            return False, str(e)

    def disconnect(self):
        self._running = False
        time.sleep(0.15)
        if self._port and self._port.is_open:
            self._port.close()
        self._port = None

    def send(self, line: str):
        """Queue one line for transmission (caller must NOT include \\n)."""
        self._tx_q.put(line)

    @property
    def connected(self) -> bool:
        return self._port is not None and self._port.is_open

    def _run(self):
        buf = ""
        while self._running:
            # ── Transmit ──────────────────────────────────────────────
            try:
                while True:
                    line = self._tx_q.get_nowait()
                    self._port.write((line + "\n").encode())
            except queue.Empty:
                pass

            # ── Receive ───────────────────────────────────────────────
            try:
                chunk = self._port.read(128).decode(errors="replace")
                if chunk:
                    buf += chunk
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        self._rx_q.put(line.rstrip("\r"))
            except Exception:
                pass

            time.sleep(0.005)


# ─────────────────────────────────────────────────────────────────────────────
# Main GUI
# ─────────────────────────────────────────────────────────────────────────────

class RobotGUI:
    LOG_MAX = 300          # max lines kept in the log widget
    SLIDER_SEND_DELAY = 0  # ms debounce after slider release (0 = immediate)

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("MEAM5100 Robot Controller")
        self.root.resizable(True, True)

        self.rx_q: queue.Queue = queue.Queue()
        self.bridge = BridgeSerial(self.rx_q)
        self._dir_held = False

        if not SERIAL_OK:
            self._show_install_warning()
            return

        self._build_ui()
        self._poll_rx()   # start the 50 ms RX timer

    # ─────────────────────────────────────────────────────────────────
    # UI construction
    # ─────────────────────────────────────────────────────────────────

    def _build_ui(self):
        P = 8  # default padding

        # ── Connection bar ───────────────────────────────────────────
        top = tk.Frame(self.root, bd=1, relief=tk.SUNKEN)
        top.pack(fill=tk.X, padx=P, pady=(P, 0))

        tk.Label(top, text="Bridge COM port:").pack(side=tk.LEFT, padx=(4, 2))

        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var,
                                    width=12, state="readonly")
        self.port_cb.pack(side=tk.LEFT)
        self._refresh_ports()

        tk.Button(top, text="⟳", width=2,
                  command=self._refresh_ports).pack(side=tk.LEFT, padx=2)

        self.conn_btn = tk.Button(top, text="Connect", width=10,
                                  command=self._toggle_connect)
        self.conn_btn.pack(side=tk.LEFT, padx=6)

        self.status_lbl = tk.Label(top, text="● Disconnected",
                                   fg="red", font=("", 10, "bold"))
        self.status_lbl.pack(side=tk.LEFT, padx=4)

        ttk.Separator(self.root).pack(fill=tk.X, padx=P, pady=4)

        # ── Main body: left | right ──────────────────────────────────
        body = tk.Frame(self.root)
        body.pack(fill=tk.BOTH, expand=True, padx=P)

        left = tk.Frame(body)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, P * 2))

        right = tk.Frame(body)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self._build_dpad(left)
        self._build_modes(left)
        self._build_tuning(right)
        self._build_log()

    # ── D-pad ────────────────────────────────────────────────────────

    def _build_dpad(self, parent):
        tk.Label(parent, text="Manual Drive",
                 font=("", 10, "bold")).pack(pady=(0, 4))

        dpad = tk.Frame(parent)
        dpad.pack()

        # NOTE: the web UI labels 'Left'→sends R, 'Right'→sends L.
        # This matches the existing web behavior exactly.
        def dir_btn(text, cmd, row, col, colspan=1):
            b = tk.Button(dpad, text=text, width=6, height=2,
                          font=("", 10))
            b.grid(row=row, column=col, columnspan=colspan,
                   padx=2, pady=2, sticky="nsew")
            b.bind("<ButtonPress-1>",   lambda e, d=cmd: self._dir_press(d))
            b.bind("<ButtonRelease-1>", lambda e: self._dir_release())
            return b

        dir_btn("▲ Fwd",  "F", 0, 1)
        dir_btn("◄",      "R", 1, 0)   # web: sendDir('R') is labeled Left
        dir_btn("►",      "L", 1, 2)   # web: sendDir('L') is labeled Right
        dir_btn("▼ Back", "B", 2, 1)

        stop = tk.Button(dpad, text="■ STOP", width=6, height=2,
                         bg="#f44336", fg="white", font=("", 10, "bold"),
                         command=lambda: self._send("S"))
        stop.grid(row=1, column=1, padx=2, pady=2, sticky="nsew")

        # Attack arm — big red button below the d-pad
        tk.Button(parent, text="⚡  ATTACK ARM  ⚡",
                  bg="#cc0000", fg="white",
                  font=("", 12, "bold"), height=2,
                  command=lambda: self._send("attack")).pack(
                  fill=tk.X, pady=(10, 4))

    # ── Mode buttons ─────────────────────────────────────────────────

    def _build_modes(self, parent):
        ttk.Separator(parent).pack(fill=tk.X, pady=8)
        tk.Label(parent, text="Modes",
                 font=("", 10, "bold")).pack(pady=(0, 4))

        modes = [
            (0, "Stop / Manual",      "#e0e0e0", "black"),
            (1, "Wall Follow",        "#e0e0e0", "black"),
            (2, "Centering",          "#e0e0e0", "black"),
            (3, "Low Tower",          "#9c27b0", "white"),
            (4, "Attack Nexus",       "#e91e63", "white"),
            (5, "Attack Top Tower",   "#ff9800", "white"),
        ]
        for code, label, bg, fg in modes:
            tk.Button(parent, text=label, bg=bg, fg=fg, width=18,
                      font=("", 10),
                      command=lambda c=code: self._send(f"mode {c}")
                      ).pack(pady=2, fill=tk.X)

        ttk.Separator(parent).pack(fill=tk.X, pady=8)

        # Straight movement
        tk.Label(parent, text="Straight Move",
                 font=("", 10, "bold")).pack()

        sf = tk.Frame(parent)
        sf.pack(pady=2)
        tk.Label(sf, text="Inches:").pack(side=tk.LEFT)
        self.straight_var = tk.IntVar(value=12)
        tk.Spinbox(sf, from_=1, to=300,
                   textvariable=self.straight_var,
                   width=5, font=("", 10)).pack(side=tk.LEFT, padx=4)

        tk.Button(parent, text="▶  Go Straight",
                  bg="#2196F3", fg="white", font=("", 10),
                  command=lambda: self._send(
                      f"straight {self.straight_var.get()}")
                  ).pack(fill=tk.X, pady=2)

        ttk.Separator(parent).pack(fill=tk.X, pady=8)

        tk.Button(parent, text="Ping (liveness check)",
                  width=18, font=("", 9),
                  command=lambda: self._send("ping")).pack(pady=2)

    # ── Tuning sliders ───────────────────────────────────────────────

    def _build_tuning(self, parent):
        #  (label, bridge_cmd, min, max, resolution, default, is_int)
        sliders = [
            ("RPM",          "rpm",   0,    130,  1,     85,   True),
            ("Kp",           "kp",    0,    5,    0.05,  1.4,  False),
            ("Ki",           "ki",    0,    5,    0.05,  1.0,  False),
            ("Kd",           "kd",    0,    5,    0.05,  0.0,  False),
            ("wf_Kp",        "wfkp",  0,    5,    0.05,  0.7,  False),
            ("wf_Kd",        "wfkd",  0,    10,   0.05,  1.2,  False),
            ("Sharp Offset", "sharp", 0,    255,  1,     90,   True),
        ]

        tk.Label(parent, text="Tuning",
                 font=("", 10, "bold")).grid(
                 row=0, column=0, columnspan=3,
                 sticky="w", pady=(0, 6))

        self._slider_vars = {}

        for i, (label, cmd, mn, mx, res, default, is_int) in enumerate(
                sliders, start=1):

            tk.Label(parent, text=label + ":", anchor="e",
                     width=13, font=("", 10)).grid(
                     row=i, column=0, sticky="e", padx=(0, 4), pady=4)

            var = tk.DoubleVar(value=default)
            self._slider_vars[cmd] = (var, is_int)

            fmt = "{:.0f}" if is_int else "{:.2f}"
            val_lbl = tk.Label(parent,
                               text=fmt.format(default),
                               width=6, anchor="w", font=("Courier", 10))
            val_lbl.grid(row=i, column=2, sticky="w", padx=(4, 0))

            def make_update(lbl, f):
                return lambda v: lbl.config(text=f.format(float(v)))

            sl = tk.Scale(parent, variable=var,
                          orient=tk.HORIZONTAL,
                          from_=mn, to=mx, resolution=res,
                          length=240, showvalue=False,
                          command=make_update(val_lbl, fmt))
            sl.grid(row=i, column=1, sticky="ew", pady=2)

            def make_release(c, v, ii):
                return lambda e: self._slider_released(c, v, ii)

            sl.bind("<ButtonRelease-1>", make_release(cmd, var, is_int))

        parent.columnconfigure(1, weight=1)

    # ── Log panel ────────────────────────────────────────────────────

    def _build_log(self):
        ttk.Separator(self.root).pack(fill=tk.X, padx=8, pady=(8, 0))

        lf = tk.Frame(self.root)
        lf.pack(fill=tk.BOTH, expand=True, padx=8, pady=(2, 8))

        hdr = tk.Frame(lf)
        hdr.pack(fill=tk.X)
        tk.Label(hdr, text="Bridge log",
                 font=("", 9, "bold")).pack(side=tk.LEFT)
        tk.Button(hdr, text="Clear", font=("", 8),
                  command=self._clear_log).pack(side=tk.RIGHT)

        self.log = tk.Text(lf, height=8, state=tk.DISABLED,
                           font=("Courier", 9),
                           bg="#1e1e1e", fg="#d4d4d4",
                           wrap=tk.WORD)
        sb = ttk.Scrollbar(lf, command=self.log.yview)
        self.log.configure(yscrollcommand=sb.set)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.log.pack(fill=tk.BOTH, expand=True)

        # Color tags
        self.log.tag_config("tx",  foreground="#4fc3f7")  # sent lines → blue
        self.log.tag_config("rx",  foreground="#a5d6a7")  # recv lines → green
        self.log.tag_config("err", foreground="#ef9a9a")  # errors → red

    # ─────────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────────

    def _refresh_ports(self):
        if not SERIAL_OK:
            return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_cb.current(0)

    def _toggle_connect(self):
        if self.bridge.connected:
            self.bridge.disconnect()
            self.conn_btn.config(text="Connect")
            self.status_lbl.config(text="● Disconnected", fg="red")
            self._log_line("Disconnected.", "err")
        else:
            port = self.port_var.get()
            if not port:
                self._log_line("[ERROR] No port selected.", "err")
                return
            ok, err = self.bridge.connect(port)
            if ok:
                self.conn_btn.config(text="Disconnect")
                self.status_lbl.config(text="● Connected", fg="green")
                self._log_line(f"Connected to {port} @ 115200", "rx")
            else:
                self.status_lbl.config(text="● Error", fg="orange")
                self._log_line(f"[ERROR] {err}", "err")

    def _send(self, line: str):
        if not self.bridge.connected:
            self._log_line(f"[NOT CONNECTED]  {line}", "err")
            return
        self.bridge.send(line)
        self._log_line(f"→  {line}", "tx")

    def _dir_press(self, direction: str):
        self._dir_held = True
        self._send(direction)

    def _dir_release(self):
        if self._dir_held:
            self._dir_held = False
            self._send("S")

    def _slider_released(self, cmd: str, var: tk.DoubleVar, is_int: bool):
        val = var.get()
        if is_int:
            self._send(f"{cmd} {int(val)}")
        else:
            self._send(f"{cmd} {val:.2f}")

    def _log_line(self, text: str, tag: str = "rx"):
        self.log.config(state=tk.NORMAL)
        self.log.insert(tk.END, text + "\n", tag)
        # Trim to LOG_MAX lines
        n = int(self.log.index("end-1c").split(".")[0])
        if n > self.LOG_MAX:
            self.log.delete("1.0", f"{n - self.LOG_MAX}.0")
        self.log.see(tk.END)
        self.log.config(state=tk.DISABLED)

    def _clear_log(self):
        self.log.config(state=tk.NORMAL)
        self.log.delete("1.0", tk.END)
        self.log.config(state=tk.DISABLED)

    def _poll_rx(self):
        """Drain the serial RX queue every 50 ms and show in log."""
        try:
            while True:
                line = self.rx_q.get_nowait()
                self._log_line(f"←  {line}", "rx")
        except queue.Empty:
            pass
        self.root.after(50, self._poll_rx)

    def _show_install_warning(self):
        tk.Label(self.root,
                 text="pyserial is not installed.\n\nRun:\n  pip install pyserial\n\nthen restart this script.",
                 font=("", 13), justify=tk.CENTER,
                 fg="red", pady=40).pack()


# ─────────────────────────────────────────────────────────────────────────────

def main():
    root = tk.Tk()
    root.minsize(680, 500)
    app = RobotGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
