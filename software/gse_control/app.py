"""
app.py  --  laptop control GUI (tkinter, standard library only)

Run:   python software/run_gui.py                 # pick a port in the UI
       python software/run_gui.py --port COM5     # the ground box (TX node)
       python software/run_gui.py --sim           # no hardware, simulated pad

The controls follow the GSE `main` control set (see supervisor.py):

    LIVE  FILL [F] + VEH+IGN [L]      (VEH = the fuel + ox vehicle valves)
    TEST  FILL [F] + VEH [S] + IGN [I]
    both  ARM [A], PURGE [P] (if enabled for the session), E-STOP [E],
          RESET CONTROLS [Shift+R], RESET FILL SEQ [R], CLEAR FAULTS [Shift+C]

FILL runs a timed fill (Settings); turning it off early pauses it.  VEH / VEH+IGN
and IGN unlock once the fill is complete or paused.  Pressing VEH+IGN (LIVE) or
IGN (TEST) starts a countdown that HOLDS at T-0: nothing fires until the
operator presses L (or the button) again.  Shift+L / ABORT cancels it.

Every disabled control shows why, in plain words, under it.  The PAD READ-BACK
panel shows what the pad itself reports -- the only truth about the outputs.
"""
from __future__ import annotations

import argparse
import os
import sys
import tkinter as tk
from tkinter import ttk, messagebox

from . import protocol as P
from . import supervisor as S
from .controller import Controller
from .link import list_ports

BG = "#101418"
FG = "#e8eef2"
DIM = "#8fa3b0"
CARD = "#1a2129"
ACC = "#1f6feb"
DANGER = "#c0262b"
OK = "#1d9a4e"
WARN = "#c98a00"
PURPLE = "#7b2fbf"
GREY = "#3a444d"
WHY = "#e0a050"
BTN = "#0d1116"

DEFAULT_SETTINGS = os.path.join(os.path.expanduser("~"), ".gse_simple_settings.json")

# channel -> title, key, on-action, off-action, mode it belongs to (None = both)
CHANNELS = [
    ("fill", "FILL", "F", S.FILL_ON, S.FILL_OFF, None),
    ("srv_ign", "VEH+IGN", "L", S.SRV_IGN_ON, S.SRV_IGN_OFF, S.LIVE),
    ("srv", "VEH", "S", S.SRV_ON, S.SRV_OFF, S.TEST),
    ("ign", "IGN", "I", S.IGN_PWR, S.IGN_SAFE, S.TEST),
    ("arm", "ARM", "A", S.ARM, S.DISARM, None),
    ("purge", "PURGE", "P", S.PURGE_OPEN, S.PURGE_CLOSE, None),
]
SECONDARY = [
    (S.RESET_CONTROLS, "RESET CONTROLS  [Shift+R]", GREY),
    (S.RESET_FILL, "RESET FILL SEQ  [R]", GREY),
    (S.CLEAR_FAULTS, "CLEAR FAULTS  [Shift+C]", GREY),
    (S.ESTOP, "ALL SAFE · E-STOP  [E]", DANGER),
]


def describe_port_error(port: str, exc: Exception) -> str:
    """Turn an open() failure into something an operator can act on.

    The three that actually happen, in order of frequency:
      * the port is already open somewhere else (a terminal, or a previous copy
        of this program that was closed with the window X and never exited);
      * the port is a USB device that has gone away, or is not the TX node;
      * the port name does not exist on this machine (wrong COM number, wrong
        /dev entry, board unplugged).
    """
    msg = str(exc) or exc.__class__.__name__
    low = msg.lower()
    text = f"{port}: {msg}"
    if "permission" in low or "access is denied" in low or "errno 13" in low:
        text += ("\n\nThe port exists but cannot be opened - another program "
                 "has it (a serial terminal, the Arduino IDE, or a previous "
                 "copy of this window). Close it and press Connect again.")
        text += ("\nOn Linux, also check that you are in the dialout group: "
                 "sudo usermod -aG dialout $USER, then log out and in.")
    elif "busy" in low or "resource" in low or "errno 16" in low:
        text += ("\n\nThe port is busy: close whatever else has it open and "
                 "press Connect again.")
    elif "no such file" in low or "not found" in low or "could not open port" in low \
            or "filenotfound" in low or "errno 2" in low:
        text += ("\n\nThat port does not exist. Press Refresh and pick the "
                 "entry for the TX node (it appears as an ESP32-S3 CDC device, "
                 "or a CP210x/CH340 bridge on an ESP-32S). If nothing is listed, "
                 "the board is unplugged or its cable is charge-only.")
    return text


class App(tk.Tk):
    def __init__(self, controller: Controller, sim: bool = False):
        super().__init__()
        self.ctl = controller
        self.sim = sim
        self.title("HCR-5100 GSE - ESP-NOW control" + ("  [SIMULATION]" if sim else ""))
        self.geometry("1260x860")
        self.configure(bg=BG)
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self._build()
        self.after(20, self._tick)

    # ------------------------------------------------------------------ build
    def _card(self, parent, title: str) -> tk.Frame:
        f = tk.Frame(parent, bg=CARD, bd=0, highlightthickness=1,
                     highlightbackground="#263039")
        tk.Label(f, text=title, bg=CARD, fg=DIM, font=("DejaVu Sans", 9, "bold"),
                 anchor="w").pack(fill="x", padx=10, pady=(8, 2))
        return f

    def _build(self):
        pad = dict(padx=8, pady=4)
        top = tk.Frame(self, bg=BG); top.pack(fill="x", **pad)

        # --- connection bar + mode ------------------------------------------
        self.port_var = tk.StringVar(value=self.ctl.port or "")
        self.cmb_port = ttk.Combobox(top, textvariable=self.port_var, width=22,
                                     values=list_ports())
        self.cmb_port.pack(side="left")
        tk.Button(top, text="Refresh", command=self._refresh_ports, bg=GREY, fg=FG,
                  relief="flat").pack(side="left", padx=4)
        self.btn_conn = tk.Button(top, text="Connect", command=self._toggle_conn,
                                  bg=ACC, fg="white", relief="flat", width=12)
        self.btn_conn.pack(side="left", padx=4)
        self.lbl_link = tk.Label(top, text="disconnected", bg=BG, fg=DIM)
        self.lbl_link.pack(side="left", padx=10)
        self.btn_test = tk.Button(top, text="TEST", width=8, relief="flat",
                                  command=lambda: self._set_mode(S.TEST))
        self.btn_test.pack(side="right")
        self.btn_live = tk.Button(top, text="LIVE", width=8, relief="flat",
                                  command=lambda: self._set_mode(S.LIVE))
        self.btn_live.pack(side="right", padx=4)
        tk.Label(top, text="operating mode:", bg=BG, fg=DIM).pack(side="right", padx=4)

        # --- banner ---------------------------------------------------------
        self.banner = tk.Label(self, text="DISCONNECTED", font=("DejaVu Sans", 20, "bold"),
                               bg=GREY, fg="white", pady=10)
        self.banner.pack(fill="x", padx=8, pady=(2, 8))

        body = tk.Frame(self, bg=BG); body.pack(fill="both", expand=True, padx=8)

        # --- left: controls -------------------------------------------------
        left = tk.Frame(body, bg=BG); left.pack(side="left", fill="y")
        ctl = self._card(left, "CONTROLS  (GSE control set: interlocks, fill timer, "
                               "countdown with T-0 HOLD)")
        ctl.pack(fill="x")
        self.lbl_machine = tk.Label(ctl, text="-", bg=CARD, fg=FG, anchor="w",
                                    font=("DejaVu Sans Mono", 10, "bold"))
        self.lbl_machine.pack(fill="x", padx=12, pady=(0, 6))

        grid = tk.Frame(ctl, bg=CARD); grid.pack(fill="x", padx=10)
        self.ch: dict[str, dict] = {}
        for i, (cid, title, key, _on, _off, _mode) in enumerate(CHANNELS):
            btn = tk.Button(grid, text=title, width=27, anchor="w", relief="flat",
                            bg=BTN, fg=FG, font=("DejaVu Sans", 10, "bold"),
                            command=lambda c=cid: self._click(c))
            btn.grid(row=i * 2, column=0, sticky="w", pady=(4, 0))
            led = tk.Label(grid, text="-", width=17, bg=CARD, fg=FG, anchor="w",
                           font=("DejaVu Sans Mono", 10, "bold"))
            led.grid(row=i * 2, column=1, sticky="w", padx=8, pady=(4, 0))
            desc = tk.Label(grid, text="", bg=CARD, fg=DIM, anchor="w", wraplength=250,
                            justify="left")
            desc.grid(row=i * 2, column=2, sticky="w", pady=(4, 0))
            why = tk.Label(grid, text="", bg=CARD, fg=WHY, anchor="w", wraplength=560,
                           justify="left", font=("DejaVu Sans", 8))
            why.grid(row=i * 2 + 1, column=0, columnspan=3, sticky="w", padx=(4, 0))
            self.ch[cid] = dict(btn=btn, led=led, desc=desc, why=why, title=title, key=key)

        # countdown banner
        cdf = tk.Frame(ctl, bg=CARD); cdf.pack(fill="x", padx=10, pady=(8, 2))
        self.lbl_cd = tk.Label(cdf, text="no countdown", bg=CARD, fg=DIM, anchor="w",
                               font=("DejaVu Sans", 12, "bold"), justify="left")
        self.lbl_cd.pack(side="left", fill="x", expand=True)
        self.btn_abort = tk.Button(cdf, text="ABORT  [Shift+L]", bg=GREY, fg="white",
                                   relief="flat", command=lambda: self.ctl.act(S.ABORT_COUNTDOWN))
        self.btn_abort.pack(side="right")

        # fill sequence bar
        ff = tk.Frame(ctl, bg=CARD); ff.pack(fill="x", padx=10, pady=(6, 2))
        tk.Label(ff, text="FILL SEQUENCE", bg=CARD, fg=DIM,
                 font=("DejaVu Sans", 9, "bold")).pack(anchor="w")
        self.bar_fill = ttk.Progressbar(ff, length=560, maximum=1000, mode="determinate")
        self.bar_fill.pack(fill="x", pady=2)
        self.lbl_fill = tk.Label(ff, text="idle", bg=CARD, fg=FG, anchor="w")
        self.lbl_fill.pack(fill="x")

        # secondary actions
        sec = tk.Frame(ctl, bg=CARD); sec.pack(fill="x", padx=10, pady=(8, 4))
        self.sec: dict[str, tuple] = {}
        for i, (action, text, colour) in enumerate(SECONDARY):
            b = tk.Button(sec, text=text, bg=colour, fg="white", relief="flat",
                          font=("DejaVu Sans", 9, "bold"),
                          command=lambda a=action: self.ctl.act(a))
            b.grid(row=0, column=i, sticky="we", padx=2)
            w = tk.Label(sec, text="", bg=CARD, fg=WHY, wraplength=140, justify="left",
                         font=("DejaVu Sans", 8))
            w.grid(row=1, column=i, sticky="nw", padx=2)
            self.sec[action] = (b, w, colour)

        self.lbl_keys = tk.Label(ctl, text="", bg=CARD, fg=DIM, anchor="w",
                                 font=("DejaVu Sans", 8))
        self.lbl_keys.pack(fill="x", padx=12, pady=(2, 8))

        # --- right: telemetry ----------------------------------------------
        right = tk.Frame(body, bg=BG); right.pack(side="left", fill="both",
                                                  expand=True, padx=(8, 0))
        tel = self._card(right, "PAD READ-BACK  (what the pad reports)")
        tel.pack(fill="x")
        telg = tk.Frame(tel, bg=CARD); telg.pack(fill="x", pady=(0, 8))
        self.tel = {}
        rows = [("pad_state", "pad state"), ("age", "last valid CMD"),
                ("link", "link"), ("armed", "arm interlock"),
                ("ign", "igniter relay"), ("fills", "FILL"),
                ("vehs", "VEHICLE"), ("prgs", "PURGE"),
                ("rx", "valid CMDs"), ("crc", "rejected frames"),
                ("fw", "firmware"), ("uptime", "pad uptime"),
                ("cfg", "pad config")]
        for i, (k, label) in enumerate(rows):
            r, c = divmod(i, 2)
            tk.Label(telg, text=label, bg=CARD, fg=DIM, anchor="w").grid(
                row=r, column=c * 2, sticky="w", padx=(12, 6), pady=2)
            v = tk.Label(telg, text="-", bg=CARD, fg=FG, anchor="w",
                         font=("DejaVu Sans Mono", 10, "bold"))
            v.grid(row=r, column=c * 2 + 1, sticky="w", padx=(0, 24), pady=2)
            self.tel[k] = v

        cfg = self._card(right, "PAD CONFIGURATION  (kept in sync with the pad)")
        cfg.pack(fill="x", pady=(8, 0))
        cfgg = tk.Frame(cfg, bg=CARD); cfgg.pack(fill="x", pady=(0, 4))
        self.v_tf = tk.IntVar(value=self.ctl.cfg.travel_fill)
        self.v_tv = tk.IntVar(value=self.ctl.cfg.travel_veh)
        self.v_tp = tk.IntVar(value=self.ctl.cfg.travel_prg)
        self.v_wd = tk.IntVar(value=self.ctl.cfg.wdg_ds)
        self.v_ig = tk.IntVar(value=self.ctl.cfg.ign_max_ds)
        for i, (label, var, lo, hi) in enumerate([
                ("FILL travel (deg)", self.v_tf, 5, 90),
                ("VEH travel (deg)", self.v_tv, 5, 90),
                ("PRG travel (deg)", self.v_tp, 5, 90),
                ("link watchdog (0.1 s)", self.v_wd, P.WDG_DS_MIN, P.WDG_DS_MAX),
                ("igniter limit (0.1 s)", self.v_ig, P.IGN_DS_MIN, P.IGN_DS_MAX)]):
            tk.Label(cfgg, text=label, bg=CARD, fg=DIM, anchor="w").grid(
                row=i, column=0, sticky="w", padx=12, pady=1)
            tk.Spinbox(cfgg, from_=lo, to=hi, textvariable=var, width=6,
                       bg=BTN, fg=FG, relief="flat",
                       buttonbackground="#263039", readonlybackground=BTN,
                       insertbackground=FG).grid(row=i, column=1, sticky="w", pady=1)
        tk.Button(cfgg, text="Apply to pad", bg=ACC, fg="white", relief="flat",
                  command=self._apply_cfg).grid(row=6, column=0, columnspan=2,
                                                pady=(6, 4))
        # Purge is enabled per session only, never remembered (ECR-01 H8).
        self.v_purge = tk.BooleanVar(value=self.ctl.sup.purge_enabled)
        self.chk_purge = tk.Checkbutton(
            cfg, text="purge valve fitted & enabled for this session "
                      "(never with the vehicle connected)",
            wraplength=360, justify="left",
            bg=CARD, fg=DIM, selectcolor=CARD, activebackground=CARD,
            activeforeground=FG, variable=self.v_purge, command=self._enable_purge)
        self.chk_purge.pack(anchor="w", padx=12, pady=(0, 8))

        seq = self._card(right, "SEQUENCE SETTINGS  (remembered between sessions)")
        seq.pack(fill="x", pady=(8, 0))
        seqg = tk.Frame(seq, bg=CARD); seqg.pack(fill="x", pady=(0, 8))
        st = self.ctl.sup.settings
        self.v_fs = tk.IntVar(value=st.fill_seconds)
        self.v_cs = tk.IntVar(value=st.countdown_seconds)
        self.v_ci = tk.BooleanVar(value=st.confirm_ign)
        for i, (label, var, lo, hi) in enumerate([
                ("fill time (s)", self.v_fs, S.MIN_FILL_SECONDS, S.MAX_FILL_SECONDS),
                ("ignition countdown (s)", self.v_cs, S.MIN_COUNTDOWN_SECONDS,
                 S.MAX_COUNTDOWN_SECONDS)]):
            tk.Label(seqg, text=label, bg=CARD, fg=DIM, anchor="w").grid(
                row=i, column=0, sticky="w", padx=12, pady=1)
            tk.Spinbox(seqg, from_=lo, to=hi, textvariable=var, width=6,
                       bg=BTN, fg=FG, relief="flat",
                       buttonbackground="#263039", readonlybackground=BTN,
                       insertbackground=FG).grid(row=i, column=1, sticky="w", pady=1)
        tk.Checkbutton(seqg, text="ask before IGN PWR (TEST mode)", bg=CARD, fg=DIM,
                       selectcolor=CARD, activebackground=CARD, activeforeground=FG,
                       variable=self.v_ci).grid(row=2, column=0, columnspan=2,
                                                sticky="w", padx=8)
        tk.Button(seqg, text="Apply settings", bg=ACC, fg="white", relief="flat",
                  command=self._apply_settings).grid(row=3, column=0, columnspan=2,
                                                     pady=(6, 0))

        # --- log -------------------------------------------------------------
        logf = self._card(self, "EVENT LOG  (NODE = a line from the ground board)")
        logf.pack(fill="both", expand=True, padx=8, pady=8)
        # Ground-board console: typed lines go to the TX node's text console
        # (PING, INFO, HELP, SAFE, ...); its replies appear as NODE lines.
        con = tk.Frame(logf, bg=CARD); con.pack(side="bottom", fill="x", padx=8, pady=(0, 8))
        tk.Label(con, text="ground board console:", bg=CARD, fg=DIM).pack(side="left")
        self.v_console = tk.StringVar(value="")
        self.ent_console = tk.Entry(con, textvariable=self.v_console, width=28, bg=BTN,
                                    fg=FG, insertbackground=FG, relief="flat")
        self.ent_console.pack(side="left", padx=6)
        self.ent_console.bind("<Return>", lambda e: self._console_send())
        tk.Button(con, text="Send", bg=ACC, fg="white", relief="flat",
                  command=self._console_send).pack(side="left")
        for cmd in ("PING", "INFO", "HELP"):
            tk.Button(con, text=cmd, bg=GREY, fg=FG, relief="flat", width=6,
                      command=lambda c=cmd: self.ctl.console(c)).pack(side="left", padx=(6, 0))
        tk.Label(con, text="PING → PONG with round-trip time · INFO → counters "
                           "(fwd = commands the board received)", bg=CARD, fg=DIM,
                 font=("DejaVu Sans", 8)).pack(side="left", padx=10)
        self.txt = tk.Text(logf, height=10, bg=BTN, fg="#cfe0ea", relief="flat",
                           font=("DejaVu Sans Mono", 9))
        self.txt.pack(fill="both", expand=True, padx=8, pady=(0, 4))

        self.bind_all("<KeyPress>", self._on_key)

    # ---------------------------------------------------------------- actions
    def _next_action(self, cid: str) -> str:
        """What a click on a channel button does (app.js renderToggle)."""
        s = self.ctl.sup
        if cid == "fill":
            return S.FILL_OFF if s.fill else S.FILL_ON
        if cid == "srv_ign":
            counting = s.cd_target == S.CD_SRV_IGN and not s.cd_holding
            return S.SRV_IGN_OFF if (s.srv or s.ign or counting) else S.SRV_IGN_ON
        if cid == "srv":
            return S.SRV_OFF if s.srv else S.SRV_ON
        if cid == "ign":
            counting = s.cd_target == S.CD_IGN and not s.cd_holding
            return S.IGN_SAFE if (s.ign or counting) else S.IGN_PWR
        if cid == "arm":
            return S.DISARM if s.armed else S.ARM
        return S.PURGE_CLOSE if s.purge_open else S.PURGE_OPEN

    def _click(self, cid: str):
        action = self._next_action(cid)
        if action == S.IGN_PWR:
            self._request_ign()
        else:
            self.ctl.act(action)

    def _request_ign(self):
        """TEST-mode IGN PWR, with the optional confirmation (app.js requestIgn)."""
        g = self.ctl.gates()[S.IGN_PWR]
        if g.enabled and self.ctl.sup.settings.confirm_ign:
            s = self.ctl.sup
            if s.cd_holding:
                q = ("Countdown is at T-0.0 s.\n\nTrigger IGN PWR now? The pad fires "
                     "the igniter once.")
            else:
                q = (f"SRV is ON and FILL is ready.\n\nStart the "
                     f"{s.settings.countdown_seconds} s IGN countdown? It holds at T-0.0 s; "
                     "ignition then needs a separate manual trigger (L).")
            if not messagebox.askyesno("confirm ignition", q):
                return
        self.ctl.act(S.IGN_PWR)

    def _toggle_key(self, on: str, off: str):
        """app.js:325-362 -- on-action if enabled, else off-action, else say why."""
        g = self.ctl.gates()
        if g[on].enabled:
            if on == S.IGN_PWR:
                self._request_ign()
            else:
                self.ctl.act(on)
        elif g[off].enabled:
            self.ctl.act(off)
        else:
            self.ctl.act(on)                 # logs "<label> blocked: <reason>"

    def _on_key(self, event):
        try:
            cls = event.widget.winfo_class()
        except Exception:                                   # noqa: BLE001
            cls = ""
        if cls in ("Entry", "Spinbox", "TCombobox", "TEntry", "TSpinbox", "Text"):
            return
        if event.state & 0x4 or event.state & 0x8:          # Ctrl / Alt
            return
        key = event.keysym
        if key == "Escape":                                 # gse-simple extra
            self._panic()
            return
        if len(key) != 1:
            return
        shift = bool(event.state & 0x1)
        key = key.upper()
        s = self.ctl.sup
        # hard-coded while a countdown exists (app.js:1452-1488)
        if s.cd_target and key == "L":
            if shift:
                self.ctl.act(S.ABORT_COUNTDOWN)
            elif s.cd_holding:
                if s.cd_target == S.CD_IGN:
                    self._request_ign()
                else:
                    self.ctl.act(S.SRV_IGN_ON)
            return                                          # L while counting: swallowed
        live = s.mode == S.LIVE
        if not shift:
            if key == "F":
                self._toggle_key(S.FILL_ON, S.FILL_OFF)
            elif key == "L" and live:
                self._toggle_key(S.SRV_IGN_ON, S.SRV_IGN_OFF)
            elif key == "S" and not live:
                self._toggle_key(S.SRV_ON, S.SRV_OFF)
            elif key == "I" and not live:
                self._toggle_key(S.IGN_PWR, S.IGN_SAFE)
            elif key == "E":
                self.ctl.act(S.ESTOP)
            elif key == "R":
                self.ctl.act(S.RESET_FILL)
            elif key == "A":
                g = self.ctl.gates()
                self.ctl.act(S.DISARM if g[S.DISARM].enabled else S.ARM)
            elif key == "P":
                g = self.ctl.gates()
                self.ctl.act(S.PURGE_CLOSE if g[S.PURGE_CLOSE].enabled else S.PURGE_OPEN)
        else:
            if key == "R":
                self.ctl.act(S.RESET_CONTROLS)
            elif key == "C":
                self.ctl.act(S.CLEAR_FAULTS)

    def _panic(self):
        """ESC: abort any countdown and disarm (kept from the GSE-simple GUI)."""
        s = self.ctl.sup
        if s.cd_target:
            self.ctl.act(S.ABORT_COUNTDOWN)
        if s.armed:
            self.ctl.act(S.DISARM)
        self.ctl.note("ESC: countdown aborted (if any), igniter disarmed")

    def _console_send(self):
        if self.ctl.console(self.v_console.get()):
            self.v_console.set("")

    def _set_mode(self, mode: str):
        self.ctl.set_mode(mode)

    def _refresh_ports(self):
        self.cmb_port["values"] = list_ports()

    def _toggle_conn(self):
        if self.ctl.link:
            self.ctl.disconnect()
            self.btn_conn.config(text="Connect", bg=ACC)
        else:
            port = self.port_var.get().strip() or ("SIM" if self.sim else None)
            if not port:
                messagebox.showwarning(
                    "Port",
                    "Select the TX-node USB port first "
                    "(it appears as an ESP32-S3 CDC device, or a CP210x/CH340 "
                    "bridge on an ESP-32S board).\n\n"
                    "Nothing listed? Press Refresh; if it is still empty the "
                    "board is unplugged, the cable is charge-only, or the USB "
                    "driver is missing.\n\n"
                    "No hardware at all? Run with --sim to rehearse against the "
                    "built-in pad simulator.")
                return
            try:
                self.ctl.connect(port)
            except Exception as exc:                       # noqa: BLE001
                why = describe_port_error(port, exc)
                self._log(f"connect failed: {why}")
                self.ctl.note(f"connect failed: {why}")
                messagebox.showerror("Connect failed", why)
                return
            self.btn_conn.config(text="Disconnect", bg=DANGER)
        self.lbl_link.config(text=self.ctl.ui.link_name)

    def _enable_purge(self):
        """Tick box -> PURGE ENABLE / DISABLE for this session; the pad is told
        through its configuration (allow_purge), which is re-sent until it
        reports it in force."""
        self.ctl.act(S.PURGE_ENABLE if self.v_purge.get() else S.PURGE_DISABLE)
        self.v_purge.set(self.ctl.sup.purge_enabled)

    def _apply_cfg(self):
        self.ctl.apply_cfg(travel_fill=self.v_tf.get(), travel_veh=self.v_tv.get(),
                           travel_prg=self.v_tp.get(), wdg_ds=self.v_wd.get(),
                           ign_max_ds=self.v_ig.get())

    def _apply_settings(self):
        try:
            fs, cs = int(self.v_fs.get()), int(self.v_cs.get())
        except (ValueError, tk.TclError):
            self.ctl.note("Settings not applied: fill and countdown times must be whole seconds")
            return
        self.ctl.apply_settings(fs, cs, bool(self.v_ci.get()))
        st = self.ctl.sup.settings
        self.v_fs.set(st.fill_seconds)
        self.v_cs.set(st.countdown_seconds)

    def on_close(self):
        try:
            self.ctl.disconnect()
        finally:
            self.destroy()

    # ------------------------------------------------------------------- tick
    def _tick(self):
        try:
            self.ctl.tick()
        except Exception as exc:                            # noqa: BLE001
            self._log(f"tick error: {exc}")
        self._paint()
        self.after(20, self._tick)

    def _paint(self):
        colour, text = self.ctl.verdict()
        bg = {"grey": GREY, "red": DANGER, "amber": WARN, "green": OK,
              "purple": PURPLE}[colour]
        self.banner.config(text=text, bg=bg)
        self._paint_controls()
        self._paint_readback()
        self.lbl_link.config(
            text=f"{self.ctl.ui.link_name}  |  {self.ctl.link.stats.as_text()}"
            if self.ctl.link else "disconnected")
        while self.ctl.ui.log_lines:
            self._log(self.ctl.ui.log_lines.pop(0))

    def _paint_controls(self):
        s = self.ctl.sup
        g = self.ctl.gates()
        live = s.mode == S.LIVE
        self.btn_live.config(bg=ACC if live else GREY, fg="white")
        self.btn_test.config(bg=ACC if not live else GREY, fg="white")
        self.lbl_machine.config(text=s.machine_text())

        cd_rem = s.countdown_remaining_s()
        for cid, title, key, on, off, mode in CHANNELS:
            w = self.ch[cid]
            nxt = self._next_action(cid)
            gate = g[nxt]
            if mode is not None and mode != s.mode:
                led, desc, ledc = "-", f"{mode.upper()} mode only", DIM
            else:
                led, desc, ledc = self._channel_text(cid, cd_rem)
            label = f"{title}  [{key}]  → {S.LABEL[nxt].split(' ', 1)[-1]}"
            if nxt in (S.SRV_IGN_ON, S.IGN_PWR) and s.cd_holding:
                label = f"{title}  [L]  → TRIGGER"
            elif nxt in (S.SRV_IGN_OFF, S.IGN_SAFE) and s.cd_target and not s.cd_holding \
                    and ((cid == "srv_ign" and s.cd_target == S.CD_SRV_IGN)
                         or (cid == "ign" and s.cd_target == S.CD_IGN)):
                label = f"{title}  [Shift+L]  → ABORT"
            danger = cid in ("srv_ign", "ign", "arm")
            w["btn"].config(text=label, state="normal" if gate.enabled else "disabled",
                            bg=(DANGER if danger else ACC) if gate.enabled else BTN)
            w["led"].config(text=led, fg=ledc)
            w["desc"].config(text=desc)
            w["why"].config(text="" if gate.enabled else gate.reason)

        # countdown
        if s.cd_target:
            t = "VEH+IGN" if s.cd_target == S.CD_SRV_IGN else "IGN"
            if s.cd_holding:
                self.lbl_cd.config(text=f">> IGNITION HOLD · MANUAL TRIGGER REQUIRED   "
                                        f"HOLD · 0.0 s\nPress L to trigger {t} · Shift+L or "
                                        "ABORT to cancel", fg="white", bg=PURPLE)
            else:
                head = (">> LIVE LAUNCH COUNTDOWN · VEH+IGN -> HOLD" if s.cd_target == S.CD_SRV_IGN
                        else ">> IGNITION COUNTDOWN · IGN -> HOLD")
                self.lbl_cd.config(text=f"{head}   T - {cd_rem:.1f} s\nOutputs stay as they "
                                        "are until T-0 and the manual trigger · Shift+L, "
                                        "ABORT or E-STOP cancels", fg="white", bg=DANGER)
        else:
            self.lbl_cd.config(text="no countdown", fg=DIM, bg=CARD)
        ga = g[S.ABORT_COUNTDOWN]
        self.btn_abort.config(state="normal" if ga.enabled else "disabled",
                              bg=WARN if ga.enabled else GREY)

        # fill sequence
        self.bar_fill["value"] = int(s.fill_fraction() * 1000)
        self.lbl_fill.config(text=f"{s.fill_status()}  ·  {s.fill_total_s()} s fill")

        for action, (b, why, colour) in self.sec.items():
            gate = g[action]
            b.config(state="normal" if gate.enabled else "disabled",
                     bg=colour if gate.enabled else BTN)
            why.config(text="" if gate.enabled else gate.reason)

        keys = ("keys (LIVE): F fill · L veh+ign · " if live
                else "keys (TEST): F fill · S veh · I ign · ")
        self.lbl_keys.config(text=keys + "E e-stop · Shift+R reset all · R reset fill · "
                                          "Shift+C faults · A arm · P purge · "
                                          "Shift+L abort countdown · Esc abort+disarm")

    def _channel_text(self, cid: str, cd_rem: float) -> tuple[str, str, str]:
        """(LED text, description, LED colour) for one channel row."""
        s = self.ctl.sup
        rb = s.readback
        counting = None
        if s.cd_target and ((cid == "srv_ign" and s.cd_target == S.CD_SRV_IGN)
                            or (cid == "ign" and s.cd_target == S.CD_IGN)):
            counting = "HOLD · 0.0s" if s.cd_holding else f"COUNTDOWN · {cd_rem:.1f}s"
        if cid == "fill":
            if s.fill:
                return "ON", "Fill valve open · click → PAUSE/OFF", WARN
            if s.fill_paused:
                return ("PAUSED", f"Paused ({s.fill_target_ms / 1000:.1f} s left) · "
                                  "click → RESUME", WARN)
            return "OFF", "Fill valve servo · click → ON", FG
        if cid == "srv_ign":
            if counting:
                return counting, "Countdown · Shift+L / click → ABORT", PURPLE
            led = f"{'OPEN' if s.srv else 'CLOSED'}/{'PWR' if s.ign else 'SAFE'}"
            if s.srv or s.ign:
                return led, "ENERGISED (VEH OPEN + IGN) · click → OFF", PURPLE
            return led, (f"Ready · press for {s.settings.countdown_seconds} s countdown → HOLD"
                         if self.ctl.gates()[S.SRV_IGN_ON].enabled
                         else "Combined launch channel · locked"), FG
        if cid == "srv":
            return (("OPEN", "Fuel + ox valves OPEN · click → CLOSE", WARN) if s.srv else
                    ("CLOSED", "Fuel + ox vehicle valves · click → OPEN", FG))
        if cid == "ign":
            if counting:
                return counting, "Countdown · Shift+L / click → ABORT", PURPLE
            if s.ign:
                return "PWR", "IGNITER ENERGISED · click → SAFE", PURPLE
            return "SAFE", (f"Igniter armed · press for {s.settings.countdown_seconds} s "
                            "countdown → HOLD" if self.ctl.gates()[S.IGN_PWR].enabled
                            else "Igniter power · locked"), FG
        if cid == "arm":
            if s.armed:
                return ("ARMED", "ARMED · arm code in every command · click → DISARM"
                        if rb is not None and rb.armed
                        else "ARM requested · waiting for fresh pad telemetry", OK)
            return "SAFE", "Igniter arm interlock · click → ARM", FG
        # purge
        if not s.purge_enabled:
            return "DISABLED", "CO2 purge · disabled for this session (Pad configuration)", DIM
        open_rb = bool(rb is not None and rb.out_prg)
        if s.purge_open:
            return ("OPEN" if open_rb else "OPENING",
                    "Purge valve OPEN · click → CLOSE" if open_rb
                    else "Awaiting pad readback → OPEN", WARN)
        return "CLOSED", "CO2 purge valve (static fire only) · click → OPEN", FG

    def _paint_readback(self):
        st = self.ctl.pad

        def setv(key, text, colour=FG):
            self.tel[key].config(text=text, fg=colour)

        if st is None:
            for k in self.tel:
                setv(k, "-", DIM)
            return
        names = {P.PAD_BOOT: "BOOT", P.PAD_WAIT_HELLO: "WAIT HELLO",
                 P.PAD_RUN: "RUN", P.PAD_LATCHED: "LATCHED/FAULT"}
        setv("pad_state", names.get(st.pad_state, str(st.pad_state)))
        setv("age", f"{st.age_ms if st.age_ms != 0xFFFF else -1} ms",
             OK if st.age_ms != 0xFFFF and st.age_ms < st.wdg_ds * 100 else DANGER)
        setv("link", "OK" if st.link_ok else "LOST", OK if st.link_ok else DANGER)
        setv("armed", "ARMED" if st.armed else "inhibited", OK if st.armed else DIM)
        setv("ign", "ENERGISED" if st.ign_on else "off", PURPLE if st.ign_on else DIM)
        setv("fills", "OPEN" if st.out_fill else "CLOSED", WARN if st.out_fill else FG)
        setv("vehs", "OPEN" if st.out_veh else "CLOSED", WARN if st.out_veh else FG)
        setv("prgs", "OPEN" if st.out_prg else "CLOSED", WARN if st.out_prg else FG)
        setv("rx", str(st.rx_count))
        setv("crc", str(st.crc_err), DANGER if st.crc_err else FG)
        setv("fw", f"{st.fw_major}.{st.fw_minor}")
        setv("uptime", f"{st.uptime_s} s")
        synced = self.ctl.cfg_synced()
        setv("cfg", "IN SYNC" if synced else "SENDING", OK if synced else WARN)

    def _log(self, line: str):
        self.txt.insert("end", line + "\n")
        self.txt.see("end")
        if int(self.txt.index("end-1c").split(".")[0]) > 600:
            self.txt.delete("1.0", "100.0")


def main(argv=None):
    ap = argparse.ArgumentParser(description="HCR-5100 GSE laptop control")
    ap.add_argument("--port", help="serial port of the TX node (e.g. COM5, /dev/ttyUSB0)")
    ap.add_argument("--sim", action="store_true", help="use the built-in pad simulator")
    ap.add_argument("--log-dir", default="logs", help="CSV log directory (default ./logs)")
    ap.add_argument("--settings", default=DEFAULT_SETTINGS,
                    help=f"settings file (default {DEFAULT_SETTINGS})")
    args = ap.parse_args(argv)

    ctl = Controller(port="SIM" if args.sim else args.port, log_dir=args.log_dir,
                     settings_path=args.settings)
    app = App(ctl, sim=args.sim)
    if args.sim:
        ctl.connect("SIM")
        app.btn_conn.config(text="Disconnect", bg=DANGER)
        app.lbl_link.config(text=ctl.ui.link_name)
    app.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
