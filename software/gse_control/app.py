"""
app.py  --  laptop control GUI (tkinter, standard library only)

Run:   python software/run_gui.py                 # pick a port in the UI
       python software/run_gui.py --port COM7     # or /dev/ttyACM0
       python software/run_gui.py --sim           # no hardware, simulated pad

Layout is deliberately boring and high-contrast: this is a range tool, not a
dashboard.  The three things that matter are always on screen:

    1. a big state banner (link / armed / comms loss / igniter energised)
    2. physical switch positions with their pad read-back
    3. the igniter, which is hold-to-fire and requires ARM

Everything shown in grey/italic is *requested*; everything shown solid is
*reported by the pad*.
"""
from __future__ import annotations

import argparse
import sys
import time
import tkinter as tk
from tkinter import ttk, messagebox

from . import protocol as P
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


def ctl_purge_enabled(ctl) -> bool:
    """Initial tick state of the "purge valve fitted" box.

    Taken from the controller's own gate (`ui.purge_enabled`), which is what
    `Controller.set_prg()` actually checks -- not from the widget, and not from
    a constant.  Starting the box ticked while the controller still has purge
    blocked would show the operator a tick they can act on and then silently
    refuse; the two must be born equal.
    """
    return bool(getattr(getattr(ctl, "ui", None), "purge_enabled", False))


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
        self.title("HCR-5100 GSE - LoRa control" + ("  [SIMULATION]" if sim else ""))
        self.geometry("1080x680")
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

        # --- connection bar ------------------------------------------------
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

        # --- banner ---------------------------------------------------------
        self.banner = tk.Label(self, text="DISCONNECTED", font=("DejaVu Sans", 20, "bold"),
                               bg=GREY, fg="white", pady=10)
        self.banner.pack(fill="x", padx=8, pady=(2, 8))

        body = tk.Frame(self, bg=BG); body.pack(fill="both", expand=True, padx=8)

        # --- left: controls -------------------------------------------------
        left = tk.Frame(body, bg=BG); left.pack(side="left", fill="y")

        sw = self._card(left, "SWITCH POSITIONS  (pad mapping in brackets)")
        sw.pack(fill="x", pady=(0, 8))
        self.b_fill = self._toggle(sw, "FILL VALVE   [SwB / Ch5]",
                                   lambda v: self.ctl.set_fill(v))
        self.b_veh = self._toggle(sw, "VEHICLE VALVES   [SwD / Ch6]",
                                  lambda v: self.ctl.set_veh(v))
        self.b_prg = self._toggle(sw, "PURGE VALVE   [SwA / Ch8]",
                                  lambda v: self.ctl.set_prg(v))
        # A Checkbutton is a *widget*, not a variable: it has no .get() of its
        # own (cget() reads widget options, not state).  The state lives in a
        # BooleanVar that both the widget and the callbacks read, so the value
        # and the tick mark can never disagree.  (Reading self.chk_purge.get()
        # raised "AttributeError: 'Checkbutton' object has no attribute 'get'"
        # on every Apply and on every purge toggle.)
        self.v_purge = tk.BooleanVar(value=ctl_purge_enabled(self.ctl))
        self.chk_purge = tk.Checkbutton(
            sw, text="purge valve fitted (enables SwA)", bg=CARD, fg=DIM,
            selectcolor=CARD, activebackground=CARD, activeforeground=FG,
            variable=self.v_purge, command=self._enable_purge)
        self.chk_purge.pack(anchor="w", padx=12, pady=(0, 8))

        ig = self._card(left, "IGNITER  [SwC + stick lockout / Ch3]")
        ig.pack(fill="x")
        row = tk.Frame(ig, bg=CARD); row.pack(fill="x", padx=10, pady=6)
        self.btn_arm = tk.Button(row, text="ARM", width=10, bg=GREY, fg="white",
                                 relief="flat", font=("DejaVu Sans", 12, "bold"),
                                 command=self._toggle_arm)
        self.btn_arm.pack(side="left")
        self.btn_ign = tk.Button(row, text="HOLD TO FIRE", width=16, bg=GREY, fg="white",
                                 relief="flat", font=("DejaVu Sans", 12, "bold"))
        self.btn_ign.pack(side="left", padx=8)
        self.btn_ign.bind("<ButtonPress-1>", lambda e: self._ign(True))
        self.btn_ign.bind("<ButtonRelease-1>", lambda e: self._ign(False))
        tk.Label(ig, text="Relay is hard-limited to the configured on-time and is "
                          "dropped on link loss.", bg=CARD, fg=DIM, wraplength=330,
                 justify="left").pack(anchor="w", padx=12, pady=(0, 8))

        # --- right: telemetry ----------------------------------------------
        right = tk.Frame(body, bg=BG); right.pack(side="left", fill="both",
                                                  expand=True, padx=(8, 0))
        tel = self._card(right, "PAD READ-BACK  (solid = reported by the pad)")
        tel.pack(fill="x")
        telg = tk.Frame(tel, bg=CARD); telg.pack(fill="x", pady=(0, 8))
        self.tel = {}
        rows = [("pad_state", "pad state"), ("age", "last valid CMD"),
                ("link", "link"), ("armed", "arm interlock"),
                ("ign", "igniter relay"), ("fills", "FILL"),
                ("vehs", "VEHICLE"), ("prgs", "PURGE"),
                ("rx", "valid CMDs"), ("crc", "rejected frames"),
                ("fw", "firmware"), ("uptime", "pad uptime")]
        for i, (k, label) in enumerate(rows):
            r, c = divmod(i, 2)
            tk.Label(telg, text=label, bg=CARD, fg=DIM, anchor="w").grid(
                row=r, column=c * 2, sticky="w", padx=(12, 6), pady=2)
            v = tk.Label(telg, text="-", bg=CARD, fg=FG, anchor="w",
                         font=("DejaVu Sans Mono", 10, "bold"))
            v.grid(row=r, column=c * 2 + 1, sticky="w", padx=(0, 24), pady=2)
            self.tel[k] = v

        cfg = self._card(right, "PAD CONFIGURATION  (applies immediately)")
        cfg.pack(fill="x", pady=(8, 0))
        cfgg = tk.Frame(cfg, bg=CARD); cfgg.pack(fill="x", pady=(0, 8))
        self.v_tf = tk.IntVar(value=45); self.v_tv = tk.IntVar(value=45)
        self.v_tp = tk.IntVar(value=45); self.v_wd = tk.IntVar(value=P.DEF_WDG_DS)
        self.v_ig = tk.IntVar(value=P.DEF_IGN_MAX_DS)
        for i, (label, var, lo, hi) in enumerate([
                ("FILL travel (deg)", self.v_tf, 5, 90),
                ("VEH travel (deg)", self.v_tv, 5, 90),
                ("PRG travel (deg)", self.v_tp, 5, 90),
                ("link watchdog (0.1 s)", self.v_wd, P.WDG_DS_MIN, P.WDG_DS_MAX),
                ("igniter limit (0.1 s)", self.v_ig, P.IGN_DS_MIN, P.IGN_DS_MAX)]):
            tk.Label(cfgg, text=label, bg=CARD, fg=DIM, anchor="w").grid(
                row=i, column=0, sticky="w", padx=12, pady=1)
            tk.Spinbox(cfgg, from_=lo, to=hi, textvariable=var, width=6,
                       bg="#0d1116", fg=FG, relief="flat",
                       buttonbackground="#263039", readonlybackground="#0d1116",
                       insertbackground=FG).grid(row=i, column=1, sticky="w", pady=1)
        tk.Button(cfgg, text="Apply to pad", bg=ACC, fg="white", relief="flat",
                  command=self._apply_cfg).grid(row=6, column=0, columnspan=2,
                                                pady=(6, 8))

        # --- log -------------------------------------------------------------
        logf = self._card(self, "EVENT LOG")
        logf.pack(fill="both", expand=False, padx=8, pady=8)
        self.txt = tk.Text(logf, height=8, bg="#0d1116", fg="#cfe0ea", relief="flat",
                           font=("DejaVu Sans Mono", 9))
        self.txt.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        # keyboard: ESC = release igniter + disarm
        self.bind("<Escape>", lambda e: self._panic())

    def _toggle(self, parent, label, cb):
        var = tk.BooleanVar(value=False)
        btn = tk.Button(parent, text=f"{label}:  CLOSED", anchor="w", width=38,
                        bg="#0d1116", fg=FG, relief="flat",
                        command=lambda: self._flip(btn, var, label, cb))
        btn.pack(fill="x", padx=10, pady=2)
        return btn

    @staticmethod
    def _flip(btn, var, label, cb):
        var.set(not var.get())
        cb(var.get())
        btn.config(text=f"{label}:  {'OPEN' if var.get() else 'CLOSED'}",
                   bg="#12351f" if var.get() else "#0d1116")

    # ---------------------------------------------------------------- actions
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
                # "unable to connect" with a bare OSError string is not enough
                # to act on.  Say which of the three usual causes it is, and
                # put it in the event log as well as the dialog, because the
                # dialog disappears and the log does not.
                why = describe_port_error(port, exc)
                self._log(f"connect failed: {why}")
                self.ctl.note(f"connect failed: {why}")
                messagebox.showerror("Connect failed", why)
                return
            self.btn_conn.config(text="Disconnect", bg=DANGER)
        self.lbl_link.config(text=self.ctl.ui.link_name)

    def _enable_purge(self):
        """Tick box <-> the two places the answer actually matters.

        `ui.purge_enabled` is the GUI-side gate (Controller.set_prg refuses
        PURGE while it is false); `cfg.allow_purge` is the field sent to the pad
        in a CFG frame.  Both are updated here so they cannot drift; the pad is
        told on the next "Apply to pad", exactly like the other configuration
        fields, and the note says so.
        """
        on = bool(self.v_purge.get())
        self.ctl.ui.purge_enabled = on
        self.ctl.cfg.allow_purge = 1 if on else 0
        self.ctl.note(f"purge valve {'enabled' if on else 'disabled'}"
                      " (press 'Apply to pad' to send)")

    def _toggle_arm(self):
        self.ctl.set_arm(not self.ctl.ui.armed)
        self.btn_arm.config(bg=OK if self.ctl.ui.armed else GREY)

    def _ign(self, pressed: bool):
        if pressed and not self.ctl.ui.armed:
            self.ctl.note("igniter pressed but ARM is off - inhibited")
            return
        if pressed:
            self.ctl.press_ign()
        else:
            self.ctl.release_ign()

    def _apply_cfg(self):
        self.ctl.apply_cfg(travel_fill=self.v_tf.get(), travel_veh=self.v_tv.get(),
                           travel_prg=self.v_tp.get(), wdg_ds=self.v_wd.get(),
                           ign_max_ds=self.v_ig.get(),
                           allow_purge=1 if self.v_purge.get() else 0)

    def _panic(self):
        self.ctl.release_ign()
        self.ctl.set_arm(False)
        self.ctl.note("ESC: igniter released, disarmed")

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

        st = self.ctl.pad
        age = self.ctl.pad_age_ms()

        def setv(key, text, colour=FG):
            self.tel[key].config(text=text, fg=colour)

        if st is None:
            for k in self.tel:
                setv(k, "-", DIM)
        else:
            names = {P.PAD_BOOT: "BOOT", P.PAD_WAIT_HELLO: "WAIT HELLO",
                     P.PAD_RUN: "RUN", P.PAD_LATCHED: "LATCHED/FAULT"}
            setv("pad_state", names.get(st.pad_state, str(st.pad_state)))
            setv("age", f"{st.age_ms if st.age_ms != 0xFFFF else -1} ms",
                 OK if st.age_ms != 0xFFFF and st.age_ms < st.wdg_ds * 100 else DANGER)
            setv("link", "OK" if st.link_ok else "LOST",
                 OK if st.link_ok else DANGER)
            setv("armed", "ARMED" if st.armed else "inhibited",
                 OK if st.armed else DIM)
            setv("ign", "ENERGISED" if st.ign_on else "off",
                 PURPLE if st.ign_on else DIM)
            setv("fills", "OPEN" if st.out_fill else "CLOSED",
                 WARN if st.out_fill else FG)
            setv("vehs", "OPEN" if st.out_veh else "CLOSED",
                 WARN if st.out_veh else FG)
            setv("prgs", "OPEN" if st.out_prg else "CLOSED",
                 WARN if st.out_prg else FG)
            setv("rx", str(st.rx_count))
            setv("crc", str(st.crc_err), DANGER if st.crc_err else FG)
            setv("fw", f"{st.fw_major}.{st.fw_minor}")
            setv("uptime", f"{st.uptime_s} s")

        # igniter button enabled only when armed and link fresh
        can_fire = self.ctl.ui.armed and st is not None and age < 1500
        self.btn_ign.config(bg=DANGER if self.ctl.ui.ign_held else
                            ("#5a1a1a" if can_fire else GREY),
                            state="normal" if can_fire or self.ctl.ui.ign_held
                            else "disabled")

        self.lbl_link.config(
            text=f"{self.ctl.ui.link_name}  |  {self.ctl.link.stats.as_text()}"
            if self.ctl.link else "disconnected")

        self._sync_toggles()
        while self.ctl.ui.log_lines:
            self._log(self.ctl.ui.log_lines.pop(0))
        for e in self.ctl.events[-4:]:
            pass

    def _sync_toggles(self):
        for btn, on, label in ((self.b_fill, self.ctl.ui.fill, "FILL VALVE   [SwB / Ch5]"),
                               (self.b_veh, self.ctl.ui.veh, "VEHICLE VALVES   [SwD / Ch6]"),
                               (self.b_prg, self.ctl.ui.prg, "PURGE VALVE   [SwA / Ch8]")):
            txt = f"{label}:  {'OPEN' if on else 'CLOSED'}"
            if btn.cget("text") != txt:
                btn.config(text=txt, bg="#12351f" if on else "#0d1116")
        if self.btn_arm.cget("bg") != (OK if self.ctl.ui.armed else GREY):
            self.btn_arm.config(bg=OK if self.ctl.ui.armed else GREY)

    def _log(self, line: str):
        self.txt.insert("end", line + "\n")
        self.txt.see("end")
        if int(self.txt.index("end-1c").split(".")[0]) > 400:
            self.txt.delete("1.0", "50.0")


def main(argv=None):
    ap = argparse.ArgumentParser(description="HCR-5100 GSE laptop control")
    ap.add_argument("--port", help="serial port of the TX node (e.g. COM7, /dev/ttyACM0)")
    ap.add_argument("--sim", action="store_true", help="use the built-in pad simulator")
    ap.add_argument("--log-dir", default="logs", help="CSV log directory (default ./logs)")
    args = ap.parse_args(argv)

    ctl = Controller(port="SIM" if args.sim else args.port, log_dir=args.log_dir)
    app = App(ctl, sim=args.sim)
    if args.sim:
        ctl.connect("SIM")
        app.btn_conn.config(text="Disconnect", bg=DANGER)
        app.lbl_link.config(text=ctl.ui.link_name)
    app.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
