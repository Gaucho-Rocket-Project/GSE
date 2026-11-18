"""
test_gui_config.py -- the configuration panel must not lie to the operator

Regression test for a real field failure:

    File "software/gse_control/app.py", line 231, in _apply_cfg
        allow_purge=1 if self.chk_purge.get() else 0)
    AttributeError: 'Checkbutton' object has no attribute 'get'

A tk.Checkbutton is a widget, not a variable: `cget()` reads widget options,
there is no `.get()`.  The state belongs in a BooleanVar.  This file drives the
real App callbacks headlessly and checks the two things that mattered:

  1. "Apply to pad" works at all (it used to raise, so *no* configuration was
     ever sent, and the operator saw a traceback instead of a CFG frame);
  2. the "purge valve fitted" tick, the GUI gate that blocks PURGE, and the
     allow_purge field sent to the pad never disagree -- showing a tick and
     then refusing the command is how an operator loses trust in a range tool.

How it runs without a display (or without tkinter at all): a tiny stand-in
`tkinter` module registers itself in sys.modules and every widget it creates
records its options.  That exercises the real App code path -- widget
construction, the callbacks, the variable reads -- without needing X11.  If a
real tkinter and a display are available the test uses them instead.
"""
from __future__ import annotations

import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from gse_control import protocol as P                 # noqa: E402
from gse_control.controller import Controller         # noqa: E402


# --------------------------------------------------------------------- fake tk
class _Var:
    def __init__(self, value=None, **kw):
        self._v = value if value is not None else kw.get("value")

    def get(self):
        return self._v

    def set(self, v):
        self._v = v


class _Widget:
    """Records options and children; enough for App._build() and the callbacks."""

    def __init__(self, master=None, **kw):
        self.master = master
        self.opts = dict(kw)
        self.children = []
        self._packed = False

    # geometry managers -----------------------------------------------------
    def pack(self, *a, **kw):
        self._packed = True

    def grid(self, *a, **kw):
        self._packed = True

    def place(self, *a, **kw):
        self._packed = True

    # option access --------------------------------------------------------
    def config(self, **kw):
        self.opts.update(kw)

    configure = config

    def cget(self, key):
        return self.opts.get(key, "")

    def __getitem__(self, key):
        if key == "values":
            return self.opts.get("values", [])
        return self.opts[key]

    def __setitem__(self, key, value):
        if key == "values":
            self.opts["values"] = value
        else:
            self.opts[key] = value

    # behaviour that App actually uses -------------------------------------
    def bind_all(self, *a, **kw): pass
    def winfo_class(self): return type(self).__name__
    def pack_propagate(self, *a, **kw): pass
    def grid_propagate(self, *a, **kw): pass
    def bind(self, *a, **kw): pass
    def protocol(self, *a, **kw): pass
    def title(self, *a, **kw): pass
    def geometry(self, *a, **kw): pass
    def after(self, ms, fn=None, *a):             # no timers in a unit test
        return "after#1"
    def after_cancel(self, *a, **kw): pass
    def destroy(self): pass
    def mainloop(self, *a, **kw): pass
    def insert(self, *a, **kw): pass
    def see(self, *a, **kw): pass
    def delete(self, *a, **kw): pass
    def index(self, *a, **kw): return "1.0"
    def focus_set(self): pass
    def winfo_children(self): return list(self.children)
    def tk_focusNext(self): return None


class _Text(_Widget):
    pass


def _install_fake_tk():
    """Register a stand-in 'tkinter' before app.py imports it."""
    tk = types.ModuleType("tkinter")

    class Tk(_Widget):
        pass

    for name in ("Frame", "Label", "Button", "Checkbutton", "Entry", "Canvas",
                 "Scale", "Spinbox", "Text", "Toplevel"):
        setattr(tk, name, type(name, (_Text if name == "Text" else _Widget,), {}))
    tk.Tk = Tk
    tk.BooleanVar = _Var
    tk.IntVar = _Var
    tk.DoubleVar = _Var
    tk.StringVar = _Var
    tk.W = "w"; tk.E = "e"; tk.N = "n"; tk.S = "s"
    tk.LEFT = "left"; tk.RIGHT = "right"; tk.TOP = "top"; tk.BOTTOM = "bottom"
    tk.X = "x"; tk.Y = "y"; tk.BOTH = "both"
    tk.END = "end"; tk.NORMAL = "normal"; tk.DISABLED = "disabled"
    tk.HORIZONTAL = "horizontal"; tk.VERTICAL = "vertical"

    ttk = types.ModuleType("tkinter.ttk")
    ttk.Combobox = type("Combobox", (_Widget,), {})
    ttk.Notebook = type("Notebook", (_Widget,), {})
    ttk.Separator = type("Separator", (_Widget,), {})
    ttk.Progressbar = type("Progressbar", (_Widget,), {})

    mb = types.ModuleType("tkinter.messagebox")
    mb.showinfo = lambda *a, **k: None
    mb.showwarning = lambda *a, **k: None
    mb.showerror = lambda *a, **k: None
    mb.askyesno = lambda *a, **k: True

    tk.ttk = ttk
    tk.messagebox = mb
    sys.modules["tkinter"] = tk
    sys.modules["tkinter.ttk"] = ttk
    sys.modules["tkinter.messagebox"] = mb


def _real_tk_available() -> bool:
    try:
        import tkinter  # noqa: F401
    except Exception:                                   # noqa: BLE001
        return False
    try:
        r = tkinter.Tk()                                # needs a display
        r.destroy()
        return True
    except Exception:                                   # noqa: BLE001
        return False


if not _real_tk_available():
    for mod in [m for m in sys.modules if m.startswith("tkinter")]:
        del sys.modules[mod]
    _install_fake_tk()

from gse_control.app import App                        # noqa: E402


# ------------------------------------------------------------------ the tests
def _app(purge: bool = False):
    ctl = Controller(port="SIM")
    ctl.sup.purge_enabled = purge
    ctl.cfg.allow_purge = 1 if purge else 0
    return ctl, App(ctl)


def test_apply_cfg_does_not_raise_and_sends_a_cfg():
    """The reported traceback: Apply used to die before sending anything."""
    ctl, app = _app(purge=False)
    sent = []
    ctl.apply_cfg = lambda **kw: sent.append(kw)        # capture, no link open
    app.v_tf.set(30); app.v_tv.set(40); app.v_tp.set(50)
    app.v_wd.set(12); app.v_ig.set(6)
    app._apply_cfg()
    assert sent, "Apply to pad sent nothing - the callback is still broken"
    kw = sent[0]
    assert (kw["travel_fill"], kw["travel_veh"], kw["travel_prg"]) == (30, 40, 50)
    assert kw["wdg_ds"] == 12 and kw["ign_max_ds"] == 6
    # allow_purge is driven only by the per-session purge enable, never by Apply
    assert "allow_purge" not in kw


def test_purge_tick_gate_and_cfg_field_agree():
    """A tick the GUI then refuses to act on is worse than no tick at all."""
    from gse_control import supervisor as S
    ctl, app = _app(purge=False)
    assert app.v_purge.get() is False
    assert ctl.sup.purge_enabled is False
    assert not ctl.gates()[S.PURGE_OPEN].enabled   # offline: "Ground board not connected"

    # tick it: the session gate opens, the cfg field follows
    app.v_purge.set(True)
    app._enable_purge()
    assert ctl.sup.purge_enabled is True
    assert ctl.cfg.allow_purge == 1

    # untick: the gate closes, the cfg field follows, and an open purge valve
    # is closed with it (pad_link.rs:269-274 -- "disabled" means forced closed)
    ctl.sup.purge_open = True
    app.v_purge.set(False)
    app._enable_purge()
    assert ctl.sup.purge_enabled is False
    assert ctl.cfg.allow_purge == 0
    assert ctl.sup.purge_open is False
    assert ctl.sup.command().prg is False


def test_purge_tick_starts_from_the_controller_not_a_constant():
    """A controller already told 'purge fitted' must not be un-told by a default."""
    ctl, app = _app(purge=True)
    assert app.v_purge.get() is True
    assert ctl.sup.purge_enabled is True
    assert ctl.cfg.allow_purge == 1


def test_window_paints_and_keys_drive_the_controls():
    """The whole window paints against the simulated pad, and main's keys work:
    F toggles FILL, Shift+R resets, E is E-STOP."""
    import time
    from gse_control import supervisor as S

    class Ev:
        def __init__(self, keysym, shift=False):
            self.keysym = keysym
            self.state = 0x1 if shift else 0
            self.widget = None

    ctl, app = _app()
    ctl.sup.settings.fill_seconds = 30
    ctl.connect("SIM")
    t0 = time.monotonic()
    while not ctl.sup.controls_ready and time.monotonic() - t0 < 2:
        ctl.tick(); time.sleep(0.01)
    assert ctl.sup.controls_ready
    app._paint()
    app._on_key(Ev("f"))
    assert ctl.sup.fill, "F did not open FILL"
    t0 = time.monotonic()
    while ctl.sup.pending is not None and time.monotonic() - t0 < 2:
        ctl.tick(); time.sleep(0.01)
    app._paint()
    app._on_key(Ev("e"))
    assert ctl.sup.estop and ctl.sup.command() == S.SAFE_SET
    app._paint()
    t0 = time.monotonic()
    while ctl.sup.pending is not None and time.monotonic() - t0 < 2:
        ctl.tick(); time.sleep(0.01)
    app._on_key(Ev("R", shift=True))
    assert not ctl.sup.estop, "Shift+R (RESET CONTROLS) did not clear E-STOP"
    app._paint()
    ctl.disconnect()


def test_apply_sends_allow_purge_matching_the_tick():
    ctl, app = _app(purge=False)
    frames = []

    class FakeLink:
        name = "fake"
        state = None

        def send_cfg(self, cfg):
            frames.append(cfg)

        def send_cmd(self, cmd):            # every action also pushes a CMD
            pass

    ctl.link = FakeLink()
    app.v_purge.set(True)
    app._enable_purge()
    app._apply_cfg()
    assert frames, "no CFG frame reached the link"
    assert frames[-1].allow_purge == 1

    app.v_purge.set(False)
    app._enable_purge()
    app._apply_cfg()
    assert frames[-1].allow_purge == 0


def test_pad_side_still_gates_purge_on_cfg():
    """The GUI fix must not have moved the interlock: the pad still decides.

    The pad model keeps the allow_purge bit in `pad.cfg.allow_purge` and refuses
    to drive PURGE while it is 0 -- the GUI gate is a convenience, not the
    safety boundary.
    """
    from gse_control.sim import PadModel
    pad = PadModel()
    pad.reset()

    def cfg_frame(allow: int, seq: int) -> bytes:
        return P.encode_cfg(seq, P.Cfg(travel_fill=45, travel_veh=45,
                                       travel_prg=45, wdg_ds=P.DEF_WDG_DS,
                                       ign_max_ds=P.DEF_IGN_MAX_DS,
                                       allow_purge=allow, cfg_seq=seq))

    pad.feed(cfg_frame(0, 1))
    pad.tick()
    assert pad.cfg.allow_purge == 0, "pad accepted PURGE it was not told about"

    pad.feed(cfg_frame(1, 2))
    pad.tick()
    assert pad.cfg.allow_purge == 1, "pad did not take the allow_purge=1 CFG"



# ------------------------------------------- connecting / diagnosing a silence
def test_connect_failure_message_is_actionable():
    from gse_control.app import describe_port_error
    for exc, want in [
        (PermissionError(13, "Permission denied"), "dialout"),
        (OSError(2, "could not open port /dev/ttyACM0: No such file"), "Refresh"),
        (OSError(16, "/dev/ttyUSB0: Device or resource busy"), "busy"),
    ]:
        text = describe_port_error("/dev/ttyACM0", exc)
        assert want in text, (want, text)
    # the port name must survive into the message
    assert "/dev/ttyACM0" in describe_port_error("/dev/ttyACM0", OSError(2, "x"))


def test_node_log_lines_reach_the_event_log():
    """The pad's own banner/link/counter lines used to be collected silently."""
    ctl = Controller(port="SIM")
    ctl.connect()
    ctl.link.node_log = ["PAD link: espnow -- ESP-NOW peer 24:6f:28:aa:bb:01, channel 1",
                         "PAD LINK OK  (waiting for the TX node)"]
    ctl.tick()
    assert any("PAD link: espnow" in l for l in ctl.events), ctl.events
    # and they are only shown once
    before = len(ctl.events)
    ctl.tick()
    assert len([l for l in ctl.events[before:] if "NODE" in l]) == 0
    ctl.disconnect()


class _FakeSerial:
    """Byte-stream stand-in for pyserial (in_waiting / read / write)."""

    def __init__(self, data: bytes = b""):
        self.rx = bytearray(data)
        self.written: list[bytes] = []

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, n):
        out = bytes(self.rx[:n])
        del self.rx[:n]
        return out

    def write(self, data):
        self.written.append(bytes(data))

    def close(self):
        pass


def test_serial_link_keeps_log_lines_mixed_with_frames():
    """The TX node interleaves binary frames and 'LOG ...' lines on one port.
    Both must survive: frames to the parser, LOG lines to node_log.  (The old
    reader handed every byte to the frame parser first, so PONG and the 1 Hz
    counters almost never reached the GUI.)"""
    from gse_control.link import SerialLink
    from gse_control.sim import PadModel
    frame = PadModel().tick()                      # one real STATE frame
    assert frame and frame[0] == P.SOF
    stream = (b"rst:0x1 (POWERON_RESET),boot:0x13\r\n"   # ROM noise: ignored
              + frame
              + b"LOG PONG 1234\n"
              + frame + b"LOG tx fwd=5 state=0 safe=12 dropped=0 radio=ok\r\n")
    link = SerialLink("COM_TEST")
    link._ser = _FakeSerial(stream)
    while link._ser.in_waiting:
        link.poll()
    assert link.state is not None, "the STATE frame was lost"
    assert link.stats.rx_state == 2
    assert link.node_log == ["PONG 1234",
                             "tx fwd=5 state=0 safe=12 dropped=0 radio=ok"], link.node_log


def test_console_command_starts_with_a_newline():
    """A leading newline makes a mis-framed first byte harmless on the TX node."""
    from gse_control.link import SerialLink
    link = SerialLink("COM_TEST")
    link._ser = _FakeSerial()
    link.text_command(" info ")
    assert link._ser.written[-1] == b"\nINFO\n"


def test_console_ping_reports_a_round_trip_in_sim():
    ctl = Controller(port="SIM")
    ctl.connect()
    assert ctl.console("ping")
    ctl.tick()
    assert any("NODE PONG" in l for l in ctl.events), ctl.events
    assert any("Ground board answered PING in" in l for l in ctl.events), ctl.events
    assert ctl.console("info")
    ctl.tick()
    assert any("NODE TX sim fw 1.0" in l and "fwd=" in l for l in ctl.events)
    assert not ctl.console("   ")
    ctl.disconnect()
    assert not ctl.console("PING"), "must refuse when not connected"


def test_tx_transport_is_named_in_the_banner():
    """A dead pad link should say which transport the laptop side is on.

    The GUI can only ever see the TX node's own lines (the pad's text log goes
    out the pad's USB port, not over the air), so this reports one half of the
    comparison and nothing more.
    """
    ctl = Controller(port="SIM")
    ctl.connect()
    ctl.link.state = None                      # no telemetry at all
    ctl.link.node_log = [
        "TX esp32s fw 1.0 ready",
        "link: espnow -- ESP-NOW peer 24:6f:28:aa:bb:01, channel 1, no module, no pairing",
    ]
    colour, text = ctl.verdict()
    assert colour == "red", (colour, text)
    assert "espnow" in text and "pad never answered" in text, text

    ctl.link.node_log = ["link: lora -- LoRa UART1 RX<-D16 TX->D17"]
    colour, text = ctl.verdict()
    assert "lora" in text, text

    # a pad-side line in the log (e.g. pasted by an operator) must not be
    # mistaken for the TX node's own transport
    ctl.link.node_log = ["PAD link: espnow -- ESP-NOW peer 24:6f:28:aa:bb:01"]
    colour, text = ctl.verdict()
    assert text == "NO PAD TELEMETRY", text
    ctl.disconnect()

if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"  ok  {fn.__name__}")
    print(f"ALL PASS ({len(fns)} tests)")
