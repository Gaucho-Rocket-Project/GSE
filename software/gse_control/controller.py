"""
controller.py  --  all GUI logic, no widgets.  Headless-testable.

The controller owns the "what does the operator want" state and pushes a CMD
frame to the link at 20 Hz.  It never assumes the pad obeyed: everything the
operator sees comes from the pad's STATE frames.

Safety rules enforced here (defence in depth -- the pad enforces them again):
    * the CMD stream is always sent, even when "nothing is happening", so the
      pad watchdog stays fed only while this program is alive;
    * arm_code is only attached while the ARM toggle is on AND the GUI is able
      to see a fresh pad state (no blind arming over a dead link);
    * the igniter is a hold-to-fire control with a release that is sent
      immediately, not on the next 20 Hz tick;
    * on close, three safe frames are pushed before the port is closed.
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass, field

from . import protocol as P
from .link import Link, make_link

CMD_HZ = 20
STATE_STALE_MS = 1500       # GUI declares the pad "unknown" after this


@dataclass
class UiState:
    fill: bool = False
    veh: bool = False           # False == CLOSED; pad safe state is OPEN
    prg: bool = False
    ign_held: bool = False
    armed: bool = False
    purge_enabled: bool = False
    link_name: str = "disconnected"
    connected: bool = False
    log_lines: list = field(default_factory=list)


class Controller:
    def __init__(self, port: str | None = None, log_dir: str | None = None):
        self.ui = UiState()
        path = None
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
            path = os.path.join(log_dir, time.strftime("gse_%Y%m%d_%H%M%S.csv"))
        self.link: Link | None = None
        self.port = port
        self.log_path = path
        self.cfg = P.Cfg()
        self.cfg_seq = 0
        self.last_tick = 0.0
        self.last_cmd_sent = 0.0
        self.events: list[str] = []
        self._ign_release_pending = False

    # ------------------------------------------------------------- lifecycle
    def connect(self, port: str | None = None) -> None:
        self.disconnect()
        if port is not None:
            self.port = port
        self.link = make_link(self.port, self.log_path)
        self.link.open()
        self.ui.connected = True
        self.ui.link_name = self.link.name
        self.note(f"connected: {self.link.name} (log {self.log_path or 'off'})")
        self.link.send_hello()
        self.push_cmd(force=True)

    def disconnect(self) -> None:
        if self.link:
            self.safe_close()
            self.link.close()
            self.link = None
        self.ui.connected = False
        self.ui.link_name = "disconnected"

    def safe_close(self) -> None:
        """Called on quit: hand the pad a clean safe command."""
        if not self.link:
            return
        for _ in range(3):
            self.link.send_cmd(P.Cmd(fill=0, veh=1, prg=0, ign=0, arm_code=0,
                                     wdg_ds=self.cfg.wdg_ds,
                                     ign_max_ds=self.cfg.ign_max_ds))
            time.sleep(0.02)

    # ------------------------------------------------------------- operator
    def set_fill(self, on: bool):  self.ui.fill = bool(on);  self.push_cmd(force=True)
    def set_veh(self, on: bool):   self.ui.veh = bool(on);   self.push_cmd(force=True)
    def set_prg(self, on: bool):
        if on and not self.ui.purge_enabled:
            self.note("PURGE blocked: purge valve not enabled in configuration")
            return
        self.ui.prg = bool(on); self.push_cmd(force=True)

    def set_arm(self, on: bool):
        self.ui.armed = bool(on)
        if not on:
            self.ui.ign_held = False
        self.note("ARM on" if on else "ARM off")
        self.push_cmd(force=True)

    def press_ign(self):
        self.ui.ign_held = True
        self.push_cmd(force=True)

    def release_ign(self):
        self.ui.ign_held = False
        self.push_cmd(force=True)          # immediate, not next tick

    def apply_cfg(self, **kw):
        for k, v in kw.items():
            if hasattr(self.cfg, k):
                setattr(self.cfg, k, v)
        self.cfg.cfg_seq = (self.cfg_seq + 1) & 0xFF
        if self.link:
            self.link.send_cfg(self.cfg)
        self.note(f"CFG applied: travel {self.cfg.travel_fill}/{self.cfg.travel_veh}/"
                  f"{self.cfg.travel_prg}, wdg {self.cfg.wdg_ds} ds, "
                  f"ign {self.cfg.ign_max_ds} ds, purge {self.cfg.allow_purge}")

    # ---------------------------------------------------------------- the loop
    def arm_code(self) -> int:
        if not self.ui.armed:
            return 0
        if not self.link or not self.link.state:
            return 0                        # never arm blind
        if self.link.age_ms() > STATE_STALE_MS:
            return 0
        return P.ARM_CODE

    def push_cmd(self, force: bool = False) -> None:
        if not self.link:
            return
        now = time.monotonic()
        if not force and (now - self.last_cmd_sent) < (1.0 / CMD_HZ):
            return
        self.last_cmd_sent = now
        cmd = P.Cmd(
            arm_code=self.arm_code(),
            fill=1 if self.ui.fill else 0,
            veh=1 if self.ui.veh else 0,
            prg=1 if self.ui.prg else 0,
            ign=1 if (self.ui.ign_held and self.ui.armed) else 0,
            wdg_ds=self.cfg.wdg_ds,
            ign_max_ds=self.cfg.ign_max_ds,
            tx_ms=0, ign_wdg=0)
        self.link.send_cmd(cmd)

    def tick(self) -> list[str]:
        """Call at 20 Hz from the GUI event loop."""
        msgs: list[str] = []
        if not self.link:
            return msgs
        now = time.monotonic()
        if now - self.last_tick >= 1.0 / CMD_HZ:
            self.last_tick = now
            self.push_cmd()
        for ev in self.link.poll():
            self.events.append(f"{time.strftime('%H:%M:%S')} {ev.name}"
                               + (f" arg={ev.arg}" if ev.arg else ""))
            msgs.append(self.events[-1])
        st = self.link.state
        if st:
            if st.flags & P.ST_COMMS_LOSS:
                pass                        # pad already safe; banner shows it
        return msgs

    def note(self, text: str) -> None:
        line = f"{time.strftime('%H:%M:%S')} {text}"
        self.events.append(line)
        self.ui.log_lines.append(line)

    # -------------------------------------------------------------- readback
    @property
    def pad(self) -> P.State | None:
        return self.link.state if self.link else None

    def pad_age_ms(self) -> float:
        return self.link.age_ms() if self.link else float("inf")

    def verdict(self) -> tuple[str, str]:
        """(colour, text) for the big status banner."""
        if not self.link:
            return "grey", "DISCONNECTED"
        st = self.pad
        if st is None:
            # No STATE frame has ever arrived.  Say *why* when the node told us:
            # a pad printing RADIO FAULT will never transmit, and that is a very
            # different problem from a pad that is transmitting into a dead link.
            for line in reversed(getattr(self.link, "node_log", [])):
                if "RADIO FAULT" in line:
                    return "red", f"PAD RADIO FAULT - {line.split(':', 1)[-1].strip()}"
                if "RADIO OK" in line:
                    return "red", "NO PAD TELEMETRY - PAD RADIO OK, LINK DEAD"
            return "red", "NO PAD TELEMETRY"
        if self.pad_age_ms() > STATE_STALE_MS:
            return "red", "PAD SILENT - WATCHDOG WILL SAFE THE SYSTEM"
        if st.ign_on:
            return "purple", "IGNITER ENERGISED"
        if st.flags & P.ST_COMMS_LOSS:
            return "red", "COMMS LOSS - PAD SAFE"
        if st.armed:
            return "green", "LINK OK - ARMED"
        return "amber", "LINK OK - NOT ARMED"
