"""
controller.py  --  all GUI logic, no widgets.  Headless-testable.

The operator never drives a valve directly any more: every button and key goes
through `act()`, i.e. through the Supervisor (supervisor.py), which holds the
GSE `main` control rules -- LIVE/TEST modes, the fill timer, FILL/VEHICLE
interlocks, the ignition countdown with its T-0 HOLD, E-STOP and link-loss
latches.  The controller's job is plumbing:

    * feed the supervisor the pad's read-back and the link state every tick;
    * stream the supervisor's command to the pad at 20 Hz, and immediately on
      any change, so the pad watchdog stays fed only while this program runs;
    * keep the pad's configuration (travel, watchdog, igniter limit, purge
      allowed) re-sent until the pad reports it in force;
    * on close, command ALL SAFE, then push three SAFE-set frames (FILL/PURGE
      closed, VEHICLE open, IGN off) before the port is closed.

The pad still enforces its own failsafes underneath (1 s watchdog, igniter
needs the arm code in every frame, one shot per press, hard on-time limit).
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass, field

from . import protocol as P
from .link import Link, make_link
from .supervisor import Settings, Supervisor, PURGE_ENABLE, PURGE_DISABLE

CMD_HZ = 20
STATE_STALE_MS = 1500       # GUI declares the pad "unknown" after this
CFG_RESEND_S = 1.0          # pad_link.rs:57

LEVEL_PREFIX = {"info": "", "ok": "", "warn": "WARN  ", "error": "ERROR "}


@dataclass
class UiState:
    link_name: str = "disconnected"
    connected: bool = False
    log_lines: list = field(default_factory=list)


class Controller:
    def __init__(self, port: str | None = None, log_dir: str | None = None,
                 settings_path: str | None = None):
        self.ui = UiState()
        path = None
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
            path = os.path.join(log_dir, time.strftime("gse_%Y%m%d_%H%M%S.csv"))
        self.link: Link | None = None
        self.port = port
        self.log_path = path
        self.settings_path = settings_path
        self.sup = Supervisor(Settings.load(settings_path), log=self._sup_log)
        self.cfg = P.Cfg()
        self.cfg.allow_purge = 0            # purge is enabled per session only
        self.cfg_seq = 0
        self.last_tick = 0.0
        self.last_cmd_sent = 0.0
        self.last_cfg_sent = 0.0
        self._last_sent_cmd = None
        self.events: list[str] = []
        self._ping_at: float | None = None   # when the last PING went out
        # Index into link.node_log: the node's own "LOG ..." lines are the only
        # place the pad's boot banner, its link/transport and its counters
        # appear.  They used to be collected and never shown.
        self._log_seen = 0

    # ------------------------------------------------------------- lifecycle
    def connect(self, port: str | None = None) -> None:
        self.disconnect()
        if port is not None:
            self.port = port
        self.link = make_link(self.port, self.log_path)
        self.link.open()
        self.ui.connected = True
        self.ui.link_name = self.link.name
        self._log_seen = 0
        self.sup.reset_session()
        self.sup.connected = True
        self.note(f"connected: {self.link.name} (log {self.log_path or 'off'})")
        # SerialLink.open() just sent PING; time it so the PONG shows a round trip
        self._ping_at = time.monotonic() if self.link.name == "serial" else None
        self.link.send_hello()
        self.link.send_cfg(self._next_cfg())
        self.last_cfg_sent = time.monotonic()
        self.push_cmd(force=True)

    def disconnect(self) -> None:
        if self.link:
            self.sup.shutdown()
            self.push_cmd(force=True)       # ALL SAFE, then the SAFE set
            self.safe_close()
            self.link.close()
            self.link = None
            self.note("Pad left in the SAFE set (FILL/PURGE closed, VEHICLE open, IGN off)")
        self.sup.connected = False
        self.sup.reset_session()
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
    def act(self, action: str) -> bool:
        """Every operator button / key ends here (see supervisor.ALL_ACTIONS)."""
        done = self.sup.act(action)
        if done and action in (PURGE_ENABLE, PURGE_DISABLE):
            self.cfg.allow_purge = 1 if self.sup.purge_enabled else 0
            self.last_cfg_sent = 0.0        # resend now (pad_link.rs:289-292)
        self.push_cmd(force=True)
        return done

    def console(self, text: str) -> bool:
        """Send one text line to the ground board's console (gse_tx_core.c:
        PING, INFO, SAFE, HELP, SET LEVEL n, SET KEY n, RADIO CONFIG).  The
        reply comes back as a "NODE ..." line in the event log."""
        cmd = " ".join(text.strip().upper().split())
        if not cmd:
            return False
        if not self.link:
            self.note(f"console: not connected - '{cmd}' not sent")
            return False
        if not all(0x20 <= ord(ch) < 0x7F for ch in cmd) or len(cmd) > 60:
            self.note("console: printable ASCII only, at most 60 characters")
            return False
        self.link.text_command(cmd)
        self.note(f"> {cmd}")
        if cmd == "PING":
            self._ping_at = time.monotonic()
        return True

    def gates(self):
        return self.sup.gates()

    def set_mode(self, mode: str) -> bool:
        ok = self.sup.set_mode(mode)
        if ok:
            self.sup.settings.save(self.settings_path)
        return ok

    def apply_settings(self, fill_seconds: int, countdown_seconds: int,
                       confirm_ign: bool) -> None:
        s = self.sup.settings
        s.fill_seconds, s.countdown_seconds, s.confirm_ign = (
            fill_seconds, countdown_seconds, confirm_ign)
        s.sanitize()
        s.save(self.settings_path)
        self.note(f"Settings updated: fill time set to {s.fill_seconds} s, countdown time "
                  f"set to {s.countdown_seconds} s, ignition confirmation "
                  f"{'enabled' if s.confirm_ign else 'disabled'}")

    def apply_cfg(self, **kw):
        for k, v in kw.items():
            if hasattr(self.cfg, k) and k != "allow_purge":
                setattr(self.cfg, k, v)
        if self.link:
            self.link.send_cfg(self._next_cfg())
            self.last_cfg_sent = time.monotonic()
        self.note(f"CFG applied: travel {self.cfg.travel_fill}/{self.cfg.travel_veh}/"
                  f"{self.cfg.travel_prg}, wdg {self.cfg.wdg_ds} ds, "
                  f"ign {self.cfg.ign_max_ds} ds, purge {self.cfg.allow_purge}")

    def _next_cfg(self) -> P.Cfg:
        self.cfg_seq = (self.cfg_seq + 1) & 0xFF
        self.cfg.cfg_seq = self.cfg_seq
        return self.cfg

    def cfg_synced(self) -> bool:
        """The pad reports the configuration we want (cfg_seq aside)."""
        st = self.pad
        if st is None:
            return False
        c = self.cfg
        return (st.travel_fill, st.travel_veh, st.travel_prg, st.wdg_ds,
                st.ign_max_ds, int(bool(st.allow_purge))) == (
                c.travel_fill, c.travel_veh, c.travel_prg, c.wdg_ds,
                c.ign_max_ds, int(bool(c.allow_purge)))

    # ---------------------------------------------------------------- the loop
    def arm_code(self) -> int:
        return P.ARM_CODE if self.sup.command().arm else 0

    def push_cmd(self, force: bool = False) -> None:
        if not self.link:
            return
        now = time.monotonic()
        c = self.sup.command()
        if c != self._last_sent_cmd:
            force = True                    # a change goes out immediately
        if not force and (now - self.last_cmd_sent) < (1.0 / CMD_HZ):
            return
        self.last_cmd_sent = now
        self._last_sent_cmd = c
        self.link.send_cmd(P.Cmd(
            arm_code=P.ARM_CODE if c.arm else 0,
            fill=int(c.fill), veh=int(c.veh), prg=int(c.prg), ign=int(c.ign),
            wdg_ds=self.cfg.wdg_ds, ign_max_ds=self.cfg.ign_max_ds,
            tx_ms=0, ign_wdg=0))

    def tick(self) -> list[str]:
        """Call at >= 20 Hz from the GUI event loop."""
        msgs: list[str] = []
        if not self.link:
            self.sup.tick(None, 0, float("inf"), connected=False)
            return msgs
        for ev in self.link.poll():
            self.events.append(f"{time.strftime('%H:%M:%S')} {ev.name}"
                               + (f" arg={ev.arg}" if ev.arg else ""))
            msgs.append(self.events[-1])
        # The nodes' own text lines: "TX ... ready", "link: espnow -- ...",
        # "PAD LINK OK", "RADIO FAULT: ...", the 1 Hz counters and the pad's
        # "nothing is arriving" hint.  Show them, do not just keep them.
        for line in self.drain_node_log():
            self.note(f"NODE {line}")
            if line.startswith("PONG") and self._ping_at is not None:
                rtt = (time.monotonic() - self._ping_at) * 1000
                self._ping_at = None
                parts = line.split()
                up = (f", board up {int(parts[1]) / 1000:.1f} s"
                      if len(parts) > 1 and parts[1].isdigit() else "")
                self.note(f"Ground board answered PING in {rtt:.0f} ms{up}")
        self.sup.tick(self.link.state, self.link.stats.rx_state,
                      self.link.age_ms() / 1000.0, connected=True)
        now = time.monotonic()
        if now - self.last_tick >= 1.0 / CMD_HZ:
            self.last_tick = now
            self.push_cmd()
        else:
            self.push_cmd()                 # sends only if the command changed
        if (self.pad is not None and not self.cfg_synced()
                and now - self.last_cfg_sent >= CFG_RESEND_S):
            self.last_cfg_sent = now
            self.link.send_cfg(self._next_cfg())
        return msgs

    def drain_node_log(self) -> list[str]:
        """Return the node log lines that have arrived since the last call.

        link.node_log is trimmed to the last 40 lines, so the cursor is
        re-anchored when it would fall off the front rather than silently
        skipping lines.
        """
        lines = getattr(self.link, "node_log", None) if self.link else None
        if not lines:
            self._log_seen = 0
            return []
        if self._log_seen > len(lines):
            self._log_seen = 0
        new = list(lines[self._log_seen:])
        self._log_seen = len(lines)
        return new

    def note(self, text: str) -> None:
        line = f"{time.strftime('%H:%M:%S')} {text}"
        self.events.append(line)
        self.ui.log_lines.append(line)
        csv = getattr(self.link, "log", None)
        if csv is not None:
            csv.row("app", "LOG", note=text)

    def _sup_log(self, level: str, text: str) -> None:
        self.note(LEVEL_PREFIX.get(level, "") + text)

    # -------------------------------------------------------------- readback
    @property
    def pad(self) -> P.State | None:
        return self.link.state if self.link else None

    def pad_age_ms(self) -> float:
        return self.link.age_ms() if self.link else float("inf")

    def _tx_transport(self) -> str:
        """The transport the TX node says it is using: 'lora', 'espnow' or ''.

        The TX node prints one line at boot:

            LOG link: espnow -- ESP-NOW peer 24:6f:28:aa:bb:01, channel 1 ...

        Only the TX node's own lines can appear here: the GUI's serial port is
        the TX node, and the pad's text log goes out the *pad's* USB port (over
        the air the nodes exchange binary frames only -- by design, so that text
        can never be mistaken for, or delay, a command).  So this can say what
        the TX side has, never what the pad side has; spotting a mismatched
        pair still needs the pad's boot banner, which is why
        docs/TROUBLESHOOTING_LINK.md §0 is a one-line check on both boards.
        """
        for line in reversed(getattr(self.link, "node_log", []) or []):
            low = line.lower()
            if low.startswith("link:") or " radio wiring:" in low:
                return "espnow" if "espnow" in low else "lora"
        return ""

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
                if "RADIO FAULT" in line or "LINK FAULT" in line:
                    source = "PAD" if line.startswith("PAD ") else "TX"
                    detail = line.split(':', 1)[-1].strip() if ':' in line else line
                    return "red", f"{source} LINK FAULT - {detail}"
                if line.startswith("PAD ") and ("RADIO OK" in line or "LINK OK" in line):
                    return "red", "NO PAD TELEMETRY - PAD LINK CONFIGURED, NO DATA"

            # Name the transport: "the pad is on the other link type" is one
            # of the two ways to get here with two healthy-looking nodes (the
            # other is a dead radio), and the TX node's own banner is the only
            # half of that comparison the laptop can see.
            tx_t = self._tx_transport()
            if tx_t:
                return ("red", f"NO PAD TELEMETRY - TX on {tx_t}, pad never answered")
            return "red", "NO PAD TELEMETRY"
        if self.pad_age_ms() > STATE_STALE_MS:
            return "red", "PAD SILENT - WATCHDOG WILL SAFE THE SYSTEM"
        if st.ign_on:
            return "purple", "IGNITER ENERGISED"
        if st.flags & P.ST_COMMS_LOSS:
            return "red", "COMMS LOSS - PAD SAFE"
        if self.sup.mismatch:
            return "red", "PAD READBACK MISMATCH"
        if self.sup.latched_safe():
            return "amber", "SAFE SET LATCHED - VEHICLE VENTED - RESET CONTROLS TO CLEAR"
        if st.armed:
            return "green", "LINK OK - ARMED"
        return "amber", "LINK OK - NOT ARMED"
