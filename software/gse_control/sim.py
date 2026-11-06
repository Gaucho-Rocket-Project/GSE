"""
sim.py  --  high-fidelity model of the pad node, for rehearsal and CI

It implements the same state machine as firmware/src/pad/pad_node.cpp:

    * boots safe (FILL CLOSED, PURGE CLOSED, VEHICLE OPEN, IGN OFF)
    * accepts CMD / CFG / HELLO frames with the same validation
    * watchdog -> comms loss -> safe outputs (protocol.failsafe)
    * igniter hard on-time limit
    * emits STATE at 10 Hz and EVENT frames on edges

The GUI and the tests use it through link.SimLink, so every UI path can be
exercised with no radio and no hardware.
"""
from __future__ import annotations

import time

from . import protocol as P


class PadModel:
    def __init__(self):
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self):
        self.parser = P.Parser()
        self.cfg = P.Cfg()
        self.last_cmd: P.Cmd | None = None
        self.last_cmd_mono: float | None = None
        self.rx_count = 0
        self.crc_err = 0
        self.pad_state = P.PAD_WAIT_HELLO
        self.t0 = time.monotonic()
        self.last_state_tx = 0.0
        self.out = P.Outputs()
        self.ign_on_since: float | None = None
        self.ign_spent = False
        self.last_loss = True
        self.queue = bytearray()
        self.seq = 0
        self.events_log: list[tuple[float, P.Event]] = []

    def _emit(self, ftype: int, payload: bytes):
        self.seq = (self.seq + 1) & 0xFF
        self.queue += P.encode(ftype, self.seq, payload)

    def _event(self, code: int, arg: int = 0):
        self._emit(P.T_EVENT, bytes([code, arg, 0]))
        self.events_log.append((time.monotonic(), P.Event(code, arg)))

    # ------------------------------------------------------------------- input
    def feed(self, data: bytes):
        now = time.monotonic()
        for ftype, _seq, obj in self.parser.push(data):
            if ftype == P.T_CMD:
                self.last_cmd = obj
                self.last_cmd_mono = now
                self.rx_count += 1
                if self.pad_state == P.PAD_WAIT_HELLO:
                    self.pad_state = P.PAD_RUN
                if obj.wdg_ds:
                    self.cfg.wdg_ds = P.clamp(obj.wdg_ds, P.WDG_DS_MIN, P.WDG_DS_MAX)
                if obj.ign_max_ds:
                    self.cfg.ign_max_ds = P.clamp(obj.ign_max_ds, P.IGN_DS_MIN,
                                                  P.IGN_DS_MAX)
            elif ftype == P.T_CFG and isinstance(obj, P.Cfg):
                self.cfg = obj
                self.cfg.wdg_ds = P.clamp(obj.wdg_ds or P.DEF_WDG_DS,
                                          P.WDG_DS_MIN, P.WDG_DS_MAX)
                self.cfg.ign_max_ds = P.clamp(obj.ign_max_ds or P.DEF_IGN_MAX_DS,
                                              P.IGN_DS_MIN, P.IGN_DS_MAX)
                self._event(P.EV_CFG_CHANGED, obj.cfg_seq)
            elif ftype == P.T_HELLO:
                if self.pad_state == P.PAD_WAIT_HELLO:
                    self.pad_state = P.PAD_RUN
                self._emit(P.T_HELLO, bytes([P.NODE_PAD, 1, 0]))
        self.crc_err = self.parser.frames_bad

    # ------------------------------------------------------------------ output
    def tick(self) -> bytes:
        now = time.monotonic()
        age = (0xFFFF if self.last_cmd_mono is None
               else min(int((now - self.last_cmd_mono) * 1000), 0xFFFE))
        self.out = P.failsafe(self.last_cmd, age, self.cfg.wdg_ds,
                              self.cfg.ign_max_ds)

        # igniter sequencing: one-shot per press + hard on-time limit
        req = bool(self.out.ign)
        if not req:
            self.ign_spent = False
            if self.ign_on_since is not None:
                self._event(P.EV_IGN_OFF, 0)
                self.ign_on_since = None
            self.out.ign = 0
        elif self.ign_spent:
            self.out.ign = 0
        else:
            if self.ign_on_since is None:
                self.ign_on_since = now
                self._event(P.EV_IGN_ON, 0)
            elif (now - self.ign_on_since) * 1000 > self.cfg.ign_max_ds * 100:
                self._event(P.EV_IGN_OFF, 1)
                self.ign_on_since = None
                self.ign_spent = True
                self.out.ign = 0
            else:
                self.out.ign = 1

        if self.out.comms_loss != self.last_loss:
            self._event(P.EV_COMMS_LOSS if self.out.comms_loss else P.EV_HELLO_OK, age & 0xFF)
            self.last_loss = self.out.comms_loss

        if now - self.last_state_tx >= 0.1:
            self.last_state_tx = now
            s = P.State(
                flags=(0 if self.out.comms_loss else P.ST_LINK_OK)
                      | (0 if self.last_cmd else P.ST_LATCHED)
                      | (P.ST_ARMED if self.out.armed else 0)
                      | (P.ST_IGN_ON if self.out.ign else 0)
                      | (P.ST_COMMS_LOSS if self.out.comms_loss else 0)
                      | (P.ST_HELLO_OK if self.pad_state == P.PAD_RUN else 0),
                out_fill=self.out.fill, out_veh=self.out.veh, out_prg=self.out.prg,
                out_ign=self.out.ign, ign_src=self.out.ign_src,
                age_ms=age, rx_count=self.rx_count & 0xFFFF,
                crc_err=self.crc_err & 0xFFFF, rssi_dbm=0x7F,
                pad_state=self.pad_state,
                uptime_s=int(now - self.t0) & 0xFFFF,
                wdg_ds=self.cfg.wdg_ds, ign_max_ds=self.cfg.ign_max_ds,
                fw_major=1, fw_minor=0,
                travel_fill=self.cfg.travel_fill, travel_veh=self.cfg.travel_veh,
                travel_prg=self.cfg.travel_prg,
                allow_purge=1 if self.cfg.allow_purge else 0)
            self._emit(P.T_STATE, s.pack())

        out = bytes(self.queue)
        self.queue.clear()
        return out
