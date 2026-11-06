"""
test_protocol.py  --  proves the host implementation matches the firmware.

Run:   python -m pytest -q software/tests/test_protocol.py
   or: python software/tests/test_protocol.py
"""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "software"))

from gse_control import protocol as P  # noqa: E402


def test_crc_vector():
    assert P.crc16(b"123456789") == 0x29B1


def test_cmd_golden():
    """Byte-identical to the frame the firmware emits (tests/genvec.c)."""
    c = P.Cmd(arm_code=0xA5, fill=1, veh=1, prg=0, ign=0, wdg_ds=12,
              ign_max_ds=6, flags=0, tx_ms=0xBEEF, ign_wdg=0)
    assert P.encode_cmd(7, c).hex().upper() == "7E0110070CA5010100000C0600EFBE000089E5"


def test_state_golden():
    s = P.State(flags=P.ST_LINK_OK | P.ST_HELLO_OK, out_fill=1, out_veh=1,
                out_prg=0, out_ign=0, ign_src=0, age_ms=123, rx_count=4096,
                crc_err=3, rssi_dbm=-84, pad_state=P.PAD_RUN, uptime_s=77,
                wdg_ds=10, ign_max_ds=5, fw_major=1, fw_minor=0,
                travel_fill=45, travel_veh=45, travel_prg=30, allow_purge=1)
    assert P.encode(P.T_STATE, 3, s.pack()).hex().upper() == (
        "7E011103182101010000007B0000100300AC024D000A0501002D2D1E01E387")


def test_cfg_golden():
    c = P.Cfg(travel_fill=45, travel_veh=50, travel_prg=30, wdg_ds=10,
              ign_max_ds=5, allow_purge=1, cfg_seq=9)
    assert P.encode_cfg(4, c).hex().upper() == "7E011304082D321E0A0501090087D7"


def test_event_golden():
    e = P.Event(P.EV_IGN_ON, 0)
    assert P.encode(P.T_EVENT, 6, bytes([e.code, e.arg, 0])).hex().upper() == (
        "7E01120603060000923C")


def test_parser_noisy_stream():
    p = P.Parser()
    got = []
    rng = 12345
    for f in range(300):
        c = P.Cmd(tx_ms=f, fill=f & 1, veh=1)
        frame = P.encode_cmd(f & 0xFF, c)
        rng = (rng * 1103515245 + 12345) & 0xFFFFFFFF
        for _ in range((rng >> 16) % 3):
            got += p.push(bytes([(rng >> 24) & 0xFF]))
        got += p.push(frame)
    cmds = [x for x in got if x[0] == P.T_CMD]
    assert len(cmds) == 300
    assert [x[2].tx_ms for x in cmds] == list(range(300))


def test_parser_rejects_corruption():
    good = P.encode_cmd(1, P.Cmd(tx_ms=1))
    bad = bytearray(good)
    bad[6] ^= 0x40
    p = P.Parser()
    assert p.push(bytes(bad)) == []
    assert p.frames_bad >= 1
    assert len(p.push(good + good)) == 2


def test_failsafe_parity():
    """Same matrix as tests/test_protocol.c :: test_failsafe_matrix"""
    cmd = P.Cmd(fill=1, prg=1, veh=0, ign=1)
    o = P.failsafe(None, 0xFFFF, 10, 5)
    assert o.comms_loss and not o.armed and (o.fill, o.veh, o.prg, o.ign) == (0, 1, 0, 0)

    o = P.failsafe(cmd, 100, 10, 5)                      # not armed
    assert not o.comms_loss and not o.armed and o.ign == 0
    assert (o.fill, o.veh, o.prg) == (1, 0, 1)

    cmd.arm_code = P.ARM_CODE
    o = P.failsafe(cmd, 100, 10, 5)                      # armed + fresh
    assert o.armed and o.ign == 1 and o.ign_src == P.IGN_SRC_CMD

    cmd.arm_code = 0
    o = P.failsafe(cmd, 100, 10, 5)                      # disarm -> inhibit
    assert o.ign == 0 and not o.armed

    cmd.arm_code = P.ARM_CODE
    assert not P.failsafe(cmd, 1000, 10, 5).comms_loss    # exactly at limit
    o = P.failsafe(cmd, 1001, 10, 5)                      # 1 ms late
    assert o.comms_loss and (o.fill, o.veh, o.prg, o.ign) == (0, 1, 0, 0)


def test_struct_sizes():
    assert len(P.Cmd().pack()) == 12
    assert len(P.State().pack()) == 24
    assert len(P.Cfg().pack()) == 8


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"  ok  {fn.__name__}")
    print(f"ALL PASS ({len(fns)} tests)")
