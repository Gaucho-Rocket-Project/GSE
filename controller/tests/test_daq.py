"""DAQ data in vehicle telemetry: parsing both line formats, the format toggle,
radio-loss vs newest-line skips, statuses, CSV, and the --sim boards."""
import csv
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import daq as D                                   # noqa: E402
from sim_ground import SimGround, SimSerial       # noqa: E402


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


D_LINE = "D,7,70000,12000,4000,,20000,21.50,-3.25,,nan,00,00,00,01"


class ParseTests(unittest.TestCase):
    def test_combined_line(self):
        f = D.parse_payload(D_LINE)
        self.assertIsInstance(f, D.CombinedFrame)
        self.assertEqual((f.seq, f.t_us), (7, 70000))
        self.assertEqual(f.ua, [12000, 4000, None, 20000])
        self.assertEqual(f.temp_c, [21.5, -3.25, None, None])
        self.assertEqual(f.fault, [0, 0, 0, 1])

    def test_separate_lines_and_heartbeats(self):
        p = D.parse_payload("P,1,10,4000,5000,6000,7000")
        t = D.parse_payload("T,1,10,20.0,21.0,,22.5,00,40,01,00")
        self.assertIsInstance(p, D.PressureFrame)
        self.assertEqual(p.ua, [4000, 5000, 6000, 7000])
        self.assertIsInstance(t, D.ThermoFrame)
        self.assertEqual(t.fault, [0, 0x40, 1, 0])
        self.assertEqual(D.parse_payload("HB,5").n, 5)
        self.assertIsNone(D.parse_payload("heartbeat").n)

    def test_not_daq_and_malformed(self):
        for payload in ("pressure,12.5", "hello", "", "Ready"):
            self.assertIsNone(D.parse_payload(payload), payload)
        for payload in ("D,1,2,3", "P,x,1,1,2,3,4", "T,1,2,1,2,3,4,00,00,00,zz", "HB,", "D," + "1," * 20):
            self.assertIs(D.parse_payload(payload), D.MALFORMED, payload)

    def test_conversion_and_faults(self):
        self.assertAlmostEqual(D.ua_to_psi(4000, 0, 1600), 0)
        self.assertAlmostEqual(D.ua_to_psi(20000, 0, 1600), 1600)
        self.assertAlmostEqual(D.ua_to_psi(12000, 0, 1000), 500)
        self.assertAlmostEqual(D.psi_to_ua(D.ua_to_psi(9876, 0, 750), 0, 750), 9876)
        self.assertEqual(D.fault_text(0x41), "TC range, open circuit")


class StateTests(unittest.TestCase):
    def test_format_toggle_selects_what_is_shown(self):
        st = D.DaqState(fmt="combined", clock=Clock())
        st.feed_tlm(1, 500, "P,1,10,12000,4000,4000,4000")
        self.assertIsNone(st.latest_p)
        self.assertEqual(st.other_format, 1)
        self.assertIn("switch the DAQ line format toggle", st.link_text()[0])
        st.feed_tlm(2, 1000, D_LINE)
        self.assertEqual(st.latest_p.ua[0], 12000)
        self.assertEqual(st.latest_t.temp_c[0], 21.5)
        st.set_format("separate")
        self.assertEqual(st.other_format, 0)
        st.feed_tlm(3, 1500, D_LINE)
        self.assertEqual(st.other_format, 1)
        st.feed_tlm(4, 2000, "T,9,10,30.0,31.0,32.0,33.0,00,00,00,00")
        self.assertEqual(st.latest_t.temp_c[0], 30.0)

    def test_radio_loss_vs_newest_line_skips(self):
        clk = Clock()
        st = D.DaqState(clock=clk)
        for tlm, seq in ((1, 50), (2, 100), (4, 200)):   # TLM 3 lost on the radio
            clk.t += 0.5
            st.feed_tlm(tlm, tlm * 500, f"D,{seq},0,4000,4000,4000,4000,20,20,20,20,00,00,00,00")
        self.assertEqual(st.tlm.gaps, 1)
        self.assertEqual(st.p.gaps, 49 + 99)               # DAQ seq 50 -> 100 -> 200
        text, level = st.link_text()
        self.assertIn("radio lost 1", text)
        self.assertEqual(level, "ok")

    def test_todays_daq_heartbeat_only_is_a_warning_not_a_failure(self):
        clk = Clock()
        st = D.DaqState(clock=clk)
        for n in range(1, 4):
            clk.t += 1.0
            st.feed_tlm(n, n * 1000, "heartbeat")
        text, level = st.link_text()
        self.assertEqual(level, "warn")
        self.assertIn("DAQ alive", text)
        self.assertIn("no DAQ data lines yet", text)
        self.assertEqual(st.pressure_status(0), ("no data", "none"))

    def test_non_daq_payloads_are_explained(self):
        st = D.DaqState(clock=Clock())
        st.feed_tlm(1, 0, "pressure,12.5")
        text, level = st.link_text()
        self.assertEqual(level, "warn")
        self.assertIn("not DAQ lines, e.g. 'pressure,12.5'", text)

    def test_duplicates_ignored_and_reboots_recovered(self):
        clk = Clock()
        st = D.DaqState(clock=clk)
        st.feed_tlm(5, 0, "D,80,0,12000,4000,4000,4000,20,20,20,20,00,00,00,00")
        st.feed_tlm(5, 0, "D,80,0,12000,4000,4000,4000,20,20,20,20,00,00,00,00")
        self.assertEqual((st.tlm.frames, st.p.frames, st.tlm.restarts), (1, 1, 0))
        clk.t += 0.5                                       # DAQ rebooted 8 s in: seq 80 -> 5
        st.feed_tlm(6, 0, "D,5,0,15000,4000,4000,4000,20,20,20,20,00,00,00,00")
        self.assertEqual(st.latest_p.ua[0], 15000, "data after a DAQ reboot must not be dropped")
        self.assertEqual(st.p.restarts, 1)
        clk.t += 0.5                                       # vehicle rebooted: TLM seq 6 -> 1
        st.feed_tlm(1, 0, "D,6,0,15100,4000,4000,4000,20,20,20,20,00,00,00,00")
        self.assertEqual((st.tlm.restarts, st.latest_p.ua[0]), (1, 15100))
        self.assertIn("restarts DAQ 1 / vehicle 1", st.link_text()[0])

    def test_status_flags_and_stale(self):
        clk = Clock()
        st = D.DaqState(clock=clk)
        self.assertEqual(st.pressure_status(0), ("no data", "none"))
        st.feed(D.CombinedFrame(1, 0, [3000, 21000, None, D.psi_to_ua(950, 0, 1000)],
                                [20.0, None, 22.0, None], [0, 0, 0, 1]))
        self.assertIn("BELOW 4 mA", st.pressure_status(0)[0])
        self.assertIn("ABOVE 20 mA", st.pressure_status(1)[0])
        self.assertEqual(st.pressure_status(2), ("READ ERROR", "bad"))
        self.assertEqual(st.pressure_status(3)[1], "warn")
        self.assertEqual(st.tc_status(0), ("ok", "ok"))
        self.assertEqual(st.tc_status(1), ("READ ERROR", "bad"))
        self.assertEqual(st.tc_status(3)[0], "FAULT 0x01: open circuit")
        clk.t += D.STALE_S + 0.1
        self.assertTrue(st.pressure_status(0)[0].startswith("STALE"))
        self.assertEqual(st.link_text()[1], "bad")

    def test_range_change_and_settings_round_trip(self):
        st = D.DaqState(clock=Clock())
        st.feed(D.CombinedFrame(1, 0, [12000] * 4, [20.0] * 4, [0] * 4))
        self.assertAlmostEqual(st.psi(0, 12000), 800)
        new = D.DaqSettings()
        new.pressure[0] = D.PressureChannel("Tank", 0, 1000)
        st.set_settings(new)
        self.assertAlmostEqual(st.psi(0, 12000), 500)
        back = D.DaqSettings.from_dict(new.to_dict())
        self.assertEqual(back.pressure[0].max_psi, 1000)
        bad = new.to_dict()
        bad["pressure"][1]["max_psi"] = -5
        self.assertEqual(D.DaqSettings.from_dict(bad).pressure[1].max_psi, 1000)
        self.assertEqual(D.DaqSettings.from_dict(None).validate(), [])

    def test_csv_rows(self):
        with tempfile.TemporaryDirectory() as tmp:
            st = D.DaqState(data_dir=Path(tmp), clock=Clock())
            st.feed_tlm(3, 1500, D_LINE)
            new = D.DaqSettings()
            new.pressure[0] = D.PressureChannel("Tank", 0, 1000)
            st.set_settings(new)
            st.feed_tlm(4, 2000, D_LINE.replace("D,7,", "D,8,"))
            st.feed_tlm(5, 2500, "heartbeat")             # not a CSV row
            path = st.csv_path
            st.close()
            with path.open(newline="", encoding="utf-8") as fh:
                rows = list(csv.DictReader(fh))
        self.assertEqual([r["daq_seq"] for r in rows], ["7", "8"])
        self.assertEqual(rows[0]["tlm_seq"], "3")
        self.assertEqual(rows[0]["kind"], "D")
        self.assertEqual(rows[0]["p0_psi"], "800.00")
        self.assertEqual(rows[1]["p0_psi"], "500.00")
        self.assertEqual(rows[1]["p0_range"], "0-1000")
        self.assertEqual(rows[0]["p2_ua"], "")
        self.assertEqual(rows[0]["tc4_fault"], "01")


    def test_csv_failure_is_reported_once_and_display_continues(self):
        with tempfile.TemporaryDirectory() as tmp:
            blocker = Path(tmp) / "not_a_dir"
            blocker.write_text("x")                        # a file where the folder should be
            st = D.DaqState(data_dir=blocker / "data", clock=Clock())
            with self.assertRaises(OSError):
                st.feed_tlm(1, 0, D_LINE)
            self.assertEqual(st.latest_p.ua[0], 12000, "frame still displayed")
            st.feed_tlm(2, 0, D_LINE.replace("D,7,", "D,8,"))   # no second exception
            self.assertEqual(st.latest_p.seq, 8)
            self.assertIn("DAQ CSV NOT BEING WRITTEN", st.link_text()[0])
            st.reset()
            self.assertEqual(st.csv_error, "", "a new connection retries")


class SimTests(unittest.TestCase):
    def test_sim_daq_rates_both_formats(self):
        clk = Clock(0.0)
        for fmt, expect in (("combined", {"D": 101}), ("separate", {"P": 101, "T": 11, "HB": 2})):
            sim = D.SimDaq(fmt=fmt, clock=Clock(0.0))
            sim.clock.t = 1.0005
            kinds = [l.split(",")[0] for l in sim.lines()]
            for k, n in expect.items():
                self.assertEqual(kinds.count(k), n, (fmt, k))
        del clk

    def test_ground_forwards_newest_line_every_500_ms(self):
        clk = Clock(0.0)
        g = SimGround(interval_ms=500, fmt="combined", clock=clk, seed=1)
        g.open()
        lines = []
        for _ in range(300):                               # 3 s in 10 ms steps
            clk.t += 0.01
            lines += g.read_lines()
        tlm = [l for l in lines if l.startswith("TLM ")]
        self.assertIn(len(tlm), (5, 6))
        self.assertTrue(lines[0].startswith("ESP-NOW servo/IGN ground"))
        st = D.DaqState(clock=clk)
        for l in tlm:
            _, seq, up, payload = l.split(" ", 3)
            st.feed_tlm(int(seq), int(up), payload)
        self.assertEqual(st.tlm.gaps, 0)
        self.assertGreater(st.p.gaps, 150)                 # 100 Hz DAQ, 2 Hz forwarded
        self.assertAlmostEqual(st.psi(0, st.latest_p.ua[0]), 650, delta=5)

    def test_ground_commands_acks_and_interlocks(self):
        clk = Clock(0.0)
        g = SimGround(clock=clk, seed=2)
        g.open()

        def run(cmd, wait=0.4):
            g.write_line(cmd)
            clk.t += wait
            return [l for l in g.read_lines() if not l.startswith("TLM ")]

        run("", 0.5)
        self.assertEqual(run("PING"), ["PONG"])
        out = run("SV0_ON:1900,1900")
        self.assertTrue(out[0].startswith("SENT 11 "))
        self.assertTrue(out[1].endswith(" 11 1"))
        self.assertTrue(run("SV1_ON")[1].endswith(" 13 0"), "SRV refused while FILL open")
        self.assertTrue(run("IGN_ARM:1000")[1].endswith(" 17 0"), "no arming with FILL open")
        run("SV0_OFF")
        self.assertTrue(run("IGN_ARM:1000")[1].endswith(" 17 1"))
        self.assertTrue(run("IGN_ON")[1].endswith(" 18 0"), "arm countdown not done")
        clk.t += 1.0
        self.assertTrue(run("IGN_ON")[1].endswith(" 18 1"))
        self.assertIsNotNone(g.daq.ignited_at, "IGN starts the simulated burn")
        self.assertEqual(run("SV0_ON:3000,1900"), ["ERR targets out of range"])
        self.assertEqual(run("SV0_ON:abc"), ["ERR invalid targets"])
        self.assertEqual(run("NOPE"), ["ERR unknown command"])

    def test_sim_serial_round_trip(self):
        clk = Clock(0.0)
        port = SimSerial(SimGround(clock=clk, seed=3))
        self.assertFalse(port.isOpen())
        self.assertTrue(port.open())
        port.write(b"PING\n")
        clk.t += 0.3
        self.assertTrue(port.poll())
        data = port.readAll().decode()
        self.assertIn("PONG\r\n", data)
        self.assertIn("Ready.", data)
        port.close()
        self.assertFalse(port.isOpen())


if __name__ == "__main__":
    unittest.main()
