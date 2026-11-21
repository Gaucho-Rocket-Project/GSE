"""Offscreen tests of the DAQ panel in the window (--sim); skipped without Qt."""
import os
import sys
import tempfile
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
try:
    from PySide6.QtCore import QSettings
    from PySide6.QtWidgets import QApplication
    from ground_controller import MainWindow
    from daq_panel import DaqSettingsDialog
except ImportError:
    QApplication = None


@unittest.skipUnless(QApplication, "PySide6 or native Qt libraries not installed")
class DaqPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self._old = os.environ.get("GSE_DATA_DIR")
        os.environ["GSE_DATA_DIR"] = os.path.join(self.tmp.name, "data")
        self.store = QSettings(os.path.join(self.tmp.name, "s.ini"), QSettings.IniFormat)

    def tearDown(self):
        if self._old is None:
            os.environ.pop("GSE_DATA_DIR", None)
        else:
            os.environ["GSE_DATA_DIR"] = self._old
        self.tmp.cleanup()

    def pump(self, seconds, until=None):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            self.app.processEvents()
            if until and until():
                return True
            time.sleep(0.01)
        return False

    def test_sim_telemetry_reaches_the_panel(self):
        w = MainWindow(sim=True, sim_interval_ms=200, store=self.store)
        w.show()
        w.toggle_serial()
        self.assertTrue(self.pump(3, lambda: w.daq.p.frames >= 3), "no DAQ frames")
        w.on_tick()
        self.assertTrue(w.daq_panel.rows[0]["now"].text().endswith("psi"))
        self.assertIn("FAULT", w.daq_panel.rows[4 + 3]["status"].text())
        self.assertIsNotNone(w.daq.csv_path)
        self.assertFalse(w.daq_panel.chart.grab().isNull())
        self.assertIn("#1  D,", w.telemetry_view.toPlainText())  # raw view still works
        w.toggle_serial()
        w.close()

    def test_format_toggle_is_saved_and_drives_the_sim(self):
        w = MainWindow(sim=True, sim_interval_ms=200, store=self.store)
        w.toggle_serial()
        w.daq_panel.format_combo.setCurrentIndex(1)
        self.assertEqual(w.daq.fmt, "separate")
        self.assertEqual(w.sim_ground.fmt, "separate")
        self.assertEqual(self.store.value("daq/format"), "separate")
        self.assertTrue(self.pump(3, lambda: w.daq.p.frames >= 2))
        w.toggle_serial()
        w.close()
        again = MainWindow(sim=True, store=self.store)
        self.assertEqual(again.daq.fmt, "separate")
        again.close()

    def test_real_ground_text_in_arbitrary_serial_chunks(self):
        """What ground.ino prints, delivered the way a real port can split it."""
        stream = (b"ESP-NOW servo/IGN ground\r\nReady. Test: 1/2/3/4. Live: ...\r\n"
                  b"PONG\r\nTLM 1 3000 heartbeat\r\nSENT 11 123\r\nACK 123 11 1\r\n"
                  b"TLM 2 3500 D,40,400000,12000,4000,8000,8000,21.00,20.00,19.50,,00,00,00,01\r\n"
                  b"TLM 3 4000 D,90,900000,12100,4000,8000,8000,21.10,20.00,19.50,,00,00,00,01\r\n"
                  b"ERR unknown command\r\n")

        class FakePort:
            def __init__(self):
                self.chunks = []

            def readAll(self):
                return self.chunks.pop(0)

            def isOpen(self):
                return True

        for size in (1, 7, 64, len(stream)):
            w = MainWindow(store=self.store)
            w.serial = FakePort()
            for i in range(0, len(stream), size):
                w.serial.chunks.append(stream[i:i + size])
                w.on_ready_read()
            self.assertEqual((w.daq.tlm.frames, w.daq.p.frames, w.daq.hb.frames), (3, 2, 1), size)
            self.assertEqual(w.daq.latest_p.ua[0], 12100)
            self.assertEqual(w.daq.p.gaps, 49)
            log = w.log_view.toPlainText()
            self.assertIn("ACK cmd 11 accepted", log)
            self.assertIn("ERR unknown command", log)
            w.daq.close()
            w.close()

    def test_daq_errors_never_break_the_command_link(self):
        w = MainWindow(store=self.store)

        def boom(*_args):
            raise RuntimeError("simulated DAQ bug")

        w.daq.feed_tlm = boom
        w.daq_panel.refresh = boom
        for line in ("TLM 1 0 D,1,0,4000,4000,4000,4000,20,20,20,20,00,00,00,00",
                     "ACK 9 11 0", "ERR targets out of range"):
            w.parse_line(line)
        w.on_tick()
        w.on_tick()
        log = w.log_view.toPlainText()
        self.assertIn("DAQ data error (record ignored)", log)
        self.assertIn("REJECTED by actuator: cmd 11", log, "ACK after the bad record still handled")
        self.assertIn("ERR targets out of range", log)
        self.assertEqual(log.count("DAQ panel display error"), 1)
        w.close()

    def test_ranges_dialog_validates_and_applies(self):
        w = MainWindow(sim=True, store=self.store)
        dlg = DaqSettingsDialog(w.daq.settings, w)
        dlg.p_rows[1][1].setValue(500)
        dlg.p_rows[1][2].setValue(100)
        self.assertFalse(dlg.save())
        self.assertIn("max must be greater", dlg.error.text())
        dlg.p_rows[1][1].setValue(0)
        dlg.p_rows[1][2].setValue(750)
        dlg.t_rows[0].setText("Nozzle")
        dlg.unit.setCurrentText("F")
        self.assertTrue(dlg.save())
        self.assertEqual(w.daq_panel.apply_settings(dlg.result_settings), [])
        self.assertEqual(w.daq.settings.pressure[1].max_psi, 750)
        self.assertEqual(w.daq_panel.rows[1]["range"].text(), "0-750 psi")
        self.assertEqual(w.daq_panel.rows[4]["name"].text(), "TC1 Nozzle")
        self.assertIn("750", self.store.value("daq/settings"))
        w.close()
        self.assertEqual(MainWindow(sim=True, store=self.store).daq.settings.pressure[1].max_psi, 750)


if __name__ == "__main__":
    unittest.main()
