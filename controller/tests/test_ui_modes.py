"""Offscreen UI regression tests; skipped where Qt's native libraries are absent."""
import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
try:
    from PySide6.QtWidgets import QApplication
    from ground_controller import MainWindow
except ImportError:
    QApplication = None


@unittest.skipUnless(QApplication, "PySide6 or native Qt libraries not installed")
class ModeControlsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_live_ign_greyed_until_fill_closed_and_countdown_done(self):
        window = MainWindow()
        class ConnectedSerial:
            def isOpen(self):
                return True
        window.serial = ConnectedSerial()
        window.send = lambda command: True
        now = [0.0]
        window.state._clock = lambda: now[0]
        window.show()
        window.mode_combo.setCurrentText("LIVE")
        self.assertTrue(window.btn_ign_arm.isEnabled())
        self.assertFalse(window.btn_ign_on.isEnabled())
        window.fill_on_clicked()
        # No timer/event processing needed: the visual gate changes immediately.
        self.assertFalse(window.btn_ign_arm.isEnabled())
        self.assertFalse(window.btn_ign_on.isEnabled())
        self.assertIn("LOCKED", window.ign_gate_label.text())
        now[0] += 60
        window.on_tick()
        self.assertFalse(window.btn_ign_arm.isEnabled())
        self.assertFalse(window.btn_ign_on.isEnabled())
        window.fill_close_clicked()
        self.assertTrue(window.btn_ign_arm.isEnabled())
        self.assertFalse(window.btn_ign_on.isEnabled())
        window.ign_arm_clicked()
        self.assertFalse(window.btn_ign_on.isEnabled())
        now[0] += 5
        window.on_tick()
        self.assertTrue(window.btn_ign_on.isEnabled())
        window.close()

    def test_srv_and_combined_controls_are_test_only(self):
        window = MainWindow()
        sent = []
        window.send = lambda command: sent.append(command) or True
        window.show()
        self.app.processEvents()
        self.assertFalse(window.srv_label.isHidden())
        self.assertFalse(window.btn_srv_on.isHidden())
        window.srv_on_clicked()
        self.assertEqual(sent[-1], "SV1_ON")
        window.mode_combo.setCurrentText("LIVE")
        self.app.processEvents()
        self.assertTrue(window.srv_label.isHidden())
        self.assertTrue(window.btn_srv_on.isHidden())
        self.assertTrue(window.btn_srv_off.isHidden())
        self.assertTrue(window.btn_both_pulse.isHidden())
        sent.clear()
        window.srv_on_clicked()
        window.srv_off_clicked()
        window.both_pulse_clicked()
        self.assertEqual(sent, [])
        window.mode_combo.setCurrentText("TEST")
        self.app.processEvents()
        self.assertFalse(window.btn_srv_on.isHidden())
        window.close()
