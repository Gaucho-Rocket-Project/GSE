"""Unit tests for the LIVE-mode safety gates (no hardware / GUI required)."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from live_state import ARMED, ARMING, FILL_TIME_UP, IGN_ARMED, OFF, ON, REJECT, LiveState


class FakeClock:
    def __init__(self) -> None:
        self.now = 1000.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class LiveStateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = FakeClock()
        self.state = LiveState(
            fill_seconds=60, ign_seconds=5, fill_extend_seconds=10, clock=self.clock
        )

    def codes(self) -> list[str]:
        return [code for code, _ in self.state.drain_events()]

    def test_fill_countdown_notifies_but_stays_open(self):
        self.assertTrue(self.state.fill_on())
        self.assertEqual([code for code, _ in self.state.drain_events()], ["FILL_OPEN"])
        self.state.tick()
        self.assertEqual(self.codes(), [])
        self.clock.advance(59)
        self.assertEqual(self.state.tick(), [])
        self.clock.advance(1)
        self.assertEqual([code for code, _ in self.state.tick()], [FILL_TIME_UP])
        self.assertTrue(self.state.fill_open, "FILL must stay OPEN after timeout")
        self.assertEqual(self.state.tick(), [], "notification fires only once")

    def test_fill_can_be_closed_early_or_delayed(self):
        self.state.fill_on()
        self.codes()
        self.clock.advance(30)
        self.assertTrue(self.state.fill_close())
        self.assertFalse(self.state.fill_open)

        self.state.fill_on()
        self.codes()
        self.clock.advance(55)
        self.assertTrue(self.state.fill_extend())
        self.codes()
        self.clock.advance(10)
        self.assertEqual(self.state.tick(), [], "extend must postpone notification")
        self.clock.advance(5)
        self.assertEqual([code for code, _ in self.state.tick()], [FILL_TIME_UP])

    def test_veh_and_ign_are_locked_out_while_fill_open(self):
        self.state.fill_on()
        self.assertFalse(self.state.veh_on())
        self.assertFalse(self.state.ign_arm())
        self.assertFalse(self.state.ign_on())
        self.assertEqual(self.codes().count(REJECT), 3)
        self.assertFalse(self.state.veh_open)
        self.assertEqual(self.state.ign_state, OFF)

    def test_veh_available_after_fill_closed(self):
        self.state.fill_on()
        self.state.fill_close()
        self.codes()
        self.assertTrue(self.state.veh_on())
        self.assertTrue(self.state.veh_open)
        self.assertFalse(self.state.fill_on(), "FILL must not open while VEH is open")
        self.assertFalse(self.state.ign_arm(), "IGN must not arm while VEH is open")
        self.assertTrue(self.state.veh_off())
        self.assertTrue(self.state.veh_open is False)

    def test_ign_cannot_be_prematurely_triggered(self):
        self.assertTrue(self.state.ign_arm())
        self.assertEqual(self.state.ign_state, ARMING)
        self.codes()
        self.clock.advance(4)
        self.assertEqual(self.state.tick(), [])
        self.assertFalse(self.state.ign_on(), "ON must be locked out during countdown")
        self.assertEqual(self.state.ign_state, ARMING)
        self.codes()

        self.clock.advance(1)
        self.assertEqual([code for code, _ in self.state.tick()], [IGN_ARMED])
        self.assertEqual(self.state.ign_state, ARMED)
        self.assertTrue(self.state.ign_on())
        self.assertEqual(self.state.ign_state, ON)
        self.assertTrue(self.state.veh_open, "IGN must open VEH together with ignition")
        self.assertTrue(self.state.ign_off())
        self.assertTrue(self.state.veh_open, "IGN OFF does not close VEH")
        self.assertEqual(self.state.ign_state, OFF)

    def test_manual_veh_blocked_while_arming_or_armed(self):
        self.assertTrue(self.state.ign_arm())
        self.assertFalse(self.state.can_veh_on)
        self.assertFalse(self.state.veh_on())
        self.clock.advance(5)
        self.state.tick()
        self.assertFalse(self.state.can_veh_on)
        self.assertFalse(self.state.veh_on())
        self.assertTrue(self.state.ign_on())
        self.assertTrue(self.state.veh_open)
        self.state.all_off()
        self.assertFalse(self.state.veh_open)
        self.assertEqual(self.state.ign_state, OFF)

    def test_ign_arm_can_be_cancelled(self):
        self.state.ign_arm()
        self.codes()
        self.assertTrue(self.state.ign_off())
        self.codes()
        self.clock.advance(10)
        self.assertEqual(self.state.tick(), [], "cancelled countdown must not fire")

    def test_all_off_resets_everything(self):
        self.state.fill_on()
        self.clock.advance(1)
        self.state.all_off()
        self.assertFalse(self.state.fill_open)
        self.assertFalse(self.state.veh_open)
        self.assertEqual(self.state.ign_state, OFF)
        self.assertIsNone(self.state.fill_deadline)
        self.assertIsNone(self.state.ign_deadline)
        self.assertTrue(self.state.can_fill_on)

    def test_button_gates(self):
        self.assertTrue(self.state.can_fill_on)
        self.assertTrue(self.state.can_veh_on)   # FILL is closed, so VEH is allowed
        self.assertFalse(self.state.can_ign_on)  # IGN still needs its countdown
        self.state.fill_on()
        self.assertTrue(self.state.can_fill_close and self.state.can_fill_extend)
        self.assertFalse(self.state.can_veh_on or self.state.can_ign_arm or self.state.can_ign_on)
        self.state.fill_close()
        self.assertTrue(self.state.can_ign_arm)
        self.assertFalse(self.state.can_ign_on)
        self.state.ign_arm()
        self.assertTrue(self.state.can_ign_off)
        self.assertFalse(self.state.can_ign_on)
        self.clock.advance(5)
        self.state.tick()
        self.assertTrue(self.state.can_ign_on)
        self.state.ign_on()
        self.assertFalse(self.state.can_ign_on)
        self.assertTrue(self.state.can_ign_off)


if __name__ == "__main__":
    unittest.main()
