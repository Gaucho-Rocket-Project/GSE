"""Safety state machine for LIVE mode of the ESP-NOW ground controller.

This module is deliberately free of Qt/serial dependencies so the interlocks
can be unit-tested without a display or hardware.

FILL timeout policy (per operator requirement): when the countdown expires the
operator is notified, but FILL stays OPEN. Closing is manual so it can be done
early or delayed with the EXTEND button.
"""

from __future__ import annotations

import time
from typing import Callable, List, Tuple

OFF = "OFF"
ARMING = "ARMING"
ARMED = "ARMED"
ON = "ON"

FILL_TIME_UP = "FILL_TIME_UP"
IGN_ARMED = "IGN_ARMED"
REJECT = "REJECT"


class LiveState:
    """Tracks FILL / VEH / IGN state and the two countdowns."""

    def __init__(
        self,
        fill_seconds: float = 60.0,
        ign_seconds: float = 5.0,
        fill_extend_seconds: float = 10.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.fill_seconds = float(fill_seconds)
        self.ign_seconds = float(ign_seconds)
        self.fill_extend_seconds = float(fill_extend_seconds)
        self._clock = clock

        self.fill_open = False
        self.fill_deadline: float | None = None
        self.fill_notified = False
        self.veh_open = False
        self.ign_state = OFF
        self.ign_deadline: float | None = None
        self._events: List[Tuple[str, str]] = []

    # ---------------------------------------------------------------- helpers
    def _now(self) -> float:
        return self._clock()

    def _emit(self, code: str, message: str) -> None:
        self._events.append((code, message))

    def drain_events(self) -> List[Tuple[str, str]]:
        events = list(self._events)
        self._events.clear()
        return events

    def set_durations(
        self,
        fill_seconds: float | None = None,
        ign_seconds: float | None = None,
        fill_extend_seconds: float | None = None,
    ) -> None:
        """Change durations for the *next* countdown, not the running one."""
        if fill_seconds is not None:
            self.fill_seconds = float(fill_seconds)
        if ign_seconds is not None:
            self.ign_seconds = float(ign_seconds)
        if fill_extend_seconds is not None:
            self.fill_extend_seconds = float(fill_extend_seconds)

    # ------------------------------------------------------------ FILL (svc0)
    def fill_on(self) -> bool:
        if self.fill_open:
            self._emit(REJECT, "FILL is already OPEN")
            return False
        if self.veh_open or self.ign_state != OFF:
            self._emit(REJECT, "FILL refused: close VEH and reset IGN first")
            return False
        self.fill_open = True
        self.fill_deadline = self._now() + self.fill_seconds
        self.fill_notified = False
        self._emit("FILL_OPEN", f"FILL OPEN - {self.fill_seconds:.0f}s countdown started")
        return True

    def fill_close(self) -> bool:
        if not self.fill_open:
            self._emit(REJECT, "FILL is already CLOSED")
            return False
        self.fill_open = False
        self.fill_deadline = None
        self.fill_notified = False
        self._emit("FILL_CLOSED", "FILL CLOSED early")
        return True

    def fill_extend(self, extra: float | None = None) -> bool:
        """Delay the FILL close deadline (operator can postpone closing)."""
        if not self.fill_open:
            self._emit(REJECT, "FILL is not open - nothing to extend")
            return False
        amount = self.fill_extend_seconds if extra is None else float(extra)
        self.fill_deadline = max(self.fill_deadline or self._now(), self._now()) + amount
        self.fill_notified = False
        self._emit("FILL_EXTENDED", f"FILL close delayed by {amount:.0f}s")
        return True

    # -------------------------------------------------------------- VEH (svc1)
    def veh_on(self) -> bool:
        if self.veh_open:
            self._emit(REJECT, "VEH is already OPEN")
            return False
        if self.fill_open:
            self._emit(REJECT, "VEH refused: FILL must be CLOSED first")
            return False
        self.veh_open = True
        self._emit("VEH_OPEN", "VEH OPEN")
        return True

    def veh_off(self) -> bool:
        if not self.veh_open:
            self._emit(REJECT, "VEH is already CLOSED")
            return False
        self.veh_open = False
        self._emit("VEH_CLOSED", "VEH CLOSED")
        return True

    # ----------------------------------------------------------------- IGN
    def ign_arm(self) -> bool:
        if self.ign_state != OFF:
            self._emit(REJECT, "IGN must be reset to OFF before arming again")
            return False
        if self.fill_open or self.veh_open:
            self._emit(REJECT, "IGN refused: FILL must be CLOSED (and VEH closed)")
            return False
        self.ign_state = ARMING
        self.ign_deadline = self._now() + self.ign_seconds
        self._emit("IGN_ARMING", f"IGN countdown started ({self.ign_seconds:.0f}s) - ON is locked out")
        return True

    def ign_on(self) -> bool:
        if self.fill_open or self.veh_open:
            self._emit(REJECT, "IGN refused: FILL must be CLOSED")
            return False
        if self.ign_state != ARMED:
            self._emit(REJECT, "IGN ON locked out until the countdown completes")
            return False
        self.ign_state = ON
        self.ign_deadline = None
        self._emit("IGN_ON", "IGN ON (latched - reset with IGN OFF)")
        return True

    def ign_off(self) -> bool:
        if self.ign_state == OFF:
            self._emit(REJECT, "IGN is already OFF")
            return False
        self.ign_state = OFF
        self.ign_deadline = None
        self._emit("IGN_OFF", "IGN reset to OFF")
        return True

    # --------------------------------------------------------------- global
    def all_off(self) -> None:
        self.fill_open = False
        self.fill_deadline = None
        self.fill_notified = False
        self.veh_open = False
        self.ign_state = OFF
        self.ign_deadline = None
        self._emit("ALL_OFF", "ALL OFF - FILL/VEH closed, IGN reset")

    def tick(self) -> List[Tuple[str, str]]:
        """Advance countdowns; call periodically. Returns newly fired events."""
        now = self._now()
        if (
            self.fill_open
            and not self.fill_notified
            and self.fill_deadline is not None
            and now >= self.fill_deadline
        ):
            self.fill_notified = True
            self._emit(
                FILL_TIME_UP,
                "FILL countdown COMPLETE - FILL remains OPEN; close it when ready",
            )

        if (
            self.ign_state == ARMING
            and self.ign_deadline is not None
            and now >= self.ign_deadline
        ):
            self.ign_state = ARMED
            self.ign_deadline = None
            self._emit(
                IGN_ARMED,
                "IGN countdown COMPLETE - ARMED: trigger ON or reset OFF",
            )

        return self.drain_events()

    def remaining(self) -> Tuple[float | None, float | None]:
        """Seconds left for (FILL, IGN); None when that timer is inactive."""
        now = self._now()
        fill = (
            max(0.0, self.fill_deadline - now)
            if self.fill_open and self.fill_deadline is not None
            else None
        )
        ign = (
            max(0.0, self.ign_deadline - now)
            if self.ign_state == ARMING and self.ign_deadline is not None
            else None
        )
        return fill, ign

    # ------------------------------------------------------- button gating
    @property
    def can_fill_on(self) -> bool:
        return not self.fill_open and not self.veh_open and self.ign_state == OFF

    @property
    def can_fill_close(self) -> bool:
        return self.fill_open

    @property
    def can_fill_extend(self) -> bool:
        return self.fill_open

    @property
    def can_veh_on(self) -> bool:
        return not self.veh_open and not self.fill_open

    @property
    def can_veh_off(self) -> bool:
        return self.veh_open

    @property
    def can_ign_arm(self) -> bool:
        return self.ign_state == OFF and not self.fill_open and not self.veh_open

    @property
    def can_ign_on(self) -> bool:
        return self.ign_state == ARMED and not self.fill_open and not self.veh_open

    @property
    def can_ign_off(self) -> bool:
        return self.ign_state != OFF
