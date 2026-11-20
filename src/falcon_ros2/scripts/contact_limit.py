"""Contact force/torque limit for the teleop bridge, without ROS (unit-tested in test/).

exceeds_limit() is the trip decision; ContactLimit adds the latch: once tripped it stays tripped
until the clutch is released and pressed again, so the operator has to re-engage on purpose.
"""
import math


def _norm(v):
    return math.sqrt(sum(c * c for c in v))


def exceeds_limit(force, torque, force_limit, torque_limit):
    """Why the contact wrench is over a limit, or None.

    force: N, torque: Nm (3-vectors, any frame: only magnitudes are compared; torque may be None).
    A limit <= 0 disables that check. Reaching a limit counts as exceeding it.
    """
    reasons = []
    fmag = _norm(force)
    if force_limit > 0.0 and fmag >= force_limit:
        reasons.append(f'contact force {fmag:.1f} N >= {force_limit:.1f} N')
    if torque is not None:
        tmag = _norm(torque)
        if torque_limit > 0.0 and tmag >= torque_limit:
            reasons.append(f'contact torque {tmag:.2f} Nm >= {torque_limit:.2f} Nm')
    return '; '.join(reasons) or None


class ContactLimit:
    """Latching contact limit.

    update() is called every force tick. While the clutch is released nothing is checked (the
    contact wrench is only zeroed while engaged, and the arm isn't commanded then). After a trip,
    tripped stays True until the clutch has been released and pressed again; the check then runs
    again straight away, so pressing the clutch while still over the limit trips again.
    """

    def __init__(self):
        self.tripped = False
        self.reason = None
        self._released_since_trip = False

    def update(self, force, torque, clutch, force_limit, torque_limit):
        """Return True on the tick the limit trips (not while it stays tripped)."""
        if self.tripped:
            if not clutch:
                self._released_since_trip = True
                return False
            if not self._released_since_trip:
                return False
            self.tripped = False  # released and pressed again: re-armed
            self.reason = None
        if not clutch or force is None:
            return False
        reason = exceeds_limit(force, torque, force_limit, torque_limit)
        if reason is None:
            return False
        self.tripped = True
        self.reason = reason
        self._released_since_trip = False
        return True
