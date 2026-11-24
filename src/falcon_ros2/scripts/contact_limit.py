"""Contact force/torque limit for the teleop bridge, without ROS (unit-tested in test/).

exceeds_limit() is the trip decision; ContactLimit adds the latch: once tripped it stays tripped
until the clutch is released and pressed again, so the operator has to re-engage on purpose.
The contact wrench is measured against one zero for the whole trial (not re-zeroed per clutch
press), so re-pressing while still pressed against something can leave the limit exceeded. The
latch then stays on in a recovering state in which only motion away from the contact is allowed
(away_only()); it clears by itself once the contact is below the limit.
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


def away_only(v, away, min_force=1.0):
    """Motion v (3-vector) with any component into the contact removed.

    away: the contact force on the tool (pointing away from what pushes it). Below min_force (N)
    there is no usable direction, and no motion is allowed.
    """
    amag = _norm(away)
    if amag < min_force:
        return (0.0, 0.0, 0.0)
    u = tuple(c / amag for c in away)
    into = sum(v[i] * u[i] for i in range(3))
    if into >= 0.0:
        return tuple(v)
    return tuple(v[i] - into * u[i] for i in range(3))


class ContactLimit:
    """Latching contact limit.

    update() is called every force tick. While the clutch is released nothing is checked (the arm
    isn't commanded then). After a trip, tripped stays True until the clutch has been released and
    pressed again. On that press the wrench is checked straight away: below the limit the latch
    clears; still at or over it, the latch stays on with recovering = True (only motion away from
    the contact allowed) until the wrench drops below the limit or the clutch is released.
    """

    def __init__(self):
        self.tripped = False
        self.recovering = False
        self.reason = None
        self._released_since_trip = False

    def update(self, force, torque, clutch, force_limit, torque_limit):
        """Return True on the tick the limit trips (not while it stays tripped or recovers)."""
        if self.tripped:
            if not clutch:
                self._released_since_trip = True
                self.recovering = False
                return False
            if not (self._released_since_trip or self.recovering):
                return False  # still holding the clutch from the trip
            if force is None:
                return False
            reason = exceeds_limit(force, torque, force_limit, torque_limit)
            self._released_since_trip = False
            if reason is None:  # re-pressed (or recovered) below the limit: re-armed
                self.tripped = self.recovering = False
                self.reason = None
            else:
                self.recovering = True
                self.reason = reason
            return False
        if not clutch or force is None:
            return False
        reason = exceeds_limit(force, torque, force_limit, torque_limit)
        if reason is None:
            return False
        self.tripped = True
        self.recovering = False
        self.reason = reason
        self._released_since_trip = False
        return True
