"""Unit tests for the contact force/torque limit (no ROS needed): pytest test/test_contact_limit.py"""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from contact_limit import ContactLimit, exceeds_limit  # noqa: E402

ZERO = (0.0, 0.0, 0.0)


def test_trips_at_and_above_the_force_limit():
    assert exceeds_limit((0.0, 0.0, 19.9), ZERO, 20.0, 2.0) is None
    assert exceeds_limit((0.0, 0.0, 20.0), ZERO, 20.0, 2.0) is not None
    assert exceeds_limit((12.0, 16.0, 0.0), ZERO, 20.0, 2.0) is not None  # |F| = 20 N
    assert 'force' in exceeds_limit((0.0, 0.0, -25.0), ZERO, 20.0, 2.0)


def test_trips_at_the_torque_limit():
    assert exceeds_limit(ZERO, (0.0, 1.9, 0.0), 20.0, 2.0) is None
    assert 'torque' in exceeds_limit(ZERO, (0.0, 2.0, 0.0), 20.0, 2.0)
    assert exceeds_limit(ZERO, None, 20.0, 2.0) is None  # no torque data: torque check skipped


def test_zero_limit_disables_the_check():
    assert exceeds_limit((0.0, 0.0, 500.0), ZERO, 0.0, 2.0) is None
    assert exceeds_limit(ZERO, (50.0, 0.0, 0.0), 20.0, 0.0) is None
    assert exceeds_limit((0.0, 0.0, 500.0), (50.0, 0.0, 0.0), 0.0, 0.0) is None


def test_latched_until_clutch_released_and_pressed_again():
    cl = ContactLimit()
    big, small = (0.0, 0.0, 30.0), (0.0, 0.0, 1.0)
    assert cl.update(small, ZERO, True, 20.0, 2.0) is False and not cl.tripped
    assert cl.update(big, ZERO, True, 20.0, 2.0) is True and cl.tripped
    # Stays tripped while the clutch is held, even when the force has gone
    for _ in range(5):
        assert cl.update(small, ZERO, True, 20.0, 2.0) is False and cl.tripped
    # Released: still tripped (the arm isn't commanded)
    assert cl.update(small, ZERO, False, 20.0, 2.0) is False and cl.tripped
    # Pressed again with a small force: cleared
    assert cl.update(small, ZERO, True, 20.0, 2.0) is False and not cl.tripped


def test_repress_while_still_over_the_limit_trips_again():
    cl = ContactLimit()
    big = (0.0, 0.0, 30.0)
    assert cl.update(big, ZERO, True, 20.0, 2.0) is True
    cl.update(big, ZERO, False, 20.0, 2.0)
    assert cl.update(big, ZERO, True, 20.0, 2.0) is True and cl.tripped


def test_nothing_checked_while_released_or_without_data():
    cl = ContactLimit()
    assert cl.update((0.0, 0.0, 99.0), ZERO, False, 20.0, 2.0) is False and not cl.tripped
    assert cl.update(None, None, True, 20.0, 2.0) is False and not cl.tripped


def test_disabled_limit_never_trips_or_latches():
    cl = ContactLimit()
    assert cl.update((0.0, 0.0, 500.0), (9.0, 0.0, 0.0), True, 0.0, 0.0) is False
    assert not cl.tripped
