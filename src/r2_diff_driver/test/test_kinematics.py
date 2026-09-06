import pytest

from r2_diff_driver.kinematics import twist_to_wheel_speeds, wheel_speeds_to_twist


def test_straight_line_has_equal_wheel_speeds():
    left, right = twist_to_wheel_speeds(0.5, 0.0, 0.1646)
    assert left == pytest.approx(0.5)
    assert right == pytest.approx(0.5)


def test_turning_makes_wheels_asymmetric():
    left, right = twist_to_wheel_speeds(0.0, 1.0, 0.1646)
    assert left < 0
    assert right > 0
    assert abs(left) == pytest.approx(abs(right))


def test_round_trip():
    left, right = twist_to_wheel_speeds(0.4, 0.8, 0.1646)
    linear, angular = wheel_speeds_to_twist(left, right, 0.1646)
    assert linear == pytest.approx(0.4)
    assert angular == pytest.approx(0.8)
