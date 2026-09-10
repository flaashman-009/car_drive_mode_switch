from r2_diff_driver.pid_controller import Pid1D


def test_p_controller_returns_proportional_output():
    pid = Pid1D(kp=2.0)
    output = pid.update(1.0, 0.04)
    assert output == 2.0


def test_integral_is_bounded():
    pid = Pid1D(kp=0.0, ki=10.0, integral_limit=5.0)
    for _ in range(1000):
        pid.update(1.0, 0.04)
    assert pid.integral == 5.0


def test_output_is_bounded():
    pid = Pid1D(kp=1000.0, output_limit=100.0)
    assert pid.update(1.0, 0.04) == 100.0


def test_reset_clears_state():
    pid = Pid1D(kp=1.0, ki=1.0)
    pid.update(1.0, 0.04)
    pid.reset()
    assert pid.integral == 0.0
    assert pid.previous_error == 0.0


def test_integral_does_not_wind_up_when_saturated():
    pid = Pid1D(kp=10.0, ki=10.0, output_limit=1.0)
    pid.update(10.0, 0.1)
    assert pid.integral == 0.0
