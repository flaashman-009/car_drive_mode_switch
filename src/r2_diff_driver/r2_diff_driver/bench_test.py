"""Bench calibration helpers for the R2 differential-drive mode.

Modes:
  default         scan M1..M4 with +CMD/-CMD and report encoder deltas
  --distance-cal  calibrate ticks-per-meter by push, then cmd_per_mps by
                  PWM ramp with the rear wheels lifted
"""

import argparse
import time


CMD = 25
RUN_SEC = 1.0
SETTLE_SEC = 0.5
DEFAULT_LEVELS = (20, 40, 60, 80, 100)


def enc_delta(before, after):
    return [a - b for a, b in zip(after, before)]


def read_encoders(car):
    return list(car.get_motor_encoder())


def send_channel(car, channel, raw):
    values = [0, 0, 0, 0]
    values[channel - 1] = raw
    car.set_motor(values[0], values[1], values[2], values[3])


def run_channel(car, index, cmd, seconds):
    send_channel(car, index + 1, cmd)
    time.sleep(seconds)
    car.set_motor(0, 0, 0, 0)
    time.sleep(SETTLE_SEC)


def calibrate_ticks_per_meter(car, distance_m):
    print("Place the car at the start line and keep the wheels on the ground.")
    input("Press Enter when ready to record the start encoder count: ")
    before = read_encoders(car)
    print("Push the car forward exactly %.3f m, then stop." % distance_m)
    input("Press Enter after the car has reached the end: ")
    after = read_encoders(car)
    ticks = [a - b for a, b in zip(after, before)]
    print("push deltas:", ticks)
    return ticks


def measure_rate(car, channel, raw, run_sec):
    car.set_motor(0, 0, 0, 0)
    time.sleep(SETTLE_SEC)
    before = read_encoders(car)
    send_channel(car, channel, raw)
    time.sleep(run_sec)
    car.set_motor(0, 0, 0, 0)
    time.sleep(SETTLE_SEC)
    after = read_encoders(car)
    delta = after[channel - 1] - before[channel - 1]
    return delta / run_sec


def fit_cmd_per_mps(levels, rates, ticks_per_meter):
    num = 0.0
    den = 0.0
    rows = []
    for raw, rate in zip(levels, rates):
        speed = rate / ticks_per_meter
        cpm = raw / speed if speed else float("nan")
        num += raw * speed
        den += speed * speed
        rows.append((raw, rate, speed, cpm))
    fitted = num / den if den else float("nan")
    return rows, fitted


def ramp_calibration(car, channel, label, ticks_per_meter,
                     levels, run_sec):
    print("=== %s ramp ===" % label)
    rates = []
    for raw in levels:
        rate = measure_rate(car, channel, raw, run_sec)
        rates.append(rate)
        print("raw=%d rate=%.1f ticks/s" % (raw, rate), flush=True)
    rows, fitted = fit_cmd_per_mps(levels, rates, ticks_per_meter)
    for raw, rate, speed, cpm in rows:
        print("raw=%d -> %.3f m/s, per-point cmd_per_mps=%.2f"
              % (raw, speed, cpm))
    print("fitted cmd_per_mps = %.2f" % fitted)
    return fitted


def run_distance_calibration(car, args):
    print("=== distance-cal safety ===")
    print("Part 1 is a manual push with motors OFF.")
    print("Part 2 runs PWM with rear wheels LIFTED.")
    if args.ticks_per_meter:
        ticks = [args.ticks_per_meter, args.ticks_per_meter]
        print("using provided ticks_per_meter:", args.ticks_per_meter)
    else:
        ticks = calibrate_ticks_per_meter(car, args.distance_m)
    if not args.no_lift_warning:
        print("Lift the rear wheels off the ground before Part 2.")
        input("Press Enter when the rear wheels are lifted: ")

    left_ticks = ticks[args.left_encoder - 1]
    right_ticks = ticks[args.right_encoder - 1]
    print("left ticks/m=%.1f right ticks/m=%.1f"
          % (left_ticks, right_ticks))

    levels = tuple(int(v) for v in args.pwm_levels.split(",") if v.strip())
    left_cpm = ramp_calibration(
        car, args.left_motor, "left wheel", left_ticks, levels, args.run_sec)
    right_cpm = ramp_calibration(
        car, args.right_motor, "right wheel", right_ticks, levels, args.run_sec)

    print("=== recommended yaml values ===")
    print("left_cmd_per_mps: %.2f" % left_cpm)
    print("right_cmd_per_mps: %.2f" % right_cpm)
    print("left_encoder_ticks_per_meter: %.1f" % left_ticks)
    print("right_encoder_ticks_per_meter: %.1f" % right_ticks)


def main():
    parser = argparse.ArgumentParser(
        description="R2 differential-mode calibration helpers. "
                    "Default mode requires the rear wheels lifted.")
    parser.add_argument("--serial-port", default="/dev/myserial")
    parser.add_argument("--car-type", type=int, default=5)
    parser.add_argument("--distance-cal", action="store_true",
                        help="run push + PWM ramp calibration")
    parser.add_argument("--left-motor", type=int, default=2)
    parser.add_argument("--right-motor", type=int, default=4)
    parser.add_argument("--left-encoder", type=int, default=2)
    parser.add_argument("--right-encoder", type=int, default=4)
    parser.add_argument("--distance-m", type=float, default=1.0)
    parser.add_argument("--ticks-per-meter", type=float, default=0.0)
    parser.add_argument("--pwm-levels",
                        default=",".join(str(v) for v in DEFAULT_LEVELS))
    parser.add_argument("--run-sec", type=float, default=2.0)
    parser.add_argument("--no-lift-warning", action="store_true",
                        help="skip the lift prompt between push and PWM ramp")
    args = parser.parse_args()

    from Rosmaster_Lib import Rosmaster

    car = Rosmaster(com=args.serial_port, car_type=args.car_type)
    car.set_car_type(args.car_type)
    car.create_receive_threading()
    try:
        try:
            car.set_akm_steering_angle(0.0, ctrl_car=False)
        except Exception:
            pass

        if args.distance_cal:
            run_distance_calibration(car, args)
            return

        print("=== channel scan: +%d for %.1fs each ===" % (CMD, RUN_SEC))
        for i in range(4):
            car.set_motor(0, 0, 0, 0)
            time.sleep(SETTLE_SEC)
            before = read_encoders(car)
            run_channel(car, i, CMD, RUN_SEC)
            after = read_encoders(car)
            print("motor channel %d: encoder deltas %s"
                  % (i + 1, enc_delta(before, after)))

        print("=== direction scan: repeat channels with -%d ===" % CMD)
        for i in range(4):
            car.set_motor(0, 0, 0, 0)
            time.sleep(SETTLE_SEC)
            before = read_encoders(car)
            run_channel(car, i, -CMD, RUN_SEC)
            after = read_encoders(car)
            print("motor channel %d reverse: encoder deltas %s"
                  % (i + 1, enc_delta(before, after)))
    finally:
        car.set_motor(0, 0, 0, 0)
        print("motors stopped")


if __name__ == "__main__":
    main()
