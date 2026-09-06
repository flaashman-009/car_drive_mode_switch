"""Bench calibration helper for the R2 differential-mode motor channels."""

import argparse
import time


CMD = 25
RUN_SEC = 1.0
SETTLE_SEC = 0.5


def enc_delta(before, after):
    return [a - b for a, b in zip(after, before)]


def run_channel(car, index, cmd, seconds):
    values = [0, 0, 0, 0]
    values[index] = cmd
    car.set_motor(values[0], values[1], values[2], values[3])
    time.sleep(seconds)
    car.set_motor(0, 0, 0, 0)
    time.sleep(SETTLE_SEC)


def main():
    parser = argparse.ArgumentParser(
        description="Discover R2 set_motor channel wiring. "
                    "Keep the rear wheels lifted.")
    parser.add_argument("--serial-port", default="/dev/myserial")
    parser.add_argument("--car-type", type=int, default=5)
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

        print("=== channel scan: +%d for %.1fs each ===" % (CMD, RUN_SEC))
        for i in range(4):
            car.set_motor(0, 0, 0, 0)
            time.sleep(SETTLE_SEC)
            before = car.get_motor_encoder()
            run_channel(car, i, CMD, RUN_SEC)
            after = car.get_motor_encoder()
            print("motor channel %d: encoder deltas %s"
                  % (i + 1, enc_delta(before, after)))

        print("=== direction scan: repeat channels with -%d ===" % CMD)
        for i in range(4):
            car.set_motor(0, 0, 0, 0)
            time.sleep(SETTLE_SEC)
            before = car.get_motor_encoder()
            run_channel(car, i, -CMD, RUN_SEC)
            after = car.get_motor_encoder()
            print("motor channel %d reverse: encoder deltas %s"
                  % (i + 1, enc_delta(before, after)))
    finally:
        car.set_motor(0, 0, 0, 0)
        print("motors stopped")


if __name__ == "__main__":
    main()
