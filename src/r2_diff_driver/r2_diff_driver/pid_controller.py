"""Small single-axis PID controller used by the wheel speed loop."""


class Pid1D:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0,
                 integral_limit=30.0, output_limit=100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        self.output_limit = abs(output_limit)
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.has_previous_error = False

    def update(self, error, dt):
        if dt <= 0.0:
            dt = 0.001

        proportional = self.kp * error

        derivative = 0.0
        if self.kd != 0.0:
            if self.has_previous_error:
                derivative = self.kd * (error - self.previous_error) / dt
            self.previous_error = error
            self.has_previous_error = True

        if self.ki != 0.0:
            candidate = self.integral + self.ki * error * dt
            candidate = max(-self.integral_limit,
                            min(self.integral_limit, candidate))
            projected = proportional + candidate + derivative
            saturated_high = projected > self.output_limit and error > 0.0
            saturated_low = projected < -self.output_limit and error < 0.0
            if not (saturated_high or saturated_low):
                self.integral = candidate

        output = proportional + self.integral + derivative
        return max(-self.output_limit, min(self.output_limit, output))
