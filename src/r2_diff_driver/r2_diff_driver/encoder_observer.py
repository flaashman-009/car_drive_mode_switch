"""Convert 40 ms encoder samples into per-wheel speed in m/s.

编码器(encoder)是什么？
- 就是装在轮毂上的"计数器"，每转一转它会输出若干个脉冲(ticks)。
- 读到的是一串累加计数(比如 0,32,64,96...)，不是直接的速度。

怎么得到轮速？
- 用相邻两次读数的差值(这次-上次=这 40ms 转了多少 tick)，再除以时间，再除以
  encoder_ticks_per_meter(每走 1 米会有多少 tick)，就得到 m/s 的轮速。

实际行驶路程是不是用实际轮速推的？
- 没错。如果要算"车实际走了多远"，标准做法就是把这里的 realtime 轮速
  (actual_*_mps) 对时间积分：s += speed * dt。
- 也就是说：路程 = ∫实际轮速 dt，用真实反馈，而不是用目标速度(那会被打滑、
  负载、PID 误差影响)。当前工程只发布了 actual 轮速，还没把它积分成里程计。
"""


def _signed_delta(current, previous, wrap_range):
    delta = current - previous
    if wrap_range and wrap_range > 0:
        half = wrap_range / 2.0
        if delta > half:
            delta -= wrap_range
        elif delta < -half:
            delta += wrap_range
    return delta


class EncoderObserver:
    def __init__(self, left_index, right_index, ticks_per_meter,
                 left_sign=1.0, right_sign=1.0, wrap_range=None):
        if ticks_per_meter <= 0:
            raise ValueError("ticks_per_meter must be positive")
        self.left_index = left_index - 1
        self.right_index = right_index - 1
        self.ticks_per_meter = ticks_per_meter
        self.left_sign = left_sign
        self.right_sign = right_sign
        self.wrap_range = wrap_range
        self.previous = None

    def reset(self):
        self.previous = None

    def update(self, encoder_values, dt):
        """Return (left_mps, right_mps), or None on the first sample."""
        values = [int(v) for v in encoder_values]
        if self.previous is None:
            self.previous = values
            return None
        if dt <= 0.0:
            dt = 0.001

        left_ticks = _signed_delta(
            values[self.left_index],
            self.previous[self.left_index],
            self.wrap_range,
        )
        right_ticks = _signed_delta(
            values[self.right_index],
            self.previous[self.right_index],
            self.wrap_range,
        )
        self.previous = values

        left_mps = left_ticks / dt / self.ticks_per_meter * self.left_sign
        right_mps = right_ticks / dt / self.ticks_per_meter * self.right_sign
        return left_mps, right_mps
