"""Differential-drive ROS2 node with optional encoder feedback."""

import time

import rclpy
from geometry_msgs.msg import Twist
from r2_diff_msgs.msg import WheelState
from rclpy.node import Node
from std_msgs.msg import Bool

from r2_diff_driver.encoder_observer import EncoderObserver
from r2_diff_driver.hardware import create_backend
from r2_diff_driver.kinematics import clamp, twist_to_wheel_speeds
from r2_diff_driver.pid_controller import Pid1D


class DiffDriverNode(Node):
    def __init__(self):
        super().__init__("r2_diff_driver")
        self._declare_parameters()
        self._load_parameters()

        try:
            self.backend = create_backend(
                self.backend_name,
                serial_port=self.serial_port,
                car_type=self.car_type,
            )
            self.backend.connect()
            if self.center_steer:
                self.backend.center_steering()
            self.ready = True
            self.get_logger().info("backend connected: %s" % self.backend_name)
        except Exception as exc:
            self.ready = False
            self.backend = None
            self.get_logger().error("backend connect failed: %s" % exc)

        self.observer = None
        if self.use_feedback and self.encoder_ticks_per_meter > 0:
            try:
                self.observer = EncoderObserver(
                    self.left_encoder,
                    self.right_encoder,
                self.encoder_ticks_per_meter,
                left_sign=self.left_encoder_sign,
                right_sign=self.right_encoder_sign,
                window_size=self.encoder_window_size,
            )
            except ValueError as exc:
                self.get_logger().error("encoder observer disabled: %s" % exc)
                self.use_feedback = False
        elif self.use_feedback:
            self.get_logger().error(
                "use_encoder_feedback is true but encoder_ticks_per_meter "
                "is not calibrated; falling back to open loop")
            self.use_feedback = False

        self.left_pid = Pid1D(
            kp=self.pid_kp,
            ki=self.pid_ki,
            kd=self.pid_kd,
            integral_limit=self.pid_integral_limit,
            output_limit=self.pid_output_limit,
        )
        self.right_pid = Pid1D(
            kp=self.pid_kp,
            ki=self.pid_ki,
            kd=self.pid_kd,
            integral_limit=self.pid_integral_limit,
            output_limit=self.pid_output_limit,
        )

        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_topic, self._cmd_callback, 10)
        self.estop_sub = self.create_subscription(
            Bool, self.estop_topic, self._estop_callback, 10)
        self.wheel_state_pub = self.create_publisher(
            WheelState, self.wheel_state_topic, 10)

        self.estop = False
        self.target_left_mps = 0.0
        self.target_right_mps = 0.0
        self.actual_left_mps = 0.0
        self.actual_right_mps = 0.0
        self.last_raw_left = 0.0
        self.last_raw_right = 0.0
        self.last_cmd_mono = 0.0
        self.last_tick_mono = time.monotonic()
        self.cmd_count = 0
        self.fb_count = 0

        # Chassis reports every 40 ms; use the same period for the loop.
        self.timer = self.create_timer(0.04, self._tick)
        self.get_logger().info(
            "diff driver ready: backend=%s feedback=%s "
            "left_motor=%d right_motor=%d "
            "left_cmd_per_mps=%.1f right_cmd_per_mps=%.1f"
            % (self.backend_name, self.use_feedback, self.left_motor,
               self.right_motor, self.left_cmd_per_mps,
               self.right_cmd_per_mps))

    def _declare_parameters(self):
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("estop_topic", "/estop")
        self.declare_parameter("wheel_state_topic", "/r2_diff/wheel_state")
        self.declare_parameter("backend", "mock")
        self.declare_parameter("serial_port", "/dev/myserial")
        self.declare_parameter("car_type", 5)
        self.declare_parameter("track_width", 0.1646)
        self.declare_parameter("max_speed_mps", 0.5)
        self.declare_parameter("max_omega_radps", 2.0)
        self.declare_parameter("cmd_per_mps", 100.0)
        self.declare_parameter("left_cmd_per_mps", 0.0)
        self.declare_parameter("right_cmd_per_mps", 0.0)
        self.declare_parameter("left_motor", 2)
        self.declare_parameter("right_motor", 4)
        self.declare_parameter("left_encoder", 2)
        self.declare_parameter("right_encoder", 4)
        self.declare_parameter("left_encoder_sign", 1.0)
        self.declare_parameter("right_encoder_sign", 1.0)
        self.declare_parameter("encoder_ticks_per_meter", 0.0)
        self.declare_parameter("encoder_window_size", 3)
        self.declare_parameter("use_encoder_feedback", False)
        self.declare_parameter("pid_kp", 0.5)
        self.declare_parameter("pid_ki", 1.5)
        self.declare_parameter("pid_kd", 0.0)
        self.declare_parameter("pid_integral_limit", 30.0)
        self.declare_parameter("pid_output_limit", 100.0)
        self.declare_parameter("speed_filter_alpha", 0.3)
        self.declare_parameter("watchdog_timeout", 0.3)
        self.declare_parameter("center_steer_on_start", True)

    def _load_parameters(self):
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.estop_topic = self.get_parameter("estop_topic").value
        self.wheel_state_topic = self.get_parameter(
            "wheel_state_topic").value
        self.backend_name = self.get_parameter("backend").value
        self.serial_port = self.get_parameter("serial_port").value
        self.car_type = int(self.get_parameter("car_type").value)
        self.track_width = float(self.get_parameter("track_width").value)
        self.max_speed = float(self.get_parameter("max_speed_mps").value)
        self.max_omega = float(self.get_parameter("max_omega_radps").value)
        self.cmd_per_mps = float(self.get_parameter("cmd_per_mps").value)
        left_cmd_per_mps = float(
            self.get_parameter("left_cmd_per_mps").value)
        right_cmd_per_mps = float(
            self.get_parameter("right_cmd_per_mps").value)
        self.left_cmd_per_mps = (
            left_cmd_per_mps if left_cmd_per_mps > 0.0
            else self.cmd_per_mps)
        self.right_cmd_per_mps = (
            right_cmd_per_mps if right_cmd_per_mps > 0.0
            else self.cmd_per_mps)
        self.left_motor = int(self.get_parameter("left_motor").value)
        self.right_motor = int(self.get_parameter("right_motor").value)
        self.left_encoder = int(self.get_parameter("left_encoder").value)
        self.right_encoder = int(self.get_parameter("right_encoder").value)
        self.left_encoder_sign = float(
            self.get_parameter("left_encoder_sign").value)
        self.right_encoder_sign = float(
            self.get_parameter("right_encoder_sign").value)
        self.encoder_ticks_per_meter = float(
            self.get_parameter("encoder_ticks_per_meter").value)
        self.encoder_window_size = int(
            self.get_parameter("encoder_window_size").value)
        self.use_feedback = bool(
            self.get_parameter("use_encoder_feedback").value)
        self.pid_kp = float(self.get_parameter("pid_kp").value)
        self.pid_ki = float(self.get_parameter("pid_ki").value)
        self.pid_kd = float(self.get_parameter("pid_kd").value)
        self.pid_integral_limit = float(
            self.get_parameter("pid_integral_limit").value)
        self.pid_output_limit = float(
            self.get_parameter("pid_output_limit").value)
        self.speed_filter_alpha = float(
            self.get_parameter("speed_filter_alpha").value)
        self.speed_filter_alpha = max(
            0.01, min(1.0, self.speed_filter_alpha))
        self.watchdog_timeout = float(
            self.get_parameter("watchdog_timeout").value)
        self.center_steer = bool(
            self.get_parameter("center_steer_on_start").value)

    def _cmd_callback(self, msg):
        if not self.ready or self.estop:
            return
        linear_x = clamp(float(msg.linear.x),
                         -self.max_speed, self.max_speed)
        angular_z = clamp(float(msg.angular.z),
                          -self.max_omega, self.max_omega)
        left_mps, right_mps = twist_to_wheel_speeds(
            linear_x, angular_z, self.track_width)
        self.target_left_mps = left_mps
        self.target_right_mps = right_mps
        self.last_cmd_mono = time.monotonic()

        if not self.use_feedback:
            self._send_wheel_targets()
        self.cmd_count += 1
        if self.cmd_count % 25 == 1:
            self.get_logger().info(
                "cmd v=%.2f w=%.2f -> left=%.2f right=%.2f m/s"
                % (linear_x, angular_z, left_mps, right_mps))

    def _estop_callback(self, msg):
        self.estop = bool(msg.data)

    def _tick(self):
        now = time.monotonic()
        dt = clamp(now - self.last_tick_mono, 0.005, 0.1)
        self.last_tick_mono = now

        age = now - self.last_cmd_mono
        force_stop = self.estop or age > self.watchdog_timeout
        if force_stop:
            self.target_left_mps = 0.0
            self.target_right_mps = 0.0
            self.left_pid.reset()
            self.right_pid.reset()
            if self.observer is not None:
                self.observer.reset()

        if not self.ready:
            return

        # 当前走的其实是"两条分支"：
        #
        # 1) use_feedback=False（现值，开环）：
        #    cmd → 限幅 → 差速运动学 → target轮速 → cmd_per_mps → 直接下发。
        #    完全没看实际轮速，相当于"小区开出门就踩油门，不看是不是打滑"。
        #
        # 2) use_feedback=True（闭环，需先标定 encoder_ticks_per_meter 和 PID）
        #    在开环基础上多了一环：
        #    read_encoders → EncoderObserver算出 actual轮速 → 和目标比出误差 →
        #    PID 修正 → 下发。这才是真正的"编码器闭环/速度反馈"。
        #
        # 所以答案：现在确实还是"限幅→差速运动学→下发→看门狗+日志"的开环。
        # 离闭环还差的步骤(见 README)：
        #   a. 标定 encoder_ticks_per_meter(每米多少 tick)；
        #   b. 确认电机/编码器通道和方向符号；
        #   c. 调好 PID 增益(kp/ki/kd)。
        # 三者到位后把 use_encoder_feedback 改成 true 即进入闭环。
        if self.use_feedback:
            encoder_values = self.backend.read_encoders()
            speeds = None
            if self.observer is not None:
                speeds = self.observer.update(encoder_values, dt)
            if speeds is not None:
                self.actual_left_mps = (
                    self.speed_filter_alpha * speeds[0]
                    + (1.0 - self.speed_filter_alpha) * self.actual_left_mps)
                self.actual_right_mps = (
                    self.speed_filter_alpha * speeds[1]
                    + (1.0 - self.speed_filter_alpha) * self.actual_right_mps)
                self._run_feedback_tick(dt, speeds[0], speeds[1])
            else:
                self._send_wheel_targets()
        else:
            self._send_wheel_targets()

        self._publish_state(encoder_values if self.use_feedback else [0, 0, 0, 0])

    def _run_feedback_tick(self, dt, raw_left_mps, raw_right_mps):
        left_ff = self.target_left_mps * self.left_cmd_per_mps
        right_ff = self.target_right_mps * self.right_cmd_per_mps
        left_err = left_ff - raw_left_mps * self.left_cmd_per_mps
        right_err = right_ff - raw_right_mps * self.right_cmd_per_mps
        left_raw = left_ff + self.left_pid.update(left_err, dt)
        right_raw = right_ff + self.right_pid.update(right_err, dt)
        self._send_wheel_commands(left_raw, right_raw)
        self.fb_count += 1
        if self.fb_count % 25 == 1:
            self.get_logger().info(
                "fb left target=%.2f actual=%.2f raw=%.1f | "
                "right target=%.2f actual=%.2f raw=%.1f"
                % (self.target_left_mps, self.actual_left_mps, left_raw,
                   self.target_right_mps, self.actual_right_mps, right_raw))

    def _send_wheel_targets(self):
        left_raw = clamp(self.target_left_mps * self.left_cmd_per_mps,
                         -100.0, 100.0)
        right_raw = clamp(self.target_right_mps * self.right_cmd_per_mps,
                          -100.0, 100.0)
        self._send_wheel_commands(left_raw, right_raw)

    def _send_wheel_commands(self, left_raw, right_raw):
        left_raw = clamp(left_raw, -100.0, 100.0)
        right_raw = clamp(right_raw, -100.0, 100.0)
        self.last_raw_left = left_raw
        self.last_raw_right = right_raw
        self.backend.send_motor(
            self.left_motor, self.right_motor, left_raw, right_raw)

    def _publish_state(self, encoder_values):
        msg = WheelState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.left_target_mps = self.target_left_mps
        msg.right_target_mps = self.target_right_mps
        msg.left_actual_mps = self.actual_left_mps
        msg.right_actual_mps = self.actual_right_mps
        msg.left_raw_cmd = self.last_raw_left
        msg.right_raw_cmd = self.last_raw_right
        if len(encoder_values) >= max(self.left_encoder, self.right_encoder):
            msg.left_encoder_ticks = encoder_values[self.left_encoder - 1]
            msg.right_encoder_ticks = encoder_values[self.right_encoder - 1]
        self.wheel_state_pub.publish(msg)


def main():
    rclpy.init()
    node = DiffDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ready:
            node._send_wheel_commands(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
