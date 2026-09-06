#!/usr/bin/env python3
"""R2 software differential-drive node.

Converts a generic ROS Twist (/cmd_vel, linear.x and angular.z) into
independent rear-wheel motor commands through Rosmaster_Lib.set_motor().
The front Ackermann steering servo is held at center (0 deg).

Channel mapping (left_motor/right_motor) and raw-command-to-m/s scale
(cmd_per_mps) are vehicle-specific. Run r2_diff_bench_test.py first with the
rear wheels lifted, then fill these ROS parameters.
"""

# 本项目逻辑（数据流：上层 /cmd_vel -> 差速运动学 -> set_motor 串口帧 -> STM32 -> 电机）：
#   1. 这是一个 ROS2(Humble) 节点，订阅上层通用速度指令话题 /cmd_vel（Twist）。
#      linear.x = 车速 m/s，angular.z = 角速度 rad/s。
#   2. 用差速模型把 (v, ω) 拆成左右后轮各自的"速度指令"：
#          v_left  = v - ω·track/2
#          v_right = v + ω·track/2
#   3. 乘以 cmd_per_mps（底层原始指令 -> m/s 的换算系数），截断到 [-100,100]，
#      通过 Rosmaster_Lib.set_motor() 只给 left_motor/right_motor 两个通道发指令。
#   4. 前轮转向舵机 set_akm_steering_angle(0) 保持居中，本模式不做转向。
#   风险：R2 前轮不是万向轮，原地转会有轮胎侧滑/磨损，仅适合低速。
#   注意：set_motor 直接操作底盘串口，不能和原厂 Ackman_driver_R2 同时跑（串口冲突）。

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool
from Rosmaster_Lib import Rosmaster


class R2DiffDriver(Node):
    def __init__(self):
        super().__init__("r2_diff_driver")

        # ---- 声明 ROS2 参数，可在启动时用 -p 覆盖（--ros-args -p 键:=值）----
        self.declare_parameter("cmd_topic", "/cmd_vel")        # 上层速度指令话题
        self.declare_parameter("estop_topic", "/estop")        # 急停话题（Bool）
        self.declare_parameter("track_width", 0.1646)          # 后轮轮距 d（米）
        self.declare_parameter("max_speed_mps", 0.5)           # 允许的最大车速（m/s）
        self.declare_parameter("max_omega_radps", 2.0)         # 允许的最大角速度（rad/s）
        self.declare_parameter("cmd_per_mps", 100.0)           # 指令/m/s 换算系数（标定得出）
        # 官方通道顺序：M1 左前 / M2 左后 / M3 右前 / M4 右后
        # 软件差速只用后轮：left_motor=2(M2 左后)、right_motor=4(M4 右后)，
        # 但 set_motor() 在 R2 固件上是否真生效，仍须悬空跑 bench 实测确认。
        self.declare_parameter("left_motor", 2)                # 左后轮对应 set_motor 通道(1~4)
        self.declare_parameter("right_motor", 4)               # 右后轮对应 set_motor 通道(1~4)
        self.declare_parameter("watchdog_timeout", 0.3)        # 看门狗时间（秒），超时自动停
        self.declare_parameter("center_steer_on_start", True)  # 启动时是否把前轮舵机居中

        # ---- 读出参数并转成 Python 类型 ----
        cmd_topic = self.get_parameter("cmd_topic").value
        estop_topic = self.get_parameter("estop_topic").value
        self.track = float(self.get_parameter("track_width").value)
        self.max_speed = float(self.get_parameter("max_speed_mps").value)
        self.max_omega = float(self.get_parameter("max_omega_radps").value)
        self.cmd_per_mps = float(self.get_parameter("cmd_per_mps").value)
        self.left_motor = int(self.get_parameter("left_motor").value)
        self.right_motor = int(self.get_parameter("right_motor").value)
        self.watchdog = float(self.get_parameter("watchdog_timeout").value)
        self.center_steer = bool(self.get_parameter("center_steer_on_start").value)

        # ---- 链接底层底盘（默认 /dev/myserial），启动后台接收线程 ----
        self.car = Rosmaster()
        self.car.set_car_type(5)          # 车型=R2
        self.car.create_receive_threading()  # 后台线程解析底盘上传数据

        # ---- 差速模式下把前轮转向舵机打到 0° 居中，避免前轮乱转 ----
        if self.center_steer:
            try:
                self.car.set_akm_steering_angle(0.0, ctrl_car=False)
                self.get_logger().info("steering centered for diff mode")
            except Exception as exc:
                self.get_logger().error("center steering failed: %s" % exc)

        # ---- 订阅上层指令（/cmd_vel）和急停（/estop），并建立定时器 ----
        self.sub_cmd = self.create_subscription(Twist, cmd_topic, self.cmd_cb, 10)
        self.sub_estop = self.create_subscription(Bool, estop_topic, self.estop_cb, 10)
        self.last_cmd = time.time()   # 最后一次收到指令的时间，看门狗用
        self.estop = False            # 急停标志
        self._count = 0               # 收到的指令计数（用于节流打印日志）
        # 20Hz 定时器，每分钟(0.05*25=1.25s)在无新指令或急停时强制停车
        self.timer = self.create_timer(0.05, self.tick)

        self.get_logger().info(
            "diff mode ready: track=%.4f m left_motor=%d right_motor=%d "
            "cmd_per_mps=%.1f" % (self.track, self.left_motor,
                                  self.right_motor, self.cmd_per_mps))

    def _send(self, left_raw, right_raw):
        """把左右轮原始指令（-100~100）写入对应通道并下发到底盘。

        left_raw/right_raw: 可能是浮点，这里先限幅到 [-100,100] 再取整。
        vals 是 4 路 set_motor 的列表，只填充 left_motor/right_motor 通道，
        其余通道保持 0（停止），避免干扰其他电机。
        """
        vals = [0, 0, 0, 0]
        vals[self.left_motor - 1] = int(round(max(-100.0, min(100.0, left_raw))))
        vals[self.right_motor - 1] = int(round(max(-100.0, min(100.0, right_raw))))
        # 底层调用：打包成串口帧发给 STM32，STM32 再驱动对应电机
        self.car.set_motor(vals[0], vals[1], vals[2], vals[3])

    def cmd_cb(self, msg):
        """/cmd_vel 回调：把 (v, ω) 转成左右轮指令并下发。"""
        # 限幅，防止超出物理允许的最高车速/角速度
        v = max(-self.max_speed, min(self.max_speed, float(msg.linear.x)))
        omega = max(-self.max_omega, min(self.max_omega, float(msg.angular.z)))
        # 差速运动学：转弯时外侧轮快、内侧轮慢
        v_left = (v - omega * self.track / 2.0) * self.cmd_per_mps
        v_right = (v + omega * self.track / 2.0) * self.cmd_per_mps
        self._send(v_left, v_right)
        self.last_cmd = time.time()   # 刷新看门狗时间戳
        self._count += 1
        # 节流打印：每 25 条左右打印一次，避免刷屏
        if self._count % 25 == 1:
            self.get_logger().info(
                "cmd v=%.2f w=%.2f -> left=%.1f right=%.1f"
                % (v, omega, v_left, v_right))

    def estop_cb(self, msg):
        """/estop 回调：收到 True 置急停标志，定时器会立即把车停下。"""
        self.estop = bool(msg.data)

    def tick(self):
        """看门狗定时器（20Hz）。急停或超时无新指令时强制停车。"""
        age = time.time() - self.last_cmd
        if self.estop or age > self.watchdog:
            self._send(0.0, 0.0)   # 左右轮指令清零 -> 停止


def main():
    rclpy.init()                     # 初始化 ROS2
    node = R2DiffDriver()            # 创建节点
    try:
        rclpy.spin(node)             # 阻塞运行，循环处理订阅回调
    except KeyboardInterrupt:
        pass                         # Ctrl+C 退出
    finally:
        node._send(0.0, 0.0)         # 退出前确保电机停止
        node.destroy_node()
        rclpy.shutdown()             # 干净地关闭 ROS2


if __name__ == "__main__":
    main()
