#!/usr/bin/env python3
"""R2 differential-mode bench test.

Purpose:
1. Discover which set_motor channel drives which rear wheel / encoder.
2. Check direction convention.
3. Estimate cmd_per_mps for r2_diff_driver.

SAFETY: keep the rear wheels lifted off the ground for the whole test.
"""

# 本项目逻辑：
#   R2 原本是阿克曼底盘的 ROS 车（前轮舵机转向 + 后轮驱动）。
#   这个脚本想把它"软件当成差速车"用：通过底层 Rosmaster_Lib.set_motor()
#   单独控制每个电机口。但 set_motor 的 4 个通道分别对应哪个物理轮子、
#   正负号代表前进还是后退，出厂没统一文档，所以需要悬空标定。
#   标定结果填到 r2_diff_driver.py 的 ROS 参数里。

import time

from Rosmaster_Lib import Rosmaster


# 标定用的固定指令值（set_motor 的指令范围是 -100 ~ 100，25 约为 1/4 油门）
CMD = 25
RUN_SEC = 1.0        # 每个通道持续给电的时间（秒）
SETTLE_SEC = 0.5     # 每次发完命令后停下来的时间（秒），让编码器读数稳定


def enc_delta(before, after):
    """计算两次读取到的四路编码器计数的差值列表。

    before/after 都是长度 4 的列表（四路编码器计数）。
    返回 after - before 的列表，表示这段时间内每路编码器转了多少。
    """
    return [a - b for a, b in zip(after, before)]


def run_channel(car, index, cmd, seconds):
    """单独给某个电机通道施加速度指令，跑 seconds 秒后归零停止。

    car:   Rosmaster 小车实例
    index: 0..3，对应 set_motor 的第 1~4 个电机口
    cmd:   指令值，正负决定电机转向
    seconds: 持续运行时间
    """
    vals = [0, 0, 0, 0]        # 四路电机指令，默认全部 0（停止）
    vals[index] = cmd          # 只把当前要测的那个通道设为 cmd
    # 通过底层串口协议把四路电机指令发到底盘 STM32 控制板
    car.set_motor(vals[0], vals[1], vals[2], vals[3])
    time.sleep(seconds)        # 保持这个速度一段时间
    car.set_motor(0, 0, 0, 0)  # 全部归零，刹车/停止
    time.sleep(SETTLE_SEC)     # 等编码器停止跳动再读数


def main():
    # 打开底层库：car_type=5 表示 R2 车型，com 指定底盘串口设备
    car = Rosmaster(car_type=5, com="/dev/myserial")
    car.set_car_type(5)
    car.create_receive_threading()  # 启动后台收线程，实时解析底盘上报的数据
    try:
        # 先把前轮转向舵机打到 0° 居中（ctrl_car=False 只是摆舵机，不控制车运动）
        try:
            car.set_akm_steering_angle(0.0, ctrl_car=False)
        except Exception:
            pass  # 部分固件可能不支持该命令，失败就忽略

        print("=== channel scan: +%d for %.1fs each ===" % (CMD, RUN_SEC))
        # 依次给 4 个通道发 +25，观察每个通道转的是哪个轮子
        for i in range(4):
            car.set_motor(0, 0, 0, 0)   # 先全部停止
            time.sleep(SETTLE_SEC)
            before = car.get_motor_encoder()          # 记录这轮的编码器初值
            run_channel(car, i, CMD, RUN_SEC)         # 只转第 i+1 个通道
            after = car.get_motor_encoder()           # 记录结束时的编码器值
            print("motor channel %d: encoder deltas %s"
                  % (i + 1, enc_delta(before, after)))
            print("  -> observe which physical wheel turned; record it")

        print("=== direction scan: repeat channels with -%d ===" % CMD)
        # 再来一遍发 -25，确认负号对应哪个转向
        for i in range(4):
            car.set_motor(0, 0, 0, 0)
            time.sleep(SETTLE_SEC)
            before = car.get_motor_encoder()
            run_channel(car, i, -CMD, RUN_SEC)
            after = car.get_motor_encoder()
            print("motor channel %d reverse: encoder deltas %s"
                  % (i + 1, enc_delta(before, after)))
    finally:
        # 无论发生什么都不让小车继续转，保证安全
        car.set_motor(0, 0, 0, 0)
        print("motors stopped")


if __name__ == "__main__":
    main()
