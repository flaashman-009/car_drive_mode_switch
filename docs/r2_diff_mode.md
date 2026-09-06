# R2 软件差速模式

## 1. 目标

在 ROSMaster R2 阿克曼底盘上，用软件把后轮当独立驱动轮使用：

- 前轮舵机保持 0° 居中；
- 左右后轮分别通过 `Rosmaster_Lib.set_motor()` 下发速度；
- 上层接收通用 `/cmd_vel`（`linear.x` 为速度 m/s，`angular.z` 为角速度 rad/s）。

这不是真差速底盘，前轮不是万向轮，低速可用，原地转会有轮胎侧滑。

## 2. 运动学

输入：

```text
v   = linear.x  (m/s)
omega = angular.z (rad/s)
track = 0.1646 m
```

转换：

```text
v_left  = v - omega * track / 2
v_right = v + omega * track / 2
```

底层 `set_motor` 通道和 `cmd_per_mps` 需要先标定。

## 3. 文件

- `tools/r2_diff_driver.py`：差速驱动节点。
- `tools/r2_diff_bench_test.py`：悬空标定脚本。

## 4. 标定步骤（底盘板回来后执行）

### 4.1 安全准备

- 底盘 USB 正常，`/dev/myserial` 存在。
- **两个后轮必须悬空**。
- 前轮尽量悬空或保持地面不打紧。

### 4.2 通道扫描

```bash
python3 ~/r2_diff_bench_test.py
```

脚本会依次给电机通道 1..4 发 `+25` 和 `-25`，并打印四路编码器增量。

记录结果：

- 哪个通道让左后轮转？
- 哪个通道让右后轮转？
- 正指令是前进还是后退？
- 该通道对应的编码器索引。

### 4.3 填入参数

根据扫描结果运行驱动：

```bash
python3 -u ~/r2_diff_driver.py --ros-args \
  -p left_motor:=左轮通道 \
  -p right_motor:=右轮通道 \
  -p cmd_per_mps:=标定值
```

`cmd_per_mps` 初估值：若 `set_motor(100)` 实测轮速约 `0.5 m/s`，则 `cmd_per_mps≈200`。

### 4.4 地面低速验证

- 前轮居中；
- 先发 `v=0.2, omega=0` 直线；
- 再发 `v=0, omega=0.5` 看是否能原地偏转；
- 记录电流、抖动、轮胎磨损情况。

## 5. 模式切换

简单方案：两种模式对应两个驱动进程，一次只起一个。

```text
Ackermann 模式: r2_vel_adapter + Ackman_driver_R2
差速模式:      r2_diff_driver
```

后续若需要运行时切换，可在 `r2_diff_driver` 里增加 `/drive_mode` 话题或参数开关。

## 6. 风险与限制

- `set_motor()` 在 R2 固件上是否支持、通道顺序是什么，必须先悬空验证。
- 前轮不是万向轮，原地转阻力大，会加速轮胎磨损。
- 差速模式的里程模型需要换成：

```text
v = (v_right + v_left) / 2
omega = (v_right - v_left) / track
```

- 高层 MPC/CBF 如果按阿克曼转角输出，差速模式下需要改为输出速度 + 角速度。
- 差速模式和原厂 Ackman 驱动不能同时运行，避免串口冲突。
