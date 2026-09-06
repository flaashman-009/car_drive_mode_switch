# Yahboom ROSMaster R2 实车信息汇总

> 本文档汇总截至 2026-09 在项目中对 ROSMaster R2（亚博小车）已知的硬件、软件、算法、实车测试与故障排查信息。
> 标注 `估算`、`待确认` 的项尚未完全实测，不要当作最终标定值。

## 1. 平台总览

| 项目 | 信息 |
|---|---|
| 产品 | Yahboom ROSMaster R2 阿克曼 ROS 小车 |
| 主控 | Jetson Orin NX SUPER 16GB（AArch64） |
| 系统 | Ubuntu 22.04.5 LTS，内核 5.15.148-tegra |
| ROS | ROS2 Humble |
| ROS_DOMAIN_ID | 28 |
| 主机名 | yahboom |
| SSH 用户/密码 | `jetson` / `yahboom` |
| 常用 IP | `192.168.43.10` |
| 车载型号标识 | `my_robot_type=r2`、`my_lidar=4ROS`、`my_camera=astraplus` |

## 2. 硬件清单

### 2.1 底盘与执行器

- Ackermann 结构，前轮转向、后轮驱动。
- 两个后轮为带测速码盘的 520 金属电机；前轮转向由金属数字舵机控制。
- 实车测试确认：运动中前轮编码器增量为 0（转向轮无驱动），后轮编码器提供里程。
- 底盘控制板（Rosmaster 板 / MCU）负责电机驱动、编码器、IMU、电池电压采集。
- 控制板通过 USB 串口（CH340，正常设备 ID 应为 `1a86:7523`）连接 Jetson，对应 `/dev/myserial`。

### 2.2 传感器

| 传感器 | 型号/标识 | 接口/话题 |
|---|---|---|
| LiDAR | YDLIDAR（驱动识别为 TG30） | `/dev/ydlidar` -> `/dev/ttyUSB0`，`/scan`，约 10Hz |
| 相机 | Orbbec Astra Pro Plus（彩色 + 深度） | `/dev/video0/1` |
| IMU | 底盘板载 9 轴 IMU | `/imu/data_raw`、`/imu/yaw_deg` |
| 里程计 | 后轮编码器，经 `base_node_R2` | `/odom_raw`；EKF 后为 `/odom` |
| 手柄 | DragonRise Controller | `/joy` -> `/cmd_vel` |

### 2.3 其他外设

- 7 寸触摸屏 / 屏幕输入设备（QDtech）。
- OLED 屏（`yahboom_oled.pyc` 常驻）。
- 语音模块：CH340 `1a86:7522`，对应 `/dev/myspeech`。
- 蓝牙/WiFi：Realtek 8822CE。

### 2.4 USB 拓扑

当前 USB Hub 层级（日志中的编号）：

```text
Jetson USB
└─ 1-2      VIA USB2.0 Hub
   └─ 1-2.4  5口 Hub
      ├─ 1-2.4.1  LiDAR CP210x  -> /dev/ttyUSB0 -> /dev/ydlidar
      ├─ 1-2.4.2  语音模块 CH340 -> /dev/ttyUSB1 -> /dev/myspeech
      ├─ 1-2.4.3  底盘控制板 CH340 -> 期望 /dev/myserial（当前枚举失败）
      ├─ 1-2.4.4  相机 Hub / Orbbec
      └─ 1-2.4.5  未知
```

> 注意：语音模块和底盘控制板都是 CH340，但 PID 不同。原厂规则为
> `7522 -> myspeech`、`7523 -> myserial`。

## 3. 尺寸与动力学参数

### 3.1 尺寸

| 参数 | 数值 | 来源 |
|---|---:|---|
| 车长 | 0.3375 m | 实测/规格 |
| 车宽 | 0.1911 m | 实测/规格 |
| 轴距 L | 0.2681 m | 实测 |
| 轮距 d | 0.1646 m | 实测 |
| 前轴到质心 lf | 0.134 m | 估算 |
| 后轴到质心 lr | 0.134 m | 估算 |
| 横摆惯量 Iz | 0.02 kg·m² | 估算 |
| 质量 m | 2.5 kg | 出厂标称 |
| 轮胎半径 | 0.0325 m | 估算 |
| 最大前轮转角 | 45° (0.7854 rad) | 实测 |
| 转向比 | 1.0 | 实测 |

### 3.2 动力学模型

实车算法使用**小尺寸 2DOF 自行车模型 + Pacejka 轮胎**：

```text
状态：x, y, ψ, vx, vy, ω
输入：前轮转角 δ、纵向速度 vx
```

侧偏角：

```text
α_f = δ - atan2(vy + lf·ω, vx)
α_r = -atan2(vy - lr·ω, vx)
```

运动方程：

```text
v̇y = (Fyf·cosδ + Fyr) / m - vx·ω
ω̇  = (lf·Fyf·cosδ - lr·Fyr) / Iz
ψ̇  = ω
ẋ   = vx·cosψ - vy·sinψ
ẏ   = vx·sinψ + vy·cosψ
```

Pacejka 默认参数：

```text
mu    = 0.85
pac_b0 = 15.0, b1 = 0.3, b2 = 1.3, b3 = 1.2,
b4..b9 = 0
```

### 3.3 实测动态特性

| 项目 | 数值 |
|---|---:|
| 首次速度响应延迟 | 0.11 s |
| 90% 上升时间 | 约 0.60 s |
| 指令 0.5 m/s 稳态 | 0.50–0.51 m/s |
| 最大加速度估算 | 约 0.7 m/s² |
| 转向零位偏置（最终） | -2.9° |
| 安全最高速度 | 1.8 m/s（测试常用 0.5） |

## 4. 软件架构

### 4.1 ROS2 工作区

- 原厂 ROS2 工作区：`~/yahboomcar_ros2_ws/`
- 自建消息：`~/vvc_ws/`（`vvc_msgs`）
- 闭环/部署脚本：`~/closed_loop/`、`~/closed_loop_fix/`
- 实车 MPC/CBF 控制器：`~/r2_stack/`

### 4.2 主要节点/话题

```text
Ackman_driver_R2 / driver_node   底盘串口驱动
base_node_R2                     里程计
imu_filter_madgwick              IMU 滤波
robot_localization EKF           里程计/IMU 融合
joint_state_publisher            关节状态
robot_state_publisher            TF
yahboom_joy_R2 + joy_node        PS2 手柄
slam_toolbox                     建图
nav2 / amcl / map_server         定位
rosbridge_server                 Foxglove
```

关键话题：

```text
/cmd_vel                 R2 底盘原生速度指令
/vel_raw                 底盘反馈
/odom_raw                原始里程
/odom                    EKF 融合里程
/scan                    LiDAR
/map /tf /tf_static      建图定位
/perceived_obstacles      障碍物列表（自定义）
/vvc_cmd_vel              VVC 控制器输出
/cmd_ackermann            Gazebo 仿真指令
/wheel_enc /wheel_enc_delta  自定义轮速发布
```

### 4.3 指令转换关系

`r2_full_controller` 发布 `/vvc_cmd_vel`：

```text
linear.x = 速度（m/s）
angular.z = 转向角（rad）
```

`r2_vel_adapter` 转成 R2 原生 `/cmd_vel`：

```text
linear.x = 速度（m/s）
linear.y = 转向角(deg) / 1000   # 30° -> 0.030
angular.z = 0
```

## 5. 算法栈

### 5.1 感知

- `r2_scan_to_obstacles.py`：`/scan` -> 过滤 -> 聚类 -> 障碍物列表 `/perceived_obstacles`。
- 相机目标检测：YOLO / TensorRT 节点框架，尚未完成与 LiDAR 的正式融合。

### 5.2 规划-决策-控制

```text
MPC（候选横向偏移轨迹 + 速度） -> CBF QP 安全滤波 -> PID -> 执行
```

- MPC：约 0.3s 重规划一次，候选偏移集合如 `[-0.6 ... 0.6]`。
- CBF：`h = d - r_obs - r_veh - margin`，QP 最小修正期望指令。
- 实车 CBF 参数：安全余量 `0.08 m`，`alpha=1.0`。
- 控制周期：控制器约 50Hz，执行器指令经 watchdog 兜底。

### 5.3 建图/定位

- `start_mapping.sh`：硬件 + SLAM Toolbox 在线建图。
- `start_localization.sh`：硬件 + Nav2 AMCL 定位。
- 地图保存：`~/closed_loop/save_map.sh <name>`。
- 已知问题：AMCL/EKF 参数已补丁，但实际定位漂移曾导致导航任务 ABORTED，需标定后验证。

## 6. 实车测试记录摘要

### 2026-08-27

- 实测轴距 `0.2681m`、轮距 `0.1646m`、最大转向约 `45°`。
- 空载/悬空轮速对照：左右后轮增量约 `187.3/187.5` ticks/0.1s，基本一致。
- 落地直线偏左，右后轮约快 1.6%–2.6%。
- 最终转向零位偏置标定为 `-2.9°`。

### 2026-08-28 / 08-29

- 0.5 m/s 直线 5m 闭环：odom 距离约 4.9–5.0m，IMU 终点航向约 0.01–0.02 rad。
- 纸箱避障：无碰撞、可绕行，但横向偏移偏大（odom y 曾到约 -1.25m）。
- LiDAR 感知、YOLO 框架、MPC/CBF 实车版均已部署验证。

## 7. 当前故障与排查结论

### 7.1 现象

- 2026-08-29 17:05 左右，使用 PS2 遥控时突然失控：后轮突然自行转动/向后跑，手柄与电脑指令均无效，只能断电。
- 断电重启后：底盘控制板连不上，`edition=-1`、`voltage=0`、`/dev/myserial` 不存在。

### 7.2 关键报错

驱动日志（`~/.ros/log/2026-08-29-16-36-21-.../launch.log`）：

```text
SerialException:
device reports readiness to read but returned no data
(device disconnected or multiple access on port?)
---
Rosmaster Serial Opened! Baudrate=115200
---set_car_motion error!---
```

内核日志（当前启动）：

```text
usb 1-2.4.3: Device not responding to setup address.
usb 1-2.4.3: device not accepting address 13, error -71
```

### 7.3 Windows 交叉验证

- 语音模块 `1a86:7522` 插 Windows：识别为 `USB-SERIAL CH340K (COM7)`。
- 底盘板 `1a86:7523` 插 Windows：识别不到。
- 结论倾向：底盘板 USB 芯片/线/供电存在硬件问题，不是 Jetson 或 ROS 软件问题。

### 7.4 已执行的软件排查

- 已恢复 `udev` 原厂规则（`7522->myspeech`、`7523->myserial`）。
- 已删除手动 `/dev/myserial` 软链。
- 已关闭 USB autosuspend，仍无法枚举。
- 未修改原厂 `Ackman_driver_R2`。

### 7.5 后续处理

- 优先更换底盘控制板到 Jetson 的数据 USB 线，并尝试直接插入 Jetson 不同 USB 口（绕过 Hub）。
- 检查控制板供电/排线。
- 若仍无法枚举，判断为控制板 USB 部分故障，联系厂商检修。

## 8. 易混淆/易冲突脚本

以下脚本都会打开同一个底盘串口，运行时只允许一个：

```text
~/Ackman_driver_R2（原厂驱动）
~/r2_driver_monitor.py
~/r2_center_steer.py
~/Rosmaster/rosmaster/rosmaster_main.py
~/Rosmaster/rosmaster/wifi_rosmaster.py
```

同时运行会导致 “multiple access on port” 类串口冲突。

## 9. 常用命令

```bash
# 建图
~/closed_loop_fix/start_mapping.sh

# 保存地图
~/closed_loop/save_map.sh environment_new

# 定位
~/closed_loop_fix/start_localization.sh

# rosbridge / Foxglove
~/closed_loop_fix/start_rosbridge.sh
# ws://192.168.43.10:9090

# 检查串口占用
fuser -v /dev/myserial
```

## 10. 待确认 / 风险

- 底盘控制板当前是否可修复、需要更换哪部分。
- 实车 `lf/lr`、`Iz` 为估算值，后续可做更细标定。
- Nav2 自主闭环尚未稳定验证（AMCL 漂移问题）。
- 相机目标检测与 LiDAR 障碍物融合尚未完成。
- 磁盘占用高（约 93%），需清理后继续录包/建图。
