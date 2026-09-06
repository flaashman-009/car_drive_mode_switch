# R2 Software Differential Drive Framework

Prototype workspace for switching a Yahboom ROSMaster R2 Ackermann chassis into
a software differential-drive mode.

## Status

- Custom ROS2 interfaces: ready.
- ROS2 node/launch/config framework: ready.
- Motor channels, encoder channels, `ticks_per_meter` and PID gains: not yet
  calibrated on the vehicle (chassis USB was offline when this framework was
  created).
- Runtime process handoff between Ackermann and differential modes: interface
  is defined; process supervision is not implemented yet.

## Layout

```text
.
├── docs/                        # hardware notes and mode design notes
├── legacy/                      # earlier standalone prototypes
└── src/
    ├── r2_diff_msgs/             # custom msg/srv package (ament_cmake)
    │   ├── CMakeLists.txt
    │   ├── msg/
    │   └── srv/
    └── r2_diff_driver/           # driver/mode-manager package (ament_python)
        ├── config/               # ROS2 parameter YAML
        ├── launch/               # ROS2 launch files
        ├── r2_diff_driver/       # Python modules
        └── test/
```

## Build

Run this from the workspace root (the folder containing `src/`) on a machine
with ROS2 Humble and `colcon`:

```bash
colcon build --packages-select r2_diff_msgs r2_diff_driver
source install/setup.bash
```

## Quick start without hardware

```bash
ros2 launch r2_diff_driver differential_drive.launch.py backend:=mock
```

`backend:=mock` lets the node tree run without opening `/dev/myserial`. Publish
a Twist on `/cmd_vel` to see the target wheel speeds in the logs.

## Quick start on the vehicle (after calibration)

```bash
ros2 launch r2_diff_driver differential_drive.launch.py backend:=rosmaster
```

Before that, edit `src/r2_diff_driver/config/differential.yaml` and run the
bench calibration. ROS2 launch and parameter paths are resolved from
`install/` after `colcon build`.

## Services and topics

- Service `/set_drive_mode`:
  `r2_diff_msgs/srv/SetDriveMode`, mode is `ackermann` or `differential`.
- Topic `/drive_mode_state`: `r2_diff_msgs/msg/ModeState`.
- Topic `/r2_diff/wheel_state`: `r2_diff_msgs/msg/WheelState`.

Example service call:

```bash
ros2 service call /set_drive_mode r2_diff_msgs/srv/SetDriveMode "{mode: differential}"
```

## Replace when the chassis is available again

- Motor/encoder channel mapping from the bench test.
- `encoder_ticks_per_meter` and `cmd_per_mps` from calibration.
- PID gains from lifted-wheel step response.
- Mode-manager process handoff in
  `src/r2_diff_driver/r2_diff_driver/mode_manager_node.py`.
- Confirm `RosmasterMotorBackend` constructor arguments against the
  `Rosmaster_Lib.py` installed on the vehicle.
