# R2 Software Differential Drive Framework

ROS2 framework for switching a Yahboom ROSMaster R2 Ackermann chassis into a
software differential-drive mode.

## Status

- Custom ROS2 interfaces: ready.
- ROS2 node/launch/config framework: ready.
- Motor/encoder channels and direction signs: measured on the vehicle.
- `cmd_per_mps` and `ticks_per_meter`: calibrated on the vehicle.
- Differential encoder closed loop: verified on the bench.
- Ackermann/differential process switching: verified on the vehicle with
  `scripts/switch_drive_mode.sh`.
- Ground differential turning: still open. The chassis is mechanically
  Ackermann; software differential mode is not a true differential chassis.

## Layout

```text
.
├── docs/                        # hardware notes and mode design notes
├── legacy/                      # earlier standalone prototypes
├── scripts/
│   └── switch_drive_mode.sh     # one-key Ackermann/differential switch
└── src/
    ├── r2_diff_msgs/            # custom msg/srv package (ament_cmake)
    │   ├── CMakeLists.txt
    │   ├── msg/
    │   └── srv/
    └── r2_diff_driver/          # driver/mode-manager package (ament_python)
        ├── config/
        ├── launch/
        ├── r2_diff_driver/
        └── test/
```

## Build

Run from the workspace root on a machine with ROS2 Humble and `colcon`:

```bash
colcon build --packages-select r2_diff_msgs r2_diff_driver
source install/setup.bash
```

## Quick start without hardware

```bash
ros2 launch r2_diff_driver differential_drive.launch.py backend:=mock
```

## Quick start on the vehicle

```bash
~/switch_drive_mode.sh differential
~/switch_drive_mode.sh ackermann
```

Manual differential launch:

```bash
cd ~/r2_ws
source install/setup.bash
ros2 launch r2_diff_driver differential_drive.launch.py backend:=rosmaster
```

## Services and topics

- Service `/set_drive_mode`:
  `r2_diff_msgs/srv/SetDriveMode`.
- Topic `/drive_mode_state`: `r2_diff_msgs/msg/ModeState`.
- Topic `/r2_diff/wheel_state`: `r2_diff_msgs/msg/WheelState`.

## PCD to PGM map filtering

`scripts/pcd_to_pgm.py` converts a PCD point cloud into a ROS-style
occupancy-grid PGM/YAML pair.

```bash
python3 scripts/pcd_to_pgm.py input.pcd ~/maps/room \
  --time-field timestamp \
  --max-age-ms 200 \
  --z-min -0.1 --z-max 1.5 \
  --resolution 0.05 \
  --voxel-size 0.03 \
  --outlier-radius 0.08 \
  --min-neighbors 3 \
  --min-points 1 \
  --inflate-cells 2
```

Filter order:

```text
timestamp freshness -> Z pass-through -> voxel downsample
-> radius outlier removal -> 2D projection -> obstacle inflation
-> PGM/YAML
```

If the PCD contains `t`, `time`, or `timestamp`, use `--time-field` to drop
stale points relative to the newest timestamp. The script also prints load and
total processing latency in milliseconds.

## Vehicle progress

Measured on the Yahboom R2:

- `left_motor=2`, `right_motor=4`
- `left_encoder=2`, `right_encoder=4`
- Positive PWM drives both rear wheels forward.
- Push calibration: left about 3912 ticks/m, right about 3891 ticks/m.
- PWM ramp calibration: `left_cmd_per_mps=45.5`,
  `right_cmd_per_mps=54.5`.
- Differential encoder closed loop tracks target wheel speed on the bench.
- Encoder speed is averaged over three control periods to avoid 0/2x
  sampling spikes caused by the 40 ms chassis report rate.

Both modes share `/dev/myserial`, so they must never run at the same time.
