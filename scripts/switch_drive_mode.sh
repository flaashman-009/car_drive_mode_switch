#!/usr/bin/env bash
# One-key drive mode switch on the R2 car.
#
# Usage:
#   ./switch_drive_mode.sh ackermann
#   ./switch_drive_mode.sh differential
#
# Both drivers share /dev/myserial, so only one may run at a time.

MODE="${1:-}"

stop_diff() {
  pkill -f "diff_driver" 2>/dev/null || true
  pkill -f "mode_manager" 2>/dev/null || true
  pkill -f "differential_drive.launch.py" 2>/dev/null || true
}

stop_ackermann() {
  pkill -f "Ackman_driver_R2" 2>/dev/null || true
  pkill -f "r2_driver_monitor.py" 2>/dev/null || true
  pkill -f "base_node_R2" 2>/dev/null || true
  pkill -f "ydlidar" 2>/dev/null || true
  pkill -f "ros2 launch" 2>/dev/null || true
}

case "${MODE}" in
  ackermann|ackerman)
    echo "stop differential mode"
    stop_diff
    sleep 1
    echo "start original Ackermann stack"
    ~/r2_start.sh
    ;;
  differential|diff)
    echo "stop Ackermann stack"
    stop_ackermann
    sleep 1
    echo "start differential mode"
    cd ~/r2_ws || exit 1
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    export ROS_DOMAIN_ID=28
    mkdir -p ~/r2_logs
    setsid nohup ros2 launch r2_diff_driver differential_drive.launch.py \
      backend:=rosmaster > ~/r2_logs/diff.log 2>&1 < /dev/null &
    sleep 2
    tail -n 20 ~/r2_logs/diff.log
    ;;
  *)
    echo "usage: $0 {ackermann|differential}"
    exit 1
    ;;
esac
