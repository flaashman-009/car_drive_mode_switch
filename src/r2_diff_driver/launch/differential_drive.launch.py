"""differential_drive.launch.py —— 主用 launch。

作用：一次性把两个节点都拉起来，组成完整差速系统。
  - r2_diff_driver   负责收 /cmd_vel 驱动电机、发轮速状态；
  - r2_mode_manager  负责 /set_drive_mode 模式切换服务。
因此实际用车建议用这个。

与 mode_manager.launch.py 的区别：
  那个只起 r2_mode_manager(纯模式切换，不驱动电机)，适合只想测模式服务的场景。

原理：launch 文件其实就是一段 Python，ROS2 会执行 generate_launch_description()
返回的 LaunchDescription，里面声明了 launch 参数(backend/mode)和要启动的 Node。
参数合并规则：config/*.yaml 给默认值，再由 launch 命令行覆盖(如下面 backend)。

实际操作：
  ros2 launch r2_diff_driver differential_drive.launch.py backend:=mock
    ↑ 无真车，跑通逻辑
  ros2 launch r2_diff_driver differential_drive.launch.py backend:=rosmaster
    ↑ 真车(需先标定 + 接好 /dev/myserial)
注意：launch 是通过 install/ 下的拷贝找 config 和 launch 的，改代码要重新
colcon build 才生效。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("r2_diff_driver")
    params_file = os.path.join(package_dir, "config", "differential.yaml")

    backend = LaunchConfiguration("backend")
    mode = LaunchConfiguration("mode")

    return LaunchDescription([
        DeclareLaunchArgument(
            "backend",
            default_value="mock",
            description="Driver backend: mock for dry-run, rosmaster on the car.",
        ),
        DeclareLaunchArgument(
            "mode",
            default_value="differential",
            description="Initial mode reported by r2_mode_manager.",
        ),
        Node(
            package="r2_diff_driver",
            executable="diff_driver",
            name="r2_diff_driver",
            parameters=[params_file, {"backend": backend}],
            output="screen",
        ),
        Node(
            package="r2_diff_driver",
            executable="mode_manager",
            name="r2_mode_manager",
            parameters=[params_file, {"initial_mode": mode}],
            output="screen",
        ),
    ])
