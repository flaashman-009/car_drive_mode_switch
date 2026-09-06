"""mode_manager.launch.py —— 只起模式管理节点。

作用：单独启动 r2_mode_manager，提供 /set_drive_mode 服务和 /drive_mode_state 话题。
正常情况下一般用 differential_drive.launch.py(会连带起 driver)，
这个只在你想单独验证/调试模式切换逻辑时才用到。

实现：generate_launch_description() 返回 LaunchDescription，只包含
   到"r2_mode_manager"一个 Node，参数取 config/differential.yaml 的
   [r2_mode_manager] 段，并用命令行 arg `mode` 覆盖 initial_mode。
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

    mode = LaunchConfiguration("mode")

    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="differential",
            description="Initial drive mode reported by r2_mode_manager.",
        ),
        Node(
            package="r2_diff_driver",
            executable="mode_manager",
            name="r2_mode_manager",
            parameters=[params_file, {"initial_mode": mode}],
            output="screen",
        ),
    ])
