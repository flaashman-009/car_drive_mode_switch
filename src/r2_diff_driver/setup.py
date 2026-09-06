"""r2_diff_driver 的构建脚本 (ament_python)。

什么叫 ament_python / setup.py / setup.cfg / package.xml？
- 一个 ROS2 包 = 代码 + 元数据(package.xml) + 一份"如何构建安装"的说明。
- 对纯 Python 写的包，那说明就是 setup.py(标准 Python 打包)。colcon 读到
  package.xml 里 <build_type>ament_python</build_type>，就知道用 Python 打包。
- setup.py 里的 data_files 负责把 launch/*.launch.py 和 config/*.yaml 等非 .py
  的资源文件一起拷进 install/ 目录，这样运行时才能找到它们。

C++ vs Python 在 ROS2 怎么分工：
- 需要性能/硬实时/底层硬件(电机、传感器、图像处理)→ C++(ament_cmake)。
- 需要快速开发/调参/胶水逻辑/控制流程 → Python(ament_python)。
- 本工程：r2_diff_msgs(接口)是编译型 → C++ 走 ament_cmake；
  r2_diff_driver(业务逻辑)用 Python，烧录快也方便标定改参。
"""

import os
from glob import glob

from setuptools import find_packages, setup


package_name = "r2_diff_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="r2 team",
    maintainer_email="r2@localhost",
    description="R2 software differential-drive node framework.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "diff_driver = r2_diff_driver.diff_driver_node:main",
            "mode_manager = r2_diff_driver.mode_manager_node:main",
            "bench_test = r2_diff_driver.bench_test:main",
        ],
    },
)
