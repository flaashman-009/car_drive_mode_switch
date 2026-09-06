cmake_minimum_required(VERSION 3.8)
project(r2_diff_msgs)

# 这个包是 ament_cmake(C++) + rosidl 接口生成。
# 它只负责把 msg/ 和 srv/ 里的文本定义"编译生成"成 Python/C++ 都能用的类，
# 本身不承载业务逻辑。因为要生成一堆跨语言的代码，所以必须用 CMake 编译，
# 这就是"接口包要用 C++ 方式构建"的原因(哪怕业务代码是 Python)。

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ModeState.msg"
  "msg/WheelState.msg"
  "srv/SetDriveMode.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
