cmake_minimum_required(VERSION 3.8)
project(openarm_env_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# 安装 urdf 目录
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# 可选：安装 meshes 和 rviz（如果存在）
install(
  DIRECTORY meshes rviz mujoco
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
