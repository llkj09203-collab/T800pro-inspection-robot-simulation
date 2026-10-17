cmake_minimum_required(VERSION 3.8)
project(t800_runtime_environment)

find_package(ament_cmake REQUIRED)
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/cyclonedds_participants.sh.in")
ament_package()
