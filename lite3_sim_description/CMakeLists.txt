cmake_minimum_required(VERSION 3.8)
project(lite3_sim_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

