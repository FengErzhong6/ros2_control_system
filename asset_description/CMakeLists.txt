cmake_minimum_required(VERSION 3.10)
project(asset_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY wuji/urdf wuji/meshes wuji/rviz
  DESTINATION share/${PROJECT_NAME}/wuji
)

ament_package()
