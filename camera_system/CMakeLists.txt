cmake_minimum_required(VERSION 3.8)
project(camera_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(OrbbecSDK_DIR "/opt/OrbbecSDK_v2.7.2/lib")
find_package(OrbbecSDK REQUIRED CONFIG)

add_library(orbbec_camera_node_lib
  hardware/orbbec/src/orbbec_camera_node.cpp
)
target_compile_features(orbbec_camera_node_lib PUBLIC cxx_std_17)
target_include_directories(orbbec_camera_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware/orbbec/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(orbbec_camera_node_lib PUBLIC
  rclcpp
  sensor_msgs
)
target_link_libraries(orbbec_camera_node_lib PUBLIC ob::OrbbecSDK)
set_target_properties(orbbec_camera_node_lib PROPERTIES
  BUILD_RPATH "/opt/OrbbecSDK_v2.7.2/lib"
  INSTALL_RPATH "/opt/OrbbecSDK_v2.7.2/lib"
)

add_executable(orbbec_camera_node
  hardware/orbbec/src/orbbec_camera_main.cpp
)
target_compile_features(orbbec_camera_node PUBLIC cxx_std_17)
ament_target_dependencies(orbbec_camera_node PUBLIC
  rclcpp
)
target_link_libraries(orbbec_camera_node PRIVATE orbbec_camera_node_lib)

add_executable(print_orbbec_serials
  test/print_orbbec_serials.cpp
)
target_compile_features(print_orbbec_serials PUBLIC cxx_std_17)
target_link_libraries(print_orbbec_serials PRIVATE ob::OrbbecSDK)
set_target_properties(print_orbbec_serials PROPERTIES
  BUILD_RPATH "/opt/OrbbecSDK_v2.7.2/lib"
  INSTALL_RPATH "/opt/OrbbecSDK_v2.7.2/lib"
)

install(
  TARGETS
    orbbec_camera_node_lib
    orbbec_camera_node
    print_orbbec_serials
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY hardware/orbbec/include/
  DESTINATION include
)

install(
  DIRECTORY bringup/config bringup/launch
  DESTINATION share/${PROJECT_NAME}/bringup
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs)

ament_package()
