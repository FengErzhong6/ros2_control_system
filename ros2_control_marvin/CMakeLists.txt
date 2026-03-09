cmake_minimum_required(VERSION 3.8)
project(ros2_control_marvin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  ros2_control_marvin_hardware
  SHARED
  hardware/src/marvin.cpp
)
target_compile_features(ros2_control_marvin_hardware PUBLIC cxx_std_17)
target_include_directories(ros2_control_marvin_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(ros2_control_marvin_hardware PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${PROJECT_SOURCE_DIR}/third_party/marvinSDK/libMarvinSDK.so
)
set_target_properties(ros2_control_marvin_hardware PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)

pluginlib_export_plugin_description_file(
  hardware_interface
  ros2_control_marvin_plugins.xml
)

add_executable(gui_joint_state_to_forward_command
  bringup/src/gui_joint_state_to_forward_command.cpp
)
target_compile_features(gui_joint_state_to_forward_command PUBLIC cxx_std_17)
ament_target_dependencies(gui_joint_state_to_forward_command
  rclcpp
  sensor_msgs
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
)

install(
  DIRECTORY bringup/config bringup/launch
  DESTINATION share/${PROJECT_NAME}/bringup
)

install(
  DIRECTORY hardware/include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  FILES ros2_control_marvin_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES third_party/marvinSDK/libMarvinSDK.so
  DESTINATION lib
)

install(
  TARGETS ros2_control_marvin_hardware
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS gui_joint_state_to_forward_command
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
