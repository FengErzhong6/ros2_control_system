cmake_minimum_required(VERSION 3.8)
project(marvin_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  visualization_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

set(MARVIN_SDK_LIBRARY
  "${PROJECT_SOURCE_DIR}/third_party/marvinSDK/libMarvinSDK.so"
)
set(OMNIPICKER_SDK_LIBRARY
  "${PROJECT_SOURCE_DIR}/third_party/omnipickerSDK/libomnipicker.so"
)
set(MARVIN_KINE_LIBRARY
  "${PROJECT_SOURCE_DIR}/third_party/marvinKine/libKine.so"
)

add_library(
  marvin_system_hardware
  SHARED
  hardware/src/marvin.cpp
  hardware/src/workspace_guard.cpp
)
target_compile_features(marvin_system_hardware PUBLIC cxx_std_17)
target_include_directories(marvin_system_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(marvin_system_hardware PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(marvin_system_hardware PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${MARVIN_SDK_LIBRARY}
  ${OMNIPICKER_SDK_LIBRARY}
)
set_target_properties(marvin_system_hardware PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)

pluginlib_export_plugin_description_file(
  hardware_interface
  marvin_system_plugins.xml
)

# IK Controller plugin
add_library(
  marvin_system_ik_controller
  SHARED
  controller/src/ik/ik_controller.cpp
)
target_compile_features(marvin_system_ik_controller PUBLIC cxx_std_17)
target_include_directories(marvin_system_ik_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/third_party/marvinKine>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(marvin_system_ik_controller PUBLIC
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  std_msgs
)
target_link_libraries(marvin_system_ik_controller PUBLIC
  controller_interface::controller_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${MARVIN_KINE_LIBRARY}
)
set_target_properties(marvin_system_ik_controller PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)

pluginlib_export_plugin_description_file(
  controller_interface
  marvin_system_controller_plugins.xml
)

# Tracker Teleop Controller plugin
add_library(
  marvin_system_tracker_teleop_controller
  SHARED
  controller/src/tracker_teleop/controller.cpp
  controller/src/tracker_teleop/config.cpp
  controller/src/tracker_teleop/tracking.cpp
  controller/src/tracker_teleop/runtime.cpp
  controller/src/tracker_teleop/diagnostics.cpp
)
target_compile_features(marvin_system_tracker_teleop_controller PUBLIC cxx_std_17)
target_include_directories(marvin_system_tracker_teleop_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/third_party/marvinKine>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(marvin_system_tracker_teleop_controller PUBLIC
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  visualization_msgs
)
target_link_libraries(marvin_system_tracker_teleop_controller PUBLIC
  controller_interface::controller_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${MARVIN_KINE_LIBRARY}
)
set_target_properties(marvin_system_tracker_teleop_controller PROPERTIES
  INSTALL_RPATH "$ORIGIN"
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

add_executable(joy_to_pose
  bringup/src/joy_to_pose.cpp
)
target_compile_features(joy_to_pose PUBLIC cxx_std_17)
ament_target_dependencies(joy_to_pose
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

add_executable(ik_solution_explorer
  test/ik_solution_explorer.cpp
)
target_compile_features(ik_solution_explorer PUBLIC cxx_std_17)
target_include_directories(ik_solution_explorer PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/third_party/marvinKine>
)
target_link_libraries(ik_solution_explorer PUBLIC
  ${MARVIN_KINE_LIBRARY}
)
set_target_properties(ik_solution_explorer PROPERTIES
  INSTALL_RPATH "$ORIGIN/.."
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_add_gtest(workspace_guard_test
    test/workspace_guard_test.cpp
    hardware/src/workspace_guard.cpp
  )
  target_include_directories(workspace_guard_test PRIVATE
    ${PROJECT_SOURCE_DIR}/hardware/include
  )
  ament_target_dependencies(workspace_guard_test
    rclcpp
  )

  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
  PATTERN "scripts/*.py" EXCLUDE
)

install(
  PROGRAMS
    bringup/scripts/tracker_teleop_keyboard.py
    description/scripts/joint_gui_publisher.py
    description/scripts/vector_marker_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY bringup/config bringup/launch
  DESTINATION share/${PROJECT_NAME}/bringup
)

install(
  DIRECTORY
    hardware/include/marvin_system
    controller/include/marvin_system
  DESTINATION include
)

install(
  FILES marvin_system_plugins.xml marvin_system_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY third_party/marvinCfg/
  DESTINATION share/${PROJECT_NAME}/config/marvinCfg
)

install(
  FILES ${MARVIN_SDK_LIBRARY}
  DESTINATION lib
)

install(
  FILES
    ${OMNIPICKER_SDK_LIBRARY}
    third_party/omnipickerSDK/libomnipicker.so.0
    third_party/omnipickerSDK/libomnipicker.so.0.1.0
  DESTINATION lib
)

install(
  FILES ${MARVIN_KINE_LIBRARY}
  DESTINATION lib
)

install(
  TARGETS marvin_system_hardware marvin_system_ik_controller marvin_system_tracker_teleop_controller
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS gui_joint_state_to_forward_command joy_to_pose ik_solution_explorer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
