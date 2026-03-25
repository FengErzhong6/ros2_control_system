cmake_minimum_required(VERSION 3.16)
project(wujihand_system LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(onnxruntime REQUIRED)

# wujihandcpp (system library may not ship a CMake package config)
find_package(wujihandcpp QUIET)
if(TARGET wujihandcpp::wujihandcpp)
  set(WUJIHANDCPP_TARGET wujihandcpp::wujihandcpp)
else()
  find_path(WUJIHANDCPP_INCLUDE_DIR
    NAMES wujihandcpp/device/hand.hpp
  )
  find_library(WUJIHANDCPP_LIBRARY
    NAMES wujihandcpp
  )
  if(NOT WUJIHANDCPP_INCLUDE_DIR OR NOT WUJIHANDCPP_LIBRARY)
    message(FATAL_ERROR "Could not find wujihandcpp headers or library. Install the SDK dev package (headers + libwujihandcpp.so).")
  endif()

  add_library(wujihandcpp::wujihandcpp UNKNOWN IMPORTED)
  set_target_properties(wujihandcpp::wujihandcpp PROPERTIES
    IMPORTED_LOCATION "${WUJIHANDCPP_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${WUJIHANDCPP_INCLUDE_DIR}"
  )
  set(WUJIHANDCPP_TARGET wujihandcpp::wujihandcpp)
endif()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  wujihand_system
  SHARED
  hardware/wujihand.cpp
)
target_compile_features(wujihand_system PUBLIC cxx_std_17)
target_include_directories(wujihand_system PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/wujihand_system>
)
target_link_libraries(wujihand_system PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${WUJIHANDCPP_TARGET}
)

# Controller plugin
add_library(
  wujihand_system_rl_controller
  SHARED
  controller/rl_controller.cpp
  controller/rl_policy.cpp
)
target_compile_features(wujihand_system_rl_controller PUBLIC cxx_std_17)
target_include_directories(wujihand_system_rl_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
  $<INSTALL_INTERFACE:include/wujihand_system>
)
target_link_libraries(wujihand_system_rl_controller PUBLIC
  controller_interface::controller_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  onnxruntime::onnxruntime
)

# Export hardware plugins
pluginlib_export_plugin_description_file(
  hardware_interface
  wujihand_system_plugins.xml
)

pluginlib_export_plugin_description_file(
  controller_interface
  wujihand_system_controller_plugins.xml
)

# INSTALL
install(
  DIRECTORY
    hardware/include/wujihand_system
    controller/include/wujihand_system
  DESTINATION include
)
install(
	DIRECTORY description/launch description/ros2_control description/urdf
	DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY bringup/config bringup/launch policy
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS wujihand_system wujihand_system_rl_controller
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
