cmake_minimum_required(VERSION 3.16)
project(ros2_control_wujihand LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
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
  ros2_control_wujihand
  SHARED
  hardware/wujihand.cpp
)
target_compile_features(ros2_control_wujihand PUBLIC cxx_std_17)
target_include_directories(ros2_control_wujihand PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/ros2_control_wujihand>
)
target_link_libraries(ros2_control_wujihand PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${WUJIHANDCPP_TARGET}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(
  hardware_interface
  ros2_control_wujihand_plugins.xml
)

# INSTALL
install(
  DIRECTORY hardware/include 
  DESTINATION include/${PROJECT_NAME}
)
install(
	DIRECTORY description/launch description/ros2_control description/urdf
	DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY bringup/config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS ros2_control_wujihand
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
