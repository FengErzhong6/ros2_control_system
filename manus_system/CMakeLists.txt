cmake_minimum_required(VERSION 3.8)
project(manus_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(std_msgs REQUIRED)

set(MANUS_SDK_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/third_party/include")
set(MANUS_SDK_LIBRARY "${PROJECT_SOURCE_DIR}/third_party/lib/libManusSDK.so")

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ManusGloveRaw.msg"
  "msg/ManusGloveRawArray.msg"
  "msg/ManusRawNode.msg"
  "srv/CancelGloveCalibration.srv"
  "srv/FinishGloveCalibration.srv"
  "srv/ListGloves.srv"
  "srv/LoadGloveCalibration.srv"
  "srv/RunGloveCalibrationStep.srv"
  "srv/StartGloveCalibration.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(manus_calibration_node
  hardware/src/manus_calibration_node.cpp
)
target_compile_features(manus_calibration_node PUBLIC cxx_std_17)
target_compile_definitions(manus_calibration_node PRIVATE
  MANUS_SYSTEM_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)
target_include_directories(manus_calibration_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<BUILD_INTERFACE:${MANUS_SDK_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(manus_calibration_node PUBLIC
  rclcpp
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(manus_calibration_node
  PUBLIC
    ${MANUS_SDK_LIBRARY}
    ${cpp_typesupport_target}
)
set_target_properties(manus_calibration_node PROPERTIES
  BUILD_RPATH "${PROJECT_SOURCE_DIR}/third_party/lib"
  INSTALL_RPATH "$ORIGIN/.."
)

add_executable(manus_raw_publisher_node
  hardware/src/manus_raw_publisher_node.cpp
)
target_compile_features(manus_raw_publisher_node PUBLIC cxx_std_17)
target_compile_definitions(manus_raw_publisher_node PRIVATE
  MANUS_SYSTEM_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)
target_include_directories(manus_raw_publisher_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<BUILD_INTERFACE:${MANUS_SDK_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(manus_raw_publisher_node PUBLIC
  geometry_msgs
  rclcpp
  std_msgs
)
target_link_libraries(manus_raw_publisher_node
  PUBLIC
    ${MANUS_SDK_LIBRARY}
    ${cpp_typesupport_target}
)
set_target_properties(manus_raw_publisher_node PROPERTIES
  BUILD_RPATH "${PROJECT_SOURCE_DIR}/third_party/lib"
  INSTALL_RPATH "$ORIGIN/.."
)

add_executable(manus_gripper_node
  hardware/src/manus_gripper_node.cpp
)
target_compile_features(manus_gripper_node PUBLIC cxx_std_17)
target_include_directories(manus_gripper_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(manus_gripper_node PUBLIC
  rclcpp
  std_msgs
)
target_link_libraries(manus_gripper_node
  PUBLIC
    ${cpp_typesupport_target}
)

install(
  TARGETS
    manus_calibration_node
    manus_raw_publisher_node
    manus_gripper_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY hardware/include/
  DESTINATION include
)

install(
  DIRECTORY bringup/config bringup/launch bringup/calibrations
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    bringup/scripts/manus_calibration_workflow.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES ${MANUS_SDK_LIBRARY}
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs rclcpp rosidl_default_runtime std_msgs)

ament_package()
