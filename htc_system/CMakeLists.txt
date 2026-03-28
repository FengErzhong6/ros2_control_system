cmake_minimum_required(VERSION 3.8)
project(htc_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# --- OpenVR ---
set(OPENVR_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/third_party/openvr")
set(OPENVR_LIBRARY     "${PROJECT_SOURCE_DIR}/third_party/openvr/libopenvr_api.so")

# --- Tracker publisher node ---
add_executable(tracker_publisher src/tracker_publisher.cpp)
target_compile_features(tracker_publisher PUBLIC cxx_std_17)
target_include_directories(tracker_publisher PRIVATE ${OPENVR_INCLUDE_DIR})
target_compile_definitions(
  tracker_publisher PRIVATE
  HTC_SYSTEM_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)
ament_target_dependencies(
  tracker_publisher
  ament_index_cpp
  rclcpp
  geometry_msgs
  std_msgs
  tf2_ros
)
target_link_libraries(tracker_publisher ${CMAKE_DL_LIBS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY bringup/config bringup/launch
  DESTINATION share/${PROJECT_NAME}/bringup
)

install(
  PROGRAMS scripts/export_tf_bag_to_csv.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS tracker_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES ${OPENVR_LIBRARY}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
