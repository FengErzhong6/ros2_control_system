cmake_minimum_required(VERSION 3.8)
project(htc_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
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
ament_target_dependencies(tracker_publisher rclcpp geometry_msgs std_msgs tf2_ros)
target_link_libraries(tracker_publisher ${OPENVR_LIBRARY})

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
  TARGETS tracker_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES ${OPENVR_LIBRARY}
  DESTINATION lib
)

ament_package()
