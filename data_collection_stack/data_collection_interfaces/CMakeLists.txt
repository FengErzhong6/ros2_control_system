cmake_minimum_required(VERSION 3.8)
project(data_collection_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DeviceState.msg"
  "msg/FaultEvent.msg"
  "msg/SessionInfo.msg"
  "msg/SystemState.msg"
  "srv/AcknowledgeFault.srv"
  "srv/GetSystemState.srv"
  "srv/ListRecipes.srv"
  "action/GoHome.action"
  "action/ShutdownSystem.action"
  "action/StartSession.action"
  "action/StartSystem.action"
  "action/StopSession.action"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
