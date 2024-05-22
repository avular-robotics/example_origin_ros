cmake_minimum_required(VERSION 3.16)
project(origin_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

file(
  GLOB MSGS
  RELATIVE ${CMAKE_CURRENT_LIST_DIR}
  ./msg/*.msg
)

file(
  GLOB SRVS
  RELATIVE ${CMAKE_CURRENT_LIST_DIR}
  ./srv/*.srv)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${MSGS}
  ${SRVS}
  DEPENDENCIES std_msgs rcl_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
