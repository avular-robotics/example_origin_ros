cmake_minimum_required(VERSION 3.16)
project(knowledge_base_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(autonomy_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(complete_coverage_planner_msgs REQUIRED)


file(
  GLOB MSGS
  RELATIVE ${CMAKE_CURRENT_LIST_DIR}
  ./msg/*.msg)
file(
  GLOB SRVS
  RELATIVE ${CMAKE_CURRENT_LIST_DIR}
  ./srv/*.srv)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${SRVS}
  ${MSGS}
  DEPENDENCIES std_msgs nav_msgs autonomy_msgs geometry_msgs complete_coverage_planner_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
