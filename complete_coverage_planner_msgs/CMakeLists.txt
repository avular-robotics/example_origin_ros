cmake_minimum_required(VERSION 3.16)
project(complete_coverage_planner_msgs)

# When adding a dependency, you will have to both add it to the package.xml and
# this file.
# The dependencies listed in the package.xml are used to determine what other interface packages
# have to be installed or built first.
# Inside the CMakeLists.txt, the dependencies are used to find the packages and link the
# interfaces. This is done in three steps:
# 1. find_package() to find the package
# 2. rosidl_generate_interfaces() to link the interface packages
# 3. ament_export_dependencies() to avoid downstream packages to have to call
#    `find_package` for the dependencies of this package
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(MSGS # cmake-format: sortable
         msg/Area.msg msg/AreaStamped.msg msg/InfillPolicy.msg
)

set(SRVS # cmake-format: sortable
         srv/SetArea.srv
)
set(ACTIONS # cmake-format: sortable
            action/ComputePath.action
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${SRVS}
  ${MSGS}
  ${ACTIONS}
  DEPENDENCIES
  geometry_msgs
  nav_msgs
)

ament_export_dependencies(rosidl_default_runtime geometry_msgs nav_msgs)

# Package
ament_package()
