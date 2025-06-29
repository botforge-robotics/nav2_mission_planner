cmake_minimum_required(VERSION 3.8)
project(nav2_mission_planner_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate the messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavGoal.action"
  "srv/LaunchWithArgs.srv"
  "srv/StopLaunch.srv"
  "srv/GetMapList.srv"
  "srv/DeleteMap.srv"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_package()