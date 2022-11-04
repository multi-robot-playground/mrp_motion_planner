cmake_minimum_required(VERSION 3.5)
project(mrp_motion_planner_server)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(mrp_local_server_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(mrp_common REQUIRED)
find_package(mrp_rvo REQUIRED)
find_package(mrp_common_msgs REQUIRED)
find_package(mrp_motion_planner_msgs REQUIRED)
find_package(mrp_comms_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_components
  rclcpp_action
  mrp_local_server_core
  pluginlib
  geometry_msgs
  nav_msgs
  nav2_msgs
  mrp_common
  mrp_common_msgs
  mrp_rvo
  mrp_motion_planner_msgs
  mrp_comms_msgs
)

set(library_name motion_planner_server)
add_library(${library_name} SHARED
  src/motion_planner_server.cpp
)

set(executable_name motion_planner_server_exec)
add_executable(${executable_name}
  src/main.cpp
)

ament_target_dependencies(${library_name} ${dependencies})

target_link_libraries(${executable_name} 
  ${library_name}
)

install(
  TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  ${library_name})
ament_export_dependencies(${dependencies})
ament_package()

