cmake_minimum_required(VERSION 3.8)
project(ros2-qos)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17/20
if(NOT CMAKE_CXX_STANDARD)
  if("$ENV{ROS_DISTRO}" STREQUAL "foxy" OR "$ENV{ROS_DISTRO}" STREQUAL "galactic")
    set(CMAKE_CXX_STANDARD 17)
  else()
    set(CMAKE_CXX_STANDARD 20)
  endif()
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
set(dependencies
  rclcpp
  std_msgs
  rclcpp_lifecycle
)

foreach(pkg IN LISTS dependencies)
  find_package(${pkg} REQUIRED)
endforeach()

# BUILD
# Example History
# add_executable(history_qos_node
#   src/history_qos_node.cpp
# )
# ament_target_dependencies(history_qos_node rclcpp std_msgs)
#
# install(TARGETS
#   history_qos_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# Example Transient Local Pub
add_executable(transient_local_pub
  src/transient_local_pub.cpp
)
ament_target_dependencies(transient_local_pub ${dependencies})

install(TARGETS
  transient_local_pub
  DESTINATION lib/${PROJECT_NAME}
)

# Example Transient Local Sub
add_executable(transient_local_sub
  src/transient_local_sub.cpp
)
ament_target_dependencies(transient_local_sub ${dependencies})

install(TARGETS
  transient_local_sub
  DESTINATION lib/${PROJECT_NAME}
)

# Example Watchdog Node
add_executable(example_watchdog_node
  ./src/example_watchdog.cpp
)
ament_target_dependencies(example_watchdog_node ${dependencies})

install(TARGETS
  example_watchdog_node
  DESTINATION lib/${PROJECT_NAME}
)

# Example odom publisher
add_executable(example_odom_publisher_node
  ./src/example_odom_publisher.cpp
)
ament_target_dependencies(example_odom_publisher_node ${dependencies})

install(TARGETS
  example_odom_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
