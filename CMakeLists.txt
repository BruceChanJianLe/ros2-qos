cmake_minimum_required(VERSION 3.8)
project(ros2-qos)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# BUILD
# Example History
add_executable(history_qos_node
  src/history_qos_node.cpp
)
ament_target_dependencies(history_qos_node rclcpp std_msgs)

install(TARGETS
  history_qos_node
  DESTINATION lib/${PROJECT_NAME}
)

# Example Transient Local Pub
add_executable(transient_local_pub
  src/transient_local_pub.cpp
)
ament_target_dependencies(transient_local_pub rclcpp std_msgs)

install(TARGETS
  transient_local_pub
  DESTINATION lib/${PROJECT_NAME}
)

# Example Transient Local Sub
add_executable(transient_local_sub
  src/transient_local_sub.cpp
)
ament_target_dependencies(transient_local_sub rclcpp std_msgs)

install(TARGETS
  transient_local_sub
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
