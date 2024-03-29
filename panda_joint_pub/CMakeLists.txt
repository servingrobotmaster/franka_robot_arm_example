cmake_minimum_required(VERSION 3.8)
project(panda_joint_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)

add_executable(robot_state_publisher src/robot_state_publisher.cpp)
add_executable(robot_state_subscriber src/robot_state_subscriber.cpp)
ament_target_dependencies(robot_state_publisher rclcpp sensor_msgs urdf)
ament_target_dependencies(robot_state_subscriber rclcpp sensor_msgs)

install(TARGETS
  robot_state_publisher
  robot_state_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
