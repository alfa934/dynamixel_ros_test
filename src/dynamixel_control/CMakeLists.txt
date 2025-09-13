cmake_minimum_required(VERSION 3.8)
project(dynamixel_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(dynamixel_controller src/dynamixel_controller.cpp)
ament_target_dependencies(dynamixel_controller
  rclcpp
  dynamixel_sdk
  dynamixel_sdk_custom_interfaces
)

add_executable(position_publisher src/position_pub.cpp)
ament_target_dependencies(position_publisher
  rclcpp
  dynamixel_sdk_custom_interfaces
)

add_executable(control_gait src/control_gait.cpp)
ament_target_dependencies(control_gait
  rclcpp
  dynamixel_sdk_custom_interfaces
  std_msgs
)

install(TARGETS
  dynamixel_controller
  position_publisher
  control_gait
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
