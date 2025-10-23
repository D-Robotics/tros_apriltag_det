cmake_minimum_required(VERSION 3.8)
project(tros_apriltag_det)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(angles REQUIRED)

add_executable(msg_bridge src/msg_bridge.cpp)
add_executable(pose_det src/pose_det.cpp)

ament_target_dependencies(msg_bridge
  "rclcpp"
  "apriltag_msgs"
  "ai_msgs"
)
ament_target_dependencies(pose_det
  "rclcpp"
  "ai_msgs"
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_ros
  angles
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

install(
  TARGETS msg_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS pose_det
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
