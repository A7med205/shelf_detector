cmake_minimum_required(VERSION 3.8)
project(nav2_apps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(srv_files
  "srv/GoToLoading.srv"
  )

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToLoading.srv"
  DEPENDENCIES
)

add_executable(approach_service_server src/approach_service_server.cpp)
add_executable(transform_node src/transform_node.cpp)

ament_target_dependencies(approach_service_server
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
)

ament_target_dependencies(transform_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_target_interfaces(approach_service_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(transform_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

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

install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  approach_service_server
  transform_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
