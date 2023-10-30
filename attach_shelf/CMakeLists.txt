cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(attach_shelf REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Rotate.srv"  # Ruta al archivo de servicio personalizado
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

add_executable(param_vel_node src/pre_approach.cpp)
ament_target_dependencies(param_vel_node rclcpp geometry_msgs sensor_msgs nav_msgs attach_shelf rosidl_default_generators)

add_executable(rotate_node src/rotate_service.cpp)
ament_target_dependencies(rotate_node rclcpp geometry_msgs sensor_msgs nav_msgs std_msgs rosidl_default_generators attach_shelf)

install(TARGETS
  param_vel_node
  rotate_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  #rviz_config
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
