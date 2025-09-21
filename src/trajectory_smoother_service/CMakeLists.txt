cmake_minimum_required(VERSION 3.8)
project(trajectory_smoother_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(chomp_motion_planner REQUIRED)
find_package(moveit_planners_chomp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SmoothTrajectory.srv"
  DEPENDENCIES trajectory_msgs
)

# Build the smoother node
add_executable(trajectory_smoother_node src/trajectory_smoother_node.cpp)

ament_target_dependencies(trajectory_smoother_node
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  chomp_motion_planner
  moveit_planners_chomp
  trajectory_msgs
  geometry_msgs
  moveit_msgs
)

# Link against generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(trajectory_smoother_node "${cpp_typesupport_target}")

# Install targets
install(TARGETS trajectory_smoother_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install parameter files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
