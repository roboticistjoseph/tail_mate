cmake_minimum_required(VERSION 3.8)
project(followme_drone)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# --- C+++ dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
# --- Drone dependencies ---
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gazebo_msgs REQUIRED)
# --- Navigation dependencies ---
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)

# Adding Executables
add_executable(drone_patrol_controller scripts/drone_patrol_controller.cpp)
ament_target_dependencies(drone_patrol_controller rclcpp geometry_msgs)

add_executable(drone_motion_controller scripts/drone_motion_controller.cpp)
ament_target_dependencies(drone_motion_controller rclcpp geometry_msgs)

add_executable(drone_detection_node scripts/drone_aruco_tracking.cpp)
ament_target_dependencies(drone_detection_node rclcpp sensor_msgs cv_bridge)
target_link_libraries(drone_detection_node ${OpenCV_LIBS})

add_executable(tb3_nav_node scripts/waypoint_navigation.cpp)
ament_target_dependencies(tb3_nav_node 
  rclcpp
  rclcpp_action
  geometry_msgs 
  nav2_msgs
  nav2_util
)

# Install the executable
install(TARGETS drone_patrol_controller drone_motion_controller drone_detection_node tb3_nav_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/nav_scripts_python/single_point_nav.py
        DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/nav_scripts_python/waypoint_nav.py
DESTINATION lib/${PROJECT_NAME})


# Install subdirectories
install(DIRECTORY config launch maps nav2 rviz worlds scripts/ref_img
  DESTINATION share/${PROJECT_NAME}/
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
