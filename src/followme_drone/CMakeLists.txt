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

# Adding Dependencies


add_executable(drive_node scripts/drive_node.cpp)
ament_target_dependencies(drive_node rclcpp geometry_msgs)

add_executable(drone_patrol_controller scripts/drone_patrol_controller.cpp)
ament_target_dependencies(drone_patrol_controller rclcpp geometry_msgs)

add_executable(tb3_follower scripts/tb3_follower.cpp)
ament_target_dependencies(tb3_follower rclcpp sensor_msgs cv_bridge)
target_link_libraries(tb3_follower ${OpenCV_LIBS})

add_executable(tb3_nav_node scripts/multi_waypoints.cpp)
ament_target_dependencies(tb3_nav_node rclcpp geometry_msgs nav2_msgs nav2_util)

# add_executable(followme_drone_node scripts/followme_drone_node.cpp)
# ament_target_dependencies(followme_drone_node rclcpp geometry_msgs sensor_msgs cv_bridge)
# target_link_libraries(followme_drone_node ${OpenCV_LIBS})

# Install the executable
install(TARGETS drive_node tb3_follower tb3_nav_node drone_patrol_controller
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/single_goal_nav.py
        DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/multi_waypointspython.py
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
