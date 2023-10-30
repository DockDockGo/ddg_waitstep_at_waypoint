cmake_minimum_required(VERSION 3.8)
project(ddg_waitstep_at_waypoint_plugin)

set(lib_name ${PROJECT_NAME}_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_waypoint_follower REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_core REQUIRED)

set(dep_pkgs
    rclcpp
    nav2_waypoint_follower
    pluginlib)

add_library(${lib_name} SHARED
        src/ddg_waitstep_at_waypoint.cpp)
include_directories(include)

# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)

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

ament_target_dependencies(${lib_name} ${dep_pkgs})
pluginlib_export_plugin_description_file(ddg_waitstep_at_waypoint_plugin ddg_waitstep_at_waypoint.xml)

ament_package()
