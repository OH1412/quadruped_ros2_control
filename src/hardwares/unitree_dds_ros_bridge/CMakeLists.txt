cmake_minimum_required(VERSION 3.8)
project(unitree_dds_ros_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(backward_ros REQUIRED)

add_executable(unitree_dds_ros_bridge src/unitree_dds_ros_bridge.cpp)
target_include_directories(unitree_dds_ros_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(unitree_dds_ros_bridge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(unitree_dds_ros_bridge unitree_sdk2)
ament_target_dependencies(unitree_dds_ros_bridge rclcpp sensor_msgs nav_msgs std_msgs backward_ros)

install(TARGETS unitree_dds_ros_bridge
  DESTINATION lib/${PROJECT_NAME})

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
