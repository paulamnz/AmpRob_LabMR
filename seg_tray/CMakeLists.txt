cmake_minimum_required(VERSION 3.8)
project(seg_tray)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(seg_tray_msgs REQUIRED)

# Executables

add_executable(nav_p2p src/nav_p2p.cpp)
ament_target_dependencies(nav_p2p 
  rclcpp 
  geometry_msgs 
  nav_msgs
)

add_executable(corr_nav src/corr_nav.cpp)
ament_target_dependencies(corr_nav 
  rclcpp 
  geometry_msgs 
  nav_msgs
  sensor_msgs
  seg_tray_msgs
)

target_include_directories(corr_nav PUBLIC
  ${seg_tray_msgs_INCLUDE_DIRS}
)

add_executable(corr_nav_wall src/corr_nav_wall.cpp)
ament_target_dependencies(corr_nav_wall
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
)

# Install executables
install(TARGETS
  nav_p2p
  corr_nav
  corr_nav_wall
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
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
