cmake_minimum_required(VERSION 3.8)
project(costmap_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create obstacle_masking library
add_library(obstacle_masking STATIC
  src/obstacle_masking.cpp
)
target_include_directories(obstacle_masking PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(obstacle_masking
  nav_msgs
)

# Old executable (keep for compatibility)
add_executable(costmap_generator src/costmap_generator.cpp)
target_include_directories(costmap_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(costmap_generator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# New ROS2 node
add_executable(costmap_generator_node src/costmap_generator_node.cpp)
target_include_directories(costmap_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(costmap_generator_node obstacle_masking)
ament_target_dependencies(costmap_generator_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
)
target_compile_features(costmap_generator_node PUBLIC cxx_std_17)

install(TARGETS costmap_generator costmap_generator_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS obstacle_masking
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

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

ament_export_include_directories(include)
ament_export_libraries(obstacle_masking)
ament_export_dependencies(nav_msgs)
ament_package()
