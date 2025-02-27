cmake_minimum_required(VERSION 3.8)
project(amr_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(amr_msgs REQUIRED)   
find_package(yaml-cpp REQUIRED)   
find_package(tf2_ros REQUIRED)   
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(saver src/saver.cpp)
ament_target_dependencies(saver rclcpp nav_msgs visualization_msgs amr_msgs geometry_msgs yaml-cpp)

add_executable(visualiser src/visualiser.cpp)
ament_target_dependencies(visualiser rclcpp nav_msgs visualization_msgs amr_msgs geometry_msgs yaml-cpp tf2_ros tf2_geometry_msgs)


target_link_libraries(saver yaml-cpp)
target_link_libraries(visualiser yaml-cpp)

install(TARGETS
saver
visualiser
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
