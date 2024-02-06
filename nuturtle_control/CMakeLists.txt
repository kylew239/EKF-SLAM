cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(turtle_control_node src/turtle_control.cpp)
add_executable(odom_node src/odometry.cpp)
add_executable(circle_node src/circle.cpp)

ament_target_dependencies(turtle_control_node
                          rclcpp
                          geometry_msgs
                          nuturtlebot_msgs
                          turtlelib
                          sensor_msgs)

ament_target_dependencies(odom_node
                          rclcpp
                          nav_msgs
                          sensor_msgs
                          turtlelib
                          tf2
                          tf2_ros)

ament_target_dependencies(circle_node
                          rclcpp
                          std_srvs
                          geometry_msgs)

target_link_libraries(turtle_control_node turtlelib::turtlelib)
target_link_libraries(odom_node turtlelib::turtlelib)

# For custom Services
rosidl_generate_interfaces(${PROJECT_NAME}_srv "srv/InitialPose.srv" "srv/Control.srv" LIBRARY_NAME ${PROJECT_NAME})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_srv "rosidl_typesupport_cpp")

target_link_libraries(odom_node "${cpp_typesupport_target}")
target_link_libraries(circle_node "${cpp_typesupport_target}")


install(TARGETS
  turtle_control_node
  odom_node
  circle_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/nuturtle_control
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
