cmake_minimum_required(VERSION 3.8)
project(unitree_controller)

if(NOT CMAKE_CXX_STANDARD) #
  set(CMAKE_CXX_STANDARD 17) #
endif() #

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC) #
  add_compile_definitions(_USE_MATH_DEFINES) #
endif() #

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ros2_unitree_legged_msgs REQUIRED)
find_package(controller_manager REQUIRED)
#find_package(joint_state_controller REQUIRED) joint_trajectory_controller????
find_package(robot_state_publisher REQUIRED)
#find_package(gen_msg REQUIRED)

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

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "ros2_unitree_legged_msgs"
)

add_executable(external_force src/external_force.cpp)
ament_target_dependencies(external_force ${dependencies})

add_executable(servo src/servo.cpp)
ament_target_dependencies(servo ${dependencies})

add_executable(move_publisher src/move_publisher.cpp)
ament_target_dependencies(move_publisher ${dependencies})


#######
install(TARGETS external_force servo move_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch src
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)
#######

ament_package()
