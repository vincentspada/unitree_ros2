cmake_minimum_required(VERSION 3.8)
project(unitree_gazebo)

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

######
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ros2_unitree_legged_msgs)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

set(dependencies
  # "geometry_msgs"
  # "nav_msgs"
  # "rclcpp"
  # "sensor_msgs"
  # "tf2"
  "ros2_unitree_legged_msgs"
)

add_library(unitreeFootContactPlugin SHARED plugin/foot_contact_plugin.cc)
target_link_libraries(unitreeFootContactPlugin ${GAZEBO_LIBRARIES})

add_library(unitreeDrawForcePlugin SHARED plugin/draw_force_plugin.cc)
target_link_libraries(unitreeDrawForcePlugin ${GAZEBO_LIBRARIES})

######

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

#######
install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

#rviz urdf models
install(DIRECTORY launch worlds go1_description
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)
#######

ament_package()
