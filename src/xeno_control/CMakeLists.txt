cmake_minimum_required(VERSION 3.8)
project(xeno_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(HARDWARE_INCLUDE_DEPENDS
  pluginlib
  hardware_interface
  trajectory_msgs
  rclcpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
foreach(Dependency IN ITEMS ${HARDWARE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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



add_library(xeno_control SHARED src/xeno_hardware_interface.cpp)
ament_target_dependencies(xeno_control PUBLIC ${HARDWARE_INCLUDE_DEPENDS})

target_include_directories(xeno_control PUBLIC include)

pluginlib_export_plugin_description_file(hardware_interface xeno_control.xml)

install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/xeno_control
)

install(
  DIRECTORY bringup/
  DESTINATION share/xeno_control
)

ament_package()
