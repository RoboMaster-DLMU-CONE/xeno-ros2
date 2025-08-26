cmake_minimum_required(VERSION 3.8)
project(xeno_control)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
foreach (Dependency IN ITEMS ${HARDWARE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

option(XENO_CONTROL_SIMULATE "Run Xeno Control node with simulation" OFF)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()


add_library(xeno_control SHARED
        src/xeno_hardware_interface.cpp
        src/joints/CanDriverManager.cpp
        src/joints/Arm.cpp
        src/joints/Lift.cpp
        src/joints/Shift.cpp
        src/joints/Stretch.cpp
        src/joints/Suck.cpp
)

include(cmake/findOneMotor.cmake)
target_link_libraries(xeno_control PRIVATE OneMotor::OneMotor)

if (XENO_CONTROL_SIMULATE)
    target_compile_definitions(xeno_control PRIVATE XENO_CONTROL_SIMULATE)
endif ()

ament_target_dependencies(xeno_control PUBLIC ${HARDWARE_INCLUDE_DEPENDS})

target_include_directories(xeno_control PUBLIC include)

pluginlib_export_plugin_description_file(hardware_interface xeno_control.xml)
# descriptions
install(
        DIRECTORY description/ros2_control description/urdf
        DESTINATION share/xeno_control/description
)
# launch files
install(
        DIRECTORY bringup/
        DESTINATION share/xeno_control/launch
)
install(
        DIRECTORY description/launch
        DESTINATION share/xeno_control
)
# config files
install(
        DIRECTORY config/
        DESTINATION share/xeno_control/config
)

install(TARGETS xeno_control
        EXPORT export_xeno_control
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# ament_export_targets(export_xeno_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${HARDWARE_INCLUDE_DEPENDS})
ament_package()
