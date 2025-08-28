cmake_minimum_required(VERSION 3.8)
project(xeno_control_guard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

option(XENO_CONTROL_SIMULATE "Run Xeno Control node with simulation" OFF)

# Include OneMotor - first try to find it, then fetch if needed
include(FetchContent)
find_package(OneMotor QUIET)

if (NOT OneMotor_FOUND)
    message(STATUS "本地未找到 OneMotor 包，尝试从 GitHub 获取...")
    FetchContent_Declare(
            OneMotor_fetched
            GIT_REPOSITORY "https://github.com/RoboMaster-DLMU-CONE/OneMotor"
            GIT_TAG "main"
    )
    FetchContent_MakeAvailable(OneMotor_fetched)
else ()
    message(STATUS "已找到 OneMotor 版本 ${OneMotor_VERSION}")
endif ()

add_executable(xeno_control_guard_node src/xeno_control_guard_node.cpp)
target_include_directories(xeno_control_guard_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(xeno_control_guard_node PUBLIC c_std_99 cxx_std_20)
ament_target_dependencies(
  xeno_control_guard_node
  "rclcpp"
)

# Link OneMotor library
target_link_libraries(xeno_control_guard_node PRIVATE OneMotor::OneMotor)

# Add simulation compile definition if enabled
if (XENO_CONTROL_SIMULATE)
    target_compile_definitions(xeno_control_guard_node PRIVATE XENO_CONTROL_SIMULATE)
endif ()

install(TARGETS xeno_control_guard_node
  DESTINATION lib/${PROJECT_NAME})

# install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
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