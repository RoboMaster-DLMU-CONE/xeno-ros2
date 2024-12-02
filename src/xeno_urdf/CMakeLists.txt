cmake_minimum_required(VERSION 3.8)
project(xeno_urdf)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY r6bot/meshes r6bot/srdf r6bot/urdf r6bot/rviz
  DESTINATION share/${PROJECT_NAME}/r6bot
)

ament_package()
