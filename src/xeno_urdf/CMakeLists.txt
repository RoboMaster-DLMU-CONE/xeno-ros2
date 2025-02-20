cmake_minimum_required(VERSION 3.8)
project(xeno_urdf)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY xeno_urdf/meshes xeno_urdf/urdf xeno_urdf/rviz
  DESTINATION share/${PROJECT_NAME}/xeno_urdf
)

ament_package()
