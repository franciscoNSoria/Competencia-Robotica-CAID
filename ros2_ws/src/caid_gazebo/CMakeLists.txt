cmake_minimum_required(VERSION 3.8)
project(caid_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds mazes scripts
  DESTINATION share/${PROJECT_NAME})

ament_package()