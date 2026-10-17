cmake_minimum_required(VERSION 3.8)
project(caid_description)

find_package(ament_cmake REQUIRED)

# Copia estas carpetas a install/caid_description/share/caid_description/.
# Se van sumando a medida que existen (meshes, launch, rviz).
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME})

# Agrega share/ a GZ_SIM_RESOURCE_PATH para que Gazebo resuelva package://caid_description/...
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
