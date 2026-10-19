cmake_minimum_required(VERSION 3.8)
project(caid_description)

find_package(ament_cmake REQUIRED)

# Copia estas carpetas a install/caid_description/share/caid_description/.
install(DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME})

# Las cotas viven en hardware/cad/ (una sola fuente de verdad) y se copian al paquete al instalar.
# La ruta sube 3 niveles: caid_description -> src -> ros2_ws -> raíz del repo.
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../../hardware/cad/dimensions.yaml
  DESTINATION share/${PROJECT_NAME}/config)

# Agrega share/ a GZ_SIM_RESOURCE_PATH para que Gazebo resuelva package://caid_description/...
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(urdf_tests test/test_urdf.py)
endif()

ament_package()
