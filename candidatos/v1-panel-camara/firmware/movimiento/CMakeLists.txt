# Proyecto ESP-IDF AUTONOMO del firmware de movimiento (ruedas, sonares, freno local y brazo en una sola ESP32).
# Se compila desde esta carpeta: no hace falta copiar nada al ejemplo int32_publisher.
cmake_minimum_required(VERSION 3.16)

# El componente micro-ROS no vive en el repo: es el que ya clonaste en ~/caid/pruebas-banco (rama jazzy).
# Si lo tenes en otro lado:  export MICROROS_COMPONENT=/ruta/a/micro_ros_espidf_component
if(DEFINED ENV{MICROROS_COMPONENT})
  set(MICROROS_DIR "$ENV{MICROROS_COMPONENT}")
else()
  set(MICROROS_DIR "$ENV{HOME}/caid/pruebas-banco/micro_ros_espidf_component")
endif()
if(NOT EXISTS "${MICROROS_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "No encuentro el componente micro-ROS en ${MICROROS_DIR}. Exporta MICROROS_COMPONENT con su ruta.")
endif()
set(EXTRA_COMPONENT_DIRS "${MICROROS_DIR}")

# sdkconfig.defaults va en el repo; sdkconfig.secreto (red Wi-Fi, clave, IP del agente) NO: lo genera configurar_wifi.sh
set(SDKCONFIG_DEFAULTS "sdkconfig.defaults")
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.secreto")
  list(APPEND SDKCONFIG_DEFAULTS "sdkconfig.secreto")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(robot_movimiento)
