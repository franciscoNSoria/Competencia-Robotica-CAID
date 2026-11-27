# Proyecto ESP-IDF de la ESP32-CAM (AI-Thinker) - CAID 2026
cmake_minimum_required(VERSION 3.16)

# Versión del firmware: la devuelve /status y se imprime al arrancar.
# Subirla en cada versión que va al robot y etiquetar en git: fw-cam-vX.Y.Z
set(PROJECT_VER "0.1.0")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_cam)
