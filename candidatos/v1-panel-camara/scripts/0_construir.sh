#!/usr/bin/env bash
# Una vez (y despues de cambiar codigo de ros2_ws): compila los paquetes de ROS 2 de esta carpeta. Sin ESP-IDF cargado.
set -euo pipefail
[ -z "${IDF_PATH:-}" ] || { echo "Esta terminal tiene ESP-IDF cargado: usa otra."; exit 1; }
AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
cd "$AQUI/../ros2_ws"
colcon build --symlink-install --packages-select caid_vision
