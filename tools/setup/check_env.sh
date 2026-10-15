#!/usr/bin/env bash
# Verifica que la PC tenga todo lo necesario para trabajar en el proyecto CAID 2026.
# No instala nada: informa qué está, qué falta y con qué comando se instala.
#
# Uso (desde la raíz del repo, con ROS 2 cargado en la terminal):
#   source /opt/ros/jazzy/setup.bash
#   bash tools/setup/check_env.sh

set -u

N_OK=0
N_FALTA=0
N_AVISO=0

ok()    { printf '  \033[32m[OK]\033[0m     %s\n' "$1"; N_OK=$((N_OK + 1)); }
falta() { printf '  \033[31m[FALTA]\033[0m  %s\n           -> %s\n' "$1" "$2"; N_FALTA=$((N_FALTA + 1)); }
aviso() { printf '  \033[33m[AVISO]\033[0m  %s\n           -> %s\n' "$1" "$2"; N_AVISO=$((N_AVISO + 1)); }

# Paquete apt instalado: pkg <paquete> <descripción>
pkg() {
  if dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q "install ok installed"; then
    ok "$2 ($1 $(dpkg-query -W -f='${Version}' "$1" 2>/dev/null))"
  else
    falta "$2 ($1)" "sudo apt install $1"
  fi
}

# Comando disponible en el PATH: cmd <comando> <descripción> <cómo instalarlo>
cmd() {
  if command -v "$1" >/dev/null 2>&1; then
    ok "$2 ($(command -v "$1"))"
  else
    falta "$2 (comando '$1')" "$3"
  fi
}

# Módulo de Python importable: pymod <módulo> <paquete apt>
pymod() {
  if python3 -c "import $1" >/dev/null 2>&1; then
    ok "Python: módulo $1"
  else
    falta "Python: módulo $1" "sudo apt install $2"
  fi
}

echo "== Sistema operativo"
OS_ID=""; OS_VER=""; OS_NAME="desconocido"
if [ -r /etc/os-release ]; then
  OS_ID=$(. /etc/os-release && echo "${ID:-}")
  OS_VER=$(. /etc/os-release && echo "${VERSION_ID:-}")
  OS_NAME=$(. /etc/os-release && echo "${PRETTY_NAME:-desconocido}")
fi
if [ "$OS_ID" = "ubuntu" ] && [ "$OS_VER" = "24.04" ]; then
  ok "Ubuntu 24.04"
else
  falta "Ubuntu 24.04 (encontrado: $OS_NAME)" "ROS 2 Jazzy está soportado oficialmente sobre Ubuntu 24.04"
fi

echo "== ROS 2 Jazzy"
if [ "${ROS_DISTRO:-}" = "jazzy" ]; then
  ok "ROS 2 cargado en esta terminal (ROS_DISTRO=jazzy)"
else
  falta "ROS 2 Jazzy cargado en esta terminal (ROS_DISTRO='${ROS_DISTRO:-}')" \
        "source /opt/ros/jazzy/setup.bash   (y agregar esa línea al final de ~/.bashrc)"
fi
pkg ros-jazzy-desktop                  "ROS 2 Jazzy Desktop (incluye RViz y rqt)"
pkg ros-jazzy-ros-gz                   "Gazebo Harmonic + puente ROS-Gazebo"
pkg ros-jazzy-xacro                    "xacro (descripción del robot)"
pkg ros-jazzy-robot-state-publisher    "robot_state_publisher"
pkg ros-jazzy-joint-state-publisher-gui "joint_state_publisher_gui (mover articulaciones en RViz)"
pkg ros-jazzy-teleop-twist-keyboard    "teleop_twist_keyboard (manejo por teclado en desarrollo)"
pkg ros-jazzy-image-transport-plugins  "image_transport_plugins (imágenes comprimidas)"
pkg ros-jazzy-cv-bridge                "cv_bridge (OpenCV <-> ROS)"
pkg ros-jazzy-camera-calibration       "camera_calibration (calibración de la cámara)"
pkg ros-jazzy-vision-msgs              "vision_msgs (mensajes de detecciones)"
pkg python3-colcon-common-extensions   "colcon (compilar el workspace)"
pkg python3-rosdep                     "rosdep (instalar dependencias de paquetes)"
cmd gz "Gazebo (gz sim)" "sudo apt install ros-jazzy-ros-gz   (y cargar ROS en la terminal)"

echo "== Python"
cmd python3 "Python 3" "sudo apt install python3"
pymod cv2 python3-opencv
pymod numpy python3-numpy
pymod yaml python3-yaml

echo "== micro-ROS y firmware"
if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then
  ok "micro_ros_agent compilado y cargado en esta terminal"
elif command -v docker >/dev/null 2>&1 && docker image inspect microros/micro-ros-agent:jazzy >/dev/null 2>&1; then
  ok "micro-ROS agent disponible como imagen Docker (microros/micro-ros-agent:jazzy)"
else
  aviso "micro_ros_agent no encontrado (no está en apt para Jazzy)" \
        "compilarlo desde fuente con micro_ros_setup (rama jazzy) o usar Docker; ver docs/guias/entorno.md"
fi
if command -v pio >/dev/null 2>&1; then
  ok "PlatformIO ($(command -v pio))"
elif [ -x "$HOME/.platformio/penv/bin/pio" ]; then
  ok "PlatformIO (instalado por la extensión de VS Code en ~/.platformio)"
else
  falta "PlatformIO (compilar y flashear las ESP32)" \
        "extensión 'PlatformIO IDE' de VS Code, o: sudo apt install pipx && pipx install platformio"
fi
if id -nG 2>/dev/null | grep -qw dialout; then
  ok "Usuario en el grupo dialout (acceso al puerto USB serie de las ESP32)"
else
  falta "Usuario en el grupo dialout (sin esto no se puede flashear por USB)" \
        "sudo usermod -aG dialout \$USER   (después cerrar sesión y volver a entrar)"
fi

echo "== Git y GitHub"
cmd git "git" "sudo apt install git"
if [ -n "$(git config --global user.name 2>/dev/null)" ] && [ -n "$(git config --global user.email 2>/dev/null)" ]; then
  ok "git configurado como: $(git config --global user.name) <$(git config --global user.email)>"
else
  falta "git sin nombre o email" \
        "git config --global user.name \"Tu Nombre\" && git config --global user.email \"tu@email\""
fi
if command -v gh >/dev/null 2>&1; then
  ok "GitHub CLI ($(gh --version 2>/dev/null | head -n1))"
else
  aviso "GitHub CLI (gh) no instalado: es opcional, sirve para crear PRs e issues desde la terminal" \
        "ver https://github.com/cli/cli/blob/trunk/docs/install_linux.md"
fi

echo
printf 'Resumen: %d OK, %d FALTA, %d AVISO\n' "$N_OK" "$N_FALTA" "$N_AVISO"
if [ "$N_FALTA" -gt 0 ]; then
  exit 1
fi
exit 0
