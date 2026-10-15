# Guía: entorno de desarrollo

Todos los integrantes deben tener el mismo entorno, así lo que funciona en una PC funciona en todas.

| Componente | Versión |
|---|---|
| Sistema operativo | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Simulador | Gazebo Harmonic (se instala con `ros-jazzy-ros-gz`; es la combinación recomendada para Jazzy) |
| Firmware | PlatformIO + `micro_ros_platformio` con distro `jazzy` |

## 1. Verificar qué tenés instalado

Desde la raíz del repo:

```bash
source /opt/ros/jazzy/setup.bash
bash tools/setup/check_env.sh
```

El script no instala nada: lista cada componente como `[OK]`, `[FALTA]` (con el comando para
instalarlo) o `[AVISO]` (opcional). Termina con un resumen.

## 2. Instalar lo que falte

Los `[FALTA]` de paquetes apt se instalan con el comando que muestra el script. Para instalar todos
de una vez:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-image-transport-plugins ros-jazzy-cv-bridge ros-jazzy-camera-calibration \
  ros-jazzy-vision-msgs python3-colcon-common-extensions python3-rosdep python3-opencv
```

Importante: Gazebo se instala **solo** con `ros-jazzy-ros-gz`, que trae Harmonic como paquetes de
ROS. No hay que agregar el repositorio de Gazebo (osrfoundation) aparte.

## 3. Agente micro-ROS

El agente es el programa de la PC que recibe los mensajes de la ESP32 (por UDP) y los convierte en
tópicos ROS 2. **No existe como paquete apt para Jazzy**: se compila desde el código fuente, una vez,
en un workspace aparte.

```bash
mkdir -p ~/microros_ws/src && cd ~/microros_ws
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
source /opt/ros/jazzy/setup.bash
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

Para correrlo (escucha por UDP en el puerto 8888):

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Alternativa con Docker, sin compilar:

```bash
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host \
  microros/micro-ros-agent:jazzy udp4 --port 8888
```

(Montar `/dev/shm` es parte del comando oficial: sin eso, la comunicación por memoria compartida de
DDS entre el contenedor y los nodos de la PC puede fallar.)

## 4. PlatformIO (firmware de las ESP32)

Opción recomendada: la extensión **PlatformIO IDE** de VS Code. Alternativa por terminal (en Ubuntu
24.04, `pip install` a nivel sistema está bloqueado, por eso se usa pipx):

```bash
sudo apt install pipx && pipx ensurepath && pipx install platformio
```

Para poder flashear por USB, el usuario tiene que estar en el grupo `dialout`:

```bash
sudo usermod -aG dialout $USER   # después cerrar sesión y volver a entrar
```

## 5. Git

```bash
git config --global user.name "Nombre Apellido"
git config --global user.email "mail-de-github@ejemplo.com"
```

Para autenticarse con GitHub desde la terminal, lo más simple es la GitHub CLI: `gh auth login`.
