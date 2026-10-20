# Simulación (Gazebo Harmonic)

Simulación del robot en el laberinto, con ROS 2 Jazzy y Gazebo Sim 8 (Harmonic). Cubre la tarea DESC-03.

## Qué incluye

- Robot con tracción 4WD simulada (plugin `DiffDrive`).
- Tres sensores de distancia simulados (frontal, izquierdo y derecho).
- Cámara 640×480 a 15 Hz con servo de giro.
- Estados de las articulaciones, reloj de simulación y RViz.
- Bridge entre ROS 2 y Gazebo.

## Requisitos

- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim 8.x, paquetes `ros-jazzy-ros-gz*`, `rviz2`.
- Se recomienda una GPU dedicada. Medido con el mismo mundo: **RTF 17,5 %** con gráficos integrados (notebook) y **100 %** con una GTX 1660 Ti.

## Cómo levantarla

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch caid_bringup sim.launch.py
```

Argumentos de `sim.launch.py`:

| Argumento | Por defecto | Qué hace |
|---|---|---|
| `world` | `maze_2.world` | Mundo a cargar |
| `arm` | `fixed` | Modo del brazo (`none`, `fixed`, `actuated`) |
| `x`, `y`, `yaw` | `-0.45`, `2.7`, `1.5708` | Pose inicial (marcador verde de `maze_2.world`, mirando a la salida de la celda) |
| `gui` | `true` | `false` = solo el servidor, sin ventana |
| `rviz` | `true` | Abrir RViz |

Cerrar con **Ctrl+C** en la terminal del launch. Verificar que no quedó nada vivo:

```bash
ps -eo pid,etime,args | grep "gz sim" | grep -v grep
```

No debe imprimir nada.

## Paquetes involucrados

| Paquete | Rol |
|---|---|
| `caid_description` | Modelo (URDF/xacro). `gazebo.xacro` solo se incluye con `sim:=true` |
| `caid_gazebo` | Mundos (`worlds/`) |
| `caid_bringup` | `sim.launch.py`, `config/sim/bridge.yaml` y el nodo `scan_to_range` (paquete compartido con CAM-02) |

## Tópicos

Todo lo del simulador va bajo `/sim/`, salvo `/clock`.

| Tópico | Tipo ROS | Sentido | Descripción |
|---|---|---|---|
| `/sim/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo | Velocidad lineal x (m/s) y angular z (rad/s) |
| `/sim/odom` | `nav_msgs/Odometry` | Gazebo → ROS | Odometría calculada con el giro de las ruedas (sin covarianzas) |
| `/sim/ultrasonic/{front,left,right}/scan` | `sensor_msgs/LaserScan` | Gazebo → ROS | 5 rayos en un cono de 15° |
| `/sim/ultrasonic/{front,left,right}` | `sensor_msgs/Range` | nodo `scan_to_range` | Distancia mínima de los 5 rayos |
| `/sim/camera/image_raw` | `sensor_msgs/Image` | Gazebo → ROS | 640×480, `camera_optical_frame` |
| `/sim/camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo → ROS | Intrínsecos derivados del FOV |
| `/sim/camera_pan/cmd` | `std_msgs/Float64` | ROS → Gazebo | Ángulo del servo (rad), limitado a ±π/2 |
| `/sim/joint_states` | `sensor_msgs/JointState` | Gazebo → ROS | Ruedas y servo de la cámara |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → ROS | Hora de simulación |

Los nodos de ROS que consumen estos datos deben usar `use_sim_time: true`.

## Parámetros provisorios

Hay que reemplazarlos con mediciones del robot real.

| Parámetro | Valor actual | Dónde |
|---|---|---|
| Rango de los sensores de distancia | 0,02 a 4 m | `gazebo.xacro` |
| Cono de los sensores | 15° | `gazebo.xacro` |
| Ruido de los sensores | gaussiano, σ = 5 mm | `gazebo.xacro` |
| Frecuencia de los sensores | 20 Hz | `gazebo.xacro` |
| FOV horizontal de la cámara | 60° (1,047 rad) | `gazebo.xacro` |
| Límites de velocidad | ±0,5 m/s, ±3 rad/s | `gazebo.xacro` |
| `wheel_separation` | 0,204 m (**efectiva**; la geometria es 0,131 m) | `gazebo.xacro` |
| Límites de aceleración | ±1 m/s², ±5 rad/s² | `gazebo.xacro` |
| Velocidad máxima del servo | 3 rad/s | `gazebo.xacro` |
| Mundo: celda 0,45 m, pared 0,28 m | de un proyecto anterior, **no** del reglamento | `maze_2.world` |

## Calibración del giro

Un robot 4WD gira patinando, así que se comporta como si las ruedas estuvieran más separadas que
la trocha geométrica (0,131 m). `DiffDrive` usa `wheel_separation` para convertir la velocidad
angular pedida en velocidades de rueda: si el valor es chico, el robot gira de menos.

`tools/sim/prueba_giro.sh` compara el giro real (pose de Gazebo) con el de la odometría después de
girar ~3 s a 0,5 rad/s. Como la odometría usa la misma separación, vale: real / odometría =
separación usada / separación efectiva. La regla de ajuste es: nueva = actual / (real / odometría).

| `wheel_separation` | Real / odometría | Desplazamiento |
|---|---|---|
| 0,131 m (geométrica) | 61 % | 2,9 cm (73°) |
| 0,214 m | 105 % | 6,0 cm (124°) |
| 0,204 m (adoptada) | 101 % | 5,9 cm (119°) |

El desplazamiento es ≈ 0,05 cm por grado girado (≈ 4,5 cm en un giro de 90°).

Se probó además fricción anisotrópica en las ruedas (`mu1` = 1, `mu2` = 0,3 y 0,5, `fdir1` hacia
adelante): el resultado no cambió entre 0,3 y 0,5 y el desplazamiento por grado se duplicó, así que
no se usó. La separación efectiva depende de las ruedas y del piso: hay que repetir la prueba con el
robot real.

## Decisiones de diseño

- No usamos `ros2_control` (ver ADR 0001): el movimiento va por plugins nativos de Gazebo.
- Los ultrasonidos son un `gpu_lidar` de 5 rayos agrupados, y el nodo `scan_to_range` se queda con el rayo más cercano. Así el software publica `Range`, igual que lo hará el firmware real.
- El servidor de Gazebo se lanza **sin shell** y la GUI **después** de crear el robot (ver "Problemas conocidos").

## Pruebas manuales

```bash
# Avanzar y frenar
ros2 topic pub --once /sim/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
ros2 topic pub --once /sim/cmd_vel geometry_msgs/msg/Twist "{}"

# Con teclado
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/sim/cmd_vel

# Sensores
ros2 topic echo /sim/ultrasonic/front --once

# Cámara y servo
ros2 topic hz /sim/camera/image_raw
ros2 topic pub --once /sim/camera_pan/cmd std_msgs/msg/Float64 "{data: 0.5}"

tools/sim/prueba_giro.sh

```

Resultados de referencia con el robot en el punto de partida: distancia frontal ≈ 0,53 m, laterales ≈ 0,14 m, cámara ≈ 15,1 Hz, y el servo se detiene en 1,57 rad aunque se le pida 2,5.

## Problemas conocidos

- **Servidor de Gazebo huérfano:** si queda un `gz sim` vivo (se ve con `ps`), la simulación se desdobla y el robot "salta" entre dos estados. Se soluciona con `pkill -9 -f "gz sim"`.
- **GUI que arranca antes que el robot:** puede dibujar el modelo roto. Por eso el launch la abre después de crearlo.
- **Avisos inofensivos:** `gz_frame_id … not defined in SDF` y `libEGL warning … dri2 screen`.
- **RViz:** necesita `use_sim_time:=true` para mostrar el TF de las articulaciones móviles.
- **Tests:** no correr `colcon test` con el entorno de ESP-IDF cargado (su `pytest` choca con el de ROS).

## Pendiente

- Brazo en modo `actuated`: faltan controladores.
- Repetir la calibración del giro con el robot real.
- Medidas del laberinto del reglamento.
- Reemplazar los parámetros provisorios con medidas del robot real.