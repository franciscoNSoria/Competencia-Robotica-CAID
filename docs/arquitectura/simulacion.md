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
| `/sim/odom` | `nav_msgs/Odometry` | Gazebo → ROS | Odometría del plugin: integra el giro exacto de las ruedas con la separación exacta. Es casi perfecta |
| `/sim/odom_ruedas` | `nav_msgs/Odometry` | nodo `odom_ruedas` | La misma cuenta que hace el robot real: flancos de encoder enteros (40 por vuelta) y, si se piden, los errores sistemáticos de un 4x4 |
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

## Simulador rápido de la misión (sin Gazebo)

`tools/sim/mision_rapida.py` corre el `explorador.py` **de verdad**, sin ROS ni Gazebo, en menos
de un segundo. Reemplaza ROS por módulos de mentira, teletransporta al robot de celda en celda,
calcula los ultrasonidos con la grilla y hace de cámara con un oráculo: proyecta las flechas, los
cubos y los canastos del mismo YAML que usa `gen_maze.py`, con el mismo modelo de cámara que la
navegación, y respeta las paredes que tapan. Sirve para probar la **lógica de la misión** antes
de gastar una corrida de Gazebo; no prueba el control fino ni la visión real.

```bash
python3 tools/sim/mision_rapida.py ros2_ws/src/caid_gazebo/mazes/e2_bifurcaciones_simple_abierta.yaml
python3 tools/sim/mision_rapida.py <yaml> --canasto-falso   # repite el falso canasto de E2
python3 tools/sim/mision_rapida.py <yaml> -v                # todo el log del explorador
```

Sale con código 0 si entregó los tres cubos. `caid_navigation/test/test_mision_rapida.py` lo
corre como prueba.

Al 04/11 reproduce la falla de la primera corrida en Gazebo con E2 (un "canasto verde" falso
adentro del laberinto y un salto de 13–14 celdas al relocalizarse) y, con los arreglos, entrega
los tres cubos en los 12 escenarios simples (E1 a E6, con la partida cerrada y abierta).

## Zona de partida abierta (e1a a e6a)

En la pista real la zona de partida es **abierta**: no hay paredes de referencia para volver a
entrar. `simplificar_laberintos.py` escribe, además de cada `_simple.yaml`, un
`_simple_abierta.yaml` con las paredes de la zona sacadas (la puerta sigue entre paredes y los
canastos van al centro de su celda). En el launch son los escenarios `e1a` a `e6a`.

El explorador lo resuelve así:

1. **Reconocimiento** al arrancar: barrido con el servo, media vuelta y otro barrido, para
   ubicar los tres canastos con la pose perfecta de la partida. Los canastos solo se aceptan
   con el robot en la zona.
2. **Puerta**: la primera celda con pared a los dos lados. Mientras busca, no sale por ahí.
3. **Punto de reingreso**: la primera bifurcación pasando la puerta, con el rumbo que traía.
   Toda ruta hacia adentro pasa por ahí. Después de cada entrega vuelve, se orienta igual,
   verifica las paredes contra el mapa y sigue buscando.

## Servo de la cámara

El `camera_pan_joint` está en el URDF y en el puente desde SIM-02, pero hasta el 04/11 no lo movía
nadie: el explorador leía siempre con la cámara al frente. Con las flechas nuevas eso perdía la
mitad de las pistas, porque una flecha solo se lee a 0,45–0,80 m y la pared lateral de la celda de
al lado **no entra en el campo visual** mirando de frente (60° de FOV: la pared de al lado queda a
unos 76°).

Ahora, en un cruce, el explorador pasa por el estado `MIRAR`: frena, manda el servo a cada ángulo
de `angulos_camara` (por defecto 0 y ±0,95 rad), espera a que llegue, mira `tiempo_mirar` y sigue.
Solo barre cuando vale la pena: en una celda con dos o más salidas, buscando un cubo y si todavía
no leyó una flecha de ese color ahí. Las detecciones se descartan mientras el servo se mueve,
porque el ángulo de la foto no sería el que se usa para proyectar el rayo.

Medido con flechas renderizadas, para la pared lateral de la celda contigua:

| ángulo del servo | resultado |
|---|---|
| 0 rad (al frente) | la flecha **no entra en cuadro** |
| 0,95 rad | se detecta, se ubica y se clasifica bien, las tres clases |

Se apaga con `usar_servo_camara:=false`.

## Odometría: cuál usar

`/sim/odom` es demasiado buena y esconde el problema real. El nodo `caid_bringup/odom_ruedas.py`
rehace la cuenta como la va a hacer el robot: cuenta **flancos enteros** de los encoders ópticos
(40 por vuelta de rueda, medido en `docs/guias/caracterizacion-motores.md`) y permite meterle los
dos errores sistemáticos que importan.

Por qué eso y no ruido blanco sobre `/sim/odom`: el error de un skid-steer de 4 ruedas **no es
ruido**, es un error de **escala en el rumbo** que se acumula giro a giro. Las cuatro ruedas
arrastran de costado al girar, así que el robot se comporta como si la trocha fuera mayor que los
131 mm geométricos (en `gazebo.xacro` está calibrada en 204 mm, y ese número cambia con el piso,
la carga y la velocidad de giro). Si la separación que usa la cuenta se desvía un 10 %, cada giro
de 90° se va 9°, siempre para el mismo lado. Ruido blanco no reproduce eso.

Números de este robot (radio 33,5 mm, 40 flancos por vuelta):

| Cosa | Valor |
|---|---|
| Un flanco | 5,26 mm de avance de la rueda |
| Una celda de 400 mm | 76 flancos |
| Un giro de 90° (separación 204 mm) | 30,4 flancos por rueda |
| Error de rumbo por un flanco de diferencia | 1,5° |

O sea: un giro de 90° se resuelve con **30 flancos**, y errar por uno solo ya son 1,5°. Sin IMU,
el rumbo solo se endereza contra las paredes (`corregir_posicion` y `estimar_rumbo` en
`explorador.py`), y por eso conviene probar con la odometría mala.

Se elige al lanzar la misión:

```bash
ros2 launch caid_navigation mision_sim.launch.py odom:=ruedas    # por defecto: solo cuantización
ros2 launch caid_navigation mision_sim.launch.py odom:=ruidosa   # + errores sistemáticos
ros2 launch caid_navigation mision_sim.launch.py odom:=gazebo    # la vieja, casi perfecta
```

Los valores de `ruidosa` (radio +2 %, separación +8 %, deslizamiento 5 %) son una **estimación**:
hay que calibrarlos contra el robot real con `tools/sim/prueba_giro.sh`.

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