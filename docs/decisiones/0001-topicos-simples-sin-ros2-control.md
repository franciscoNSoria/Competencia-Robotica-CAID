# 0001. Robot expuesto por tópicos simples de micro-ROS, sin ros2_control

- **Estado:** Propuesta
- **Fecha:** 2026-10-14

## Contexto

El robot tiene dos microcontroladores: una ESP32 que controla motores, encoders, ultrasonidos y
servos, y una ESP32-CAM que transmite video. El "cerebro" (percepción, navegación, misión) corre en
la PC con ROS 2 Jazzy, conectada por Wi-Fi. Además queremos simular en Gazebo Harmonic y usar el
mismo software en simulación y en el robot real.

`ros2_control` es el marco estándar de ROS 2 para controlar hardware. Tiene tres partes:

- un **controller_manager**, que corre un lazo periódico en la PC (por ejemplo a 100 Hz);
- una **hardware interface**, un plugin en C++ que en cada ciclo lee el hardware (`read()`:
  posiciones y velocidades) y le escribe comandos (`write()`);
- **controladores** reutilizables, como `diff_drive_controller` (convierte `cmd_vel` en
  velocidades de rueda y calcula la odometría) o `joint_trajectory_controller` (brazos).

En simulación, `gz_ros2_control` reemplaza la hardware interface por Gazebo.

## Decisión

No usamos `ros2_control`. La ESP32 corre micro-ROS y publica y recibe **tópicos simples**:
`/cmd_vel` (`geometry_msgs/Twist`), ticks de encoders, distancias de ultrasonidos, órdenes del brazo
y estado del robot. El lazo de control de velocidad de las ruedas corre **en la ESP32**, junto a los
encoders. La odometría se calcula en la PC a partir de los ticks.

En Gazebo usamos los sistemas nativos (`DiffDrive` con 4 ruedas, `JointPositionController`,
cámara y `gpu_lidar` convertido a `Range`) más un nodo "emulador de firmware" que expone
exactamente los mismos tópicos que la ESP32 real.

## Alternativas consideradas

- **ros2_control con una hardware interface que hable con la ESP32 por Wi-Fi.** El lazo de control
  quedaría en la PC, del otro lado de un enlace Wi-Fi con latencia variable (decenas de ms, con
  picos). Un lazo de velocidad de ruedas necesita una frecuencia estable, y eso solo se garantiza
  en el microcontrolador. Además habría que escribir el plugin en C++ y mantener dos capas de
  comunicación (micro-ROS y la interfaz).
- **ros2_control solo en simulación (`gz_ros2_control`).** La simulación y el robot real
  tendrían interfaces distintas (por ejemplo, `diff_drive_controller` en Jazzy solo acepta
  `TwistStamped`), y perderíamos la paridad que justamente buscamos.

## Consecuencias

- Más simple: los mismos tópicos en simulación y en el robot, y nada de C++ obligatorio en la PC.
- Continúa lo que el equipo ya usó en el curso (el plugin `DiffDrive` de Gazebo).
- A cambio, implementamos nosotros cosas que `ros2_control` ya trae: la odometría a partir de los
  ticks, el límite de aceleración y el watchdog (si dejan de llegar órdenes, los motores frenan).
- `ros2_control` tendría sentido con una computadora a bordo conectada por cable a los drivers, o
  para usar MoveIt con el brazo. No es nuestro caso.
