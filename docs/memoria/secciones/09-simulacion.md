# 09 · Modelo del robot y simulación en Gazebo

## Objetivo

La simulación expone los mismos sensores y actuadores que el robot real, con las mismas interfaces
de ROS 2, para que el software de navegación y percepción sea el mismo en simulación y en la
pista. Usa sistemas nativos de Gazebo, sin `ros2_control` (ADR 0001).

## Arquitectura de la simulación

| Pieza | Función |
|---|---|
| `caid_description` | Modelo URDF/xacro; `gazebo.xacro` agrega los plugins solo con `sim:=true` |
| `caid_gazebo` | Mundos del laberinto |
| `caid_bringup` | Launch, bridge ROS↔Gazebo y el nodo `scan_to_range` |

Guía de uso y lista completa de tópicos: `docs/arquitectura/simulacion.md`.

## Sensores y actuadores simulados y sus límites

| Elemento | Cómo se simula | Parámetros | Diferencias con el real |
|---|---|---|---|
| Tracción | Plugin `DiffDrive`, 2 ruedas por lado | separación efectiva 0,204 m (geométrica 0,131 m), radio 0,0335 m, ±0,5 m/s y ±3 rad/s, aceleración ±1 m/s² y ±5 rad/s² | La odometría no incluye patinaje; la separación efectiva absorbe el del giro del 4WD |
| Ultrasonidos | `gpu_lidar` de 5 rayos en un cono de 15°, convertido a `Range` con el rayo más cercano | 0,02–4 m, 20 Hz, ruido gaussiano σ = 5 mm | No modela ecos múltiples ni reflexiones en superficies inclinadas |
| Cámara | Sensor `camera` | 640×480, 15 Hz, FOV 60° | Sin distorsión (coeficientes en cero), sin ruido ni exposición |
| Servo de la cámara | `JointPositionController` | ±π/2, velocidad máxima 3 rad/s | Servo ideal: sin juego ni holgura |
| Odometría | Calculada con el giro de las ruedas | 50 Hz | Sin covarianzas |

Los valores de ruido, FOV y límites son **provisorios** y se reemplazan con mediciones del robot
real (tarea de banco de pruebas).

## Validación

| Prueba | Resultado |
|---|---|
| Distancia frontal en el punto de partida | rayo central 0,536 m, esperado 0,535 m (diferencia de 0,6 mm) |
| Distancias laterales (celda de 45 cm) | 0,139 m y 0,137 m: robot centrado |
| Aproximación a la pared de a 0,1 m/s | la distancia frontal baja de 0,526 a 0,414 m |
| Frecuencia de la cámara | 15,1 Hz, desvío de 1–2 ms |
| Límite del servo | con 2,5 rad se detiene en 1,57 rad |
| Rendimiento (RTF) | 17,5 % con gráficos integrados; 100 % con GTX 1660 Ti |
| Giro en el lugar (0,5 rad/s, 3 s) | giro real 119,0° contra 118,4° de la odometría (101 %); desplazamiento de 5,9 cm |

## Lecciones aprendidas

- Un servidor de Gazebo huérfano duplica la simulación: el launch lo ejecuta sin shell para que
  Ctrl+C lo termine.
- La GUI debe abrirse después de crear el robot; antes puede dibujarlo roto.
- Los nodos de ROS usan `use_sim_time` para que el TF de las articulaciones móviles se vea en RViz.
- Un 4WD gira patinando: se calibra una separación efectiva (0,204 m) y no la geométrica. La fricción anisotrópica no mejoró el giro y duplicaba el desplazamiento.