# Navegación y misión (NAV, PER, MIS)

> Paquetes `caid_navigation` y `caid_vision`. Decisión de fondo: ADR 0004 (sin Nav2 ni SLAM).
> **Algoritmos, fórmulas y referencias: [navegacion-teoria.md](navegacion-teoria.md).**
> Estados: **M** medido, **C** calculado, **P** supuesto a confirmar, **S** simulado.

## 1. Qué hace

El robot arranca sin saber nada del laberinto (el reglamento no admite un mapa programado) y:

1. Al encender guarda la **partida**: es el origen del mapa, (0, 0) con rumbo 0.
2. Mira alrededor de la partida con la cámara (gira 90° a cada lado) para encontrar los canastos.
3. Explora celda por celda (exploración por fronteras + Dijkstra). Al pasar guarda la **entrada**
   del laberinto (la primera celda con pared a los dos costados) y la **primera bifurcación**
   (la primera celda adentro con 3 salidas), con su rumbo ("actitud"). Las flechas de los colores
   que faltan encarecen los otros caminos de esa bifurcación (aceleran, no obligan).
4. Si ve un cubo con la pinza libre, va a buscarlo y pide al brazo que lo agarre.
5. Con el cubo arriba va **más suave** (0,15 m/s, 0,9 rad/s, arranques limitados) por el camino
   más corto del mapa conocido hasta su canasto y pide soltarlo.
6. Después de entregar vuelve a la **entrada** (o a la bifurcación, parámetro
   `punto_de_reingreso`) y sigue desde ahí; el exterior queda para el final (salida abierta).
7. Con los tres entregados **queda quieto** junto al canasto (reposo). Si se acaba el tiempo
   (`tiempo_max`) vuelve a la partida.

Respeta las zonas rojas (no entra), esquiva bloques chicos pegados a una pared (pasa corrido) y
marca como cerrado un pasillo tapado (bloque de 20 cm).

## 2. Piezas

| Módulo | Qué hace |
|---|---|
| `geometria.py` | Medidas del robot, sensores y cámara (de `hardware/cad/dimensions.yaml`) |
| `conocimiento.py` | Lo que sabe en la ronda: lados (paso/pared con votos), bloques, obstáculos, zonas |
| `giros.py` | Si se puede girar en una celda y con qué corrimiento (huella del robot vs. bloques) |
| `planificador.py` | Dijkstra sobre (celda, rumbo): avanzar, retroceder, girar |
| `localizacion.py` | Pose: odometría corregida con paredes (lateral, frente, bordes, rumbo, factor de giro) |
| `sonar.py` | Registro de paredes con los ultrasonidos y barrido de obstáculos |
| `control.py` | Recta por el eje de un pasillo y giro en el lugar |
| `camara.py` | De píxeles al mapa: flechas, cubos, canastos y zonas rojas |
| `mision.py` | Navegador: une todo y decide (sin ROS) |
| `navegador_node.py` | Nodo ROS 2 que envuelve al navegador |
| `manipulacion_simulada.py` | Brazo de mentira para Gazebo mientras MAN no está integrado |
| `simulacion/` | Simulador 2D, render de cámara y corridas en lote para probar sin Gazebo |
| `caid_vision/detector.py` | Detección en la imagen (OpenCV clásico, sin ROS) |
| `caid_vision/vision_node.py` | Nodo ROS 2 de visión: publica `/vision/detecciones` |

## 3. Decisiones que vale la pena conocer

### 3.1 El robot casi no entra en el pasillo (C)

Huella de 278 × 163 mm con el centro de giro en `base_link`: las esquinas delanteras quedan a 162 mm
y las traseras a 160 mm. El pasillo deja 160 mm a cada lado del eje. Consecuencias:

- **No puede girar 180° en un pasillo ni en un callejón**: sale marcha atrás hasta una celda donde sí
  puede girar (el planificador lo resuelve solo: un giro imposible no existe como acción).
- **Para girar 90° frena antes y llega corrido.** `giros.py` busca el corrimiento (s, l) con la
  mayor holgura, extendiendo el giro ±3° por la duda del rumbo. En una esquina en L la mejor opción
  queda en s = −4 cm, l = +4 cm con 22 mm de holgura (C).
- Si el robot no llegó con el lateral pedido hace un **corrimiento lateral** como al estacionar
  (gira 20°, avanza unos centímetros, vuelve a alinear), verificando la huella en cada paso contra
  el mapa. Si no hay lugar, se aleja por el lado libre y vuelve a llegar; después de dos intentos,
  ese giro se marca imposible y se busca otro camino.

### 3.2 Paredes con votos

Cada lectura de un lado vota (2 si se midió de cerca, 1 de lejos) y el estado es el signo de la
suma. Una lectura mal asignada (por un error de pose de unos centímetros) no da vuelta un lado que ya
se midió varias veces.

### 3.3 Localización sin IMU

- **Lateral:** ultrasonidos de costado contra paredes conocidas (también a una celda de distancia,
  para zonas de 2 celdas de ancho). Se descartan lecturas más de 3 cm distintas de lo esperado.
- **A lo largo:** ultrasonido de frente contra una pared confirmada (solo hasta donde el cono de 15°
  no puede tocar los costados, aun con 3° de error de rumbo) y **bordes de aberturas**: cuando un
  ultrasonido lateral pasa de pared a abierto, el sensor está en el borde de la celda. En el
  simulador el cambio se ve 2,6 cm después del borde (0,20 m × tan 7,5°, C); con el sensor real
  hay que medirlo (`offset_borde`).
- **Rumbo:** pendiente de la posición lateral medida con las paredes mientras avanza, restando lo
  que giró según la odometría (regresión lineal; así sirve aunque el control esté corrigiendo el
  rumbo en ese tramo).
- **Factor de giro:** un 4WD patina al girar; el factor se calibra (parámetro `factor_giro`) y se
  ajusta en marcha con cada corrección de rumbo.

### 3.4 Obstáculos

- Un eco de frente que no cae en un borde de celda es una **sospecha**. Antes de entrar a una celda
  nueva que no se vio de lejos, el robot la **barre**: al girar hacia ella se pasa 15° y vuelve, así
  el ultrasonido de frente la recorre entera sin pararse.
- Supuesto (P): los obstáculos son de base cuadrada o circular (bloques de 8 y 20 cm, pilares de
  40 mm, canastos), así que la profundidad es igual al ancho medido; y un obstáculo cerca de una
  pared se toma pegado a ella (entre los dos no pasa el robot).
- Los pilares de los cubos son **cilindros negros** (equipo, 29/10; 40 mm de diámetro, P). La cámara
  no los ve, pero ve el cubo de arriba: cuando un cubo se confirma, su pilar entra al mapa debajo de
  él y se mueve con el promedio de las vistas.
- Si el robot pide moverse y las ruedas no se mueven en 0,8 s, está **trabado** contra algo que no
  vio: vuelve 5 cm y replanifica; a la tercera vez en un paso nuevo, lo cierra; si se traba dos
  veces en la maniobra de un giro, ese giro se marca imposible (ver teoría, 4.4).
- Al pasar, los ultrasonidos laterales miden la cara del obstáculo y corrigen su ancho.
- El control lateral busca, en las celdas que ocupa el robot y en las dos siguientes, el tramo libre
  más cercano al eje, con 2,5 cm de margen si entra y hasta 0,5 cm (yendo a 0,06 m/s) si no.

### 3.5 Cámara

- **Flechas:** el rayo del cuello de la flecha se cruza con las caras de los bloques. Se vuelven a
  proyectar punta, cuello y cola sobre cada cara candidata y se queda la que da una punta de 50 mm
  (reglamento). Sobre esa cara se miden en milímetros el cuerpo (30 / 60 / 100 → largo 1 / 2 / 3) y
  hacia qué lado apunta a lo largo de la pared: sirve con perspectiva y con inclinación de ±30°.
  Se usa una flecha vista igual en 2 imágenes.
- **Cubos y canastos:** la distancia sale del borde de abajo (apoyan a 108,5 mm y 88,5 mm, la cámara
  está a 138,5 mm, O/P). Un cubo cerca de un canasto conocido se ignora, y una detección con la
  forma vertical de un canasto (razón entre los bordes de arriba y de abajo respecto del horizonte:
  0,33 un cubo, 0,6 un canasto) no se toma como cubo.
- **Zonas rojas:** el contorno rojo del piso es un rectángulo: la envolvente convexa de lo que se ve
  queda adentro, y votan las celdas de su interior (una línea suelta no alcanza).
- Antes de entrar a un pasillo nuevo, mientras falten cubos, mira 43° a cada lado (los cubos están
  pegados a las paredes de los costados y con 60° de campo no se ven hasta estar a 2 celdas).
- Las imágenes tomadas mientras el robot gira no se usan (la pose no es confiable).

## 4. Tópicos

| Tópico | Tipo | Sentido |
|---|---|---|
| `/sim/odom` | `nav_msgs/Odometry` | entrada (odometría; en el robot real, la de los encoders) |
| `/sim/ultrasonic/{front,left,right}` | `sensor_msgs/Range` | entrada |
| `/sim/joint_states` | `sensor_msgs/JointState` | entrada (ángulo del servo de cámara) |
| `/vision/detecciones` | `std_msgs/String` (JSON) | visión → navegador |
| `/mision/pedido` | `std_msgs/String` (JSON) | navegador → brazo: `{"accion": "agarrar"/"soltar", "color", "x", "y", "z"}` en `base_link` |
| `/mision/resultado` | `std_msgs/Bool` | brazo → navegador |
| `/sim/cmd_vel` | `geometry_msgs/Twist` | salida |
| `/sim/camera_pan/cmd` | `std_msgs/Float64` | salida |
| `/nav/estado`, `/nav/mapa`, `/nav/mapa_rviz` | | diagnóstico |

Poses de manipulación (P, a confirmar con MAN): para agarrar, el robot queda en la celda del cubo,
alineado con el pasillo, con el cubo al costado a la altura del eje del brazo (`agarre_dx`); para
soltar, de espaldas al canasto con su centro 0,19 m detrás del robot (`deposito_dist`).

## 5. Cómo probar

Sin ROS (cualquier PC con Python 3, `numpy`, `opencv-python`, `pyyaml`). `PYTHONHASHSEED=0` hace
que dos corridas con la misma semilla den exactamente lo mismo (el orden de los conjuntos de Python
cambia entre procesos):

```bash
cd ros2_ws/src
export PYTHONHASHSEED=0
PYTHONPATH=caid_navigation:caid_vision python3 -m caid_navigation.simulacion.correr e2_bifurcaciones --detalle --mapa
PYTHONPATH=caid_navigation:caid_vision python3 -m caid_navigation.simulacion.lote e1 e2 e3 e4 e5 e6 --semillas 1 2 3
PYTHONPATH=caid_navigation:caid_vision python3 -m caid_navigation.simulacion.lote e2 --vision render
cd caid_navigation && python3 -m pytest test/test_navegacion.py -q
```

`--vision oraculo` usa detecciones perfectas; `--vision render` dibuja la imagen y corre el detector
de verdad (más lento, unos 50 ms por imagen).

En Gazebo:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch caid_navigation mision.launch.py laberinto:=e2_bifurcaciones
```

## 6. Resultados en el simulador 2D (S, 29/10/2026, versión final del día)

Misión completa = 3 cubos en su canasto, robot terminado en la zona de partida (a 2 celdas o menos
de la S) y ningún error. "Oráculo" son detecciones perfectas; "render" dibuja la imagen y corre el
detector real. Odometría con 1 % de error de avance y 1,5 % de giro, ultrasonidos con ruido de 5 mm.
`tiempo_max` = 900 s. Con `PYTHONHASHSEED=0` los números se repiten exactos.

| Escenario | Oráculo (3 semillas) | Render (2 semillas) | Tiempo de una completa | Falla típica |
|---|---|---|---|---|
| E1 serpentina | 2/3 | 0/2 | 840-850 s | lento (muchos giros): entrega 2 y se le acaba el tiempo |
| E2 bifurcaciones | 3/3 | 2/2 | 430-475 s | — |
| E3 obstáculos | 2/3 | 2/2 | 560-610 s | paso de ±1,6 cm entre un bloque y un pilar (ver 7) |
| E4 isla | 2/3 | 1/2 | 435-445 s | 6-8 cm de error de pose en un giro: choques y se queda sin tiempo |
| E5 competencia | 1/3 | 0/2 | 545 s | se traba contra un bloque de 8 cm en la maniobra de un giro (pasos de ±1,5 cm) |
| E6 salida abierta | 0/3 | 0/2 | — | lento (1 cubo en 900 s); en el área abierta la pose llega a 5-9 cm; con render, un "cubo azul" falso a 5 m |

Total: oráculo 10/18, render 5/12 (antes de esta tanda: 8/18 y 5/12, con E5 y E6 casi siempre
"atascados"). Lo que mejoró de verdad es la pose: el error máximo en las corridas que terminan bajó
a 2-3 cm y 6-8° (antes 4-7 cm) y ya casi no hay robots atascados (el vigilante de trabado los
saca). Los resultados por escenario se mueven bastante de una semilla a otra: con 2-3 semillas, una
corrida de diferencia no es una tendencia.

## 7. Pendiente y riesgos

- **Probar en Gazebo.** El simulador 2D no reemplaza a Gazebo (física de las ruedas, render real).
  Lo primero: `e2_bifurcaciones` con `mision.launch.py`.
- **Robot real:** medir `offset_borde` (cambio pared/abierto del ultrasonido lateral), el cono real
  del HC-SR04, el factor de giro y la altura/inclinación de la cámara (un grado de inclinación corre
  3 cm la proyección al piso a 0,5 m, C).
- **Brazo:** las poses de agarre y descarga son propuestas; definirlas con MAN.
- **Escenario E3:** el bloque de 8 cm en la fila 4, columna 14 y el soporte del cubo verde de la
  columna 15 quedan en paredes opuestas a 0,27 m uno del otro, menos que el largo del robot (0,278 m):
  queda ±1,6 cm de holgura lateral (C). El validador de `gen_maze.py` mira cada celda por separado;
  conviene que revise también celdas seguidas.
- El tiempo de la misión en el simulador está entre 7 y 15 minutos según el laberinto; hay que
  confirmar el tiempo de la ronda con la organización (`tiempo_max`).
- **Área abierta (E6):** sin paredes la pose solo sale de la odometría y al volver a la entrada el
  robot llega corrido 5-9 cm. Idea para probar: usar los canastos (posición conocida por la cámara)
  como referencias para corregir la pose en la zona de partida.
- **Inclinación de la cámara:** la distancia a cubos y canastos y el filtro de forma de cubo
  (`forma_de_cubo`) dependen del horizonte de la imagen (`cy`): 1° de inclinación lo corre 9,7 px
  (554 × tan 1°, C). Hay que calibrarlo en el robot real antes de confiar en la cámara.
- **Cubo en el 2.º piso:** los límites de carga (0,15 m/s, 0,9 rad/s, 0,4 m/s², 2 rad/s²) salen de
  suponer el cubo a 0,10 m del centro de giro y μ = 0,3 (P). Falta la foto o la medida de MAN.
- **Pilares:** se supuso 40 mm de diámetro (`pedestal_lado_mm`); confirmar con la organización.

## Historial

- 29/10/2026: primera versión.
- 29/10/2026: puntos guardados (partida, entrada, primera bifurcación), reingreso después de entregar, reposo con 3 entregados, límites de carga, pilares negros cilíndricos, vigilante de trabado, rumbo por regresión, filtro de forma de cubo; teoría en `navegacion-teoria.md`.
