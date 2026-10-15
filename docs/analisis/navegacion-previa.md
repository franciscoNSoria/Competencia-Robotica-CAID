# Análisis del trabajo previo de navegación

**Fecha:** 14/10/2026 · **Repos analizados:** `ros2-navegacionautonoma` (commit `ed523fc`, 10/07/2026, "laberinto resuelto — Tremaux acotado, deteccion de meta y fix de GIRO_U") y `tp2-ros2-packages` (commit `01eb885`, 20/06/2026, versión del TP2).

**Abreviaturas** (rutas dentro de los repos [`franciscoNSoria/ros2-navegacionautonoma`](https://github.com/franciscoNSoria/ros2-navegacionautonoma) @ `ed523fc` y [`franciscoNSoria/tp2-ros2-packages`](https://github.com/franciscoNSoria/tp2-ros2-packages) @ `01eb885`; "l." = línea):

| Alias | Archivo |
|---|---|
| WFN | `ros2-navegacionautonoma/src/my_robot_navigation/my_robot_navigation/wall_following_node.py` (603 l.) |
| WFN‑TP2 | `tp2-ros2-packages/my_robot_navigation/my_robot_navigation/wall_following_node.py` (322 l.) |
| MB / CP / GZX | `ros2-navegacionautonoma/src/my_robot_description/urdf/{mobile_base, common_properties, my_robot_gazebo}.xacro` |
| BR / EKF / LB | `ros2-navegacionautonoma/src/my_robot_bringup/{config/gazebo_bridge.yaml, config/ekf.yaml, launch/bringup.launch.xml}` |
| W2 / W1 | `ros2-navegacionautonoma/src/my_robot_bringup/worlds/{maze_2, maze}.world` |

## Resumen

- **Qué es el nodo.** Un monolito de 603 líneas que lee `/scan` y `/odometry/filtered`, publica `/cmd_vel` y, en el mismo callback, percibe paredes, mantiene el mapa, decide y controla.
- **Qué resolvió.** `maze_2`: 21×21 bloques de 0,45 m con 5 islas. Combina seguimiento de pasillo, un árbitro "celda menos visitada" (lo que el equipo llamó *Trémaux acotado*) y una meta en coordenadas fijas.
- **Qué vale la pena llevarse:**
  - la matemática de cardinales: girar a cardinal ±90° y re‑sincronizar contra la pared real;
  - la ley de control de pasillo con su análisis de amortiguamiento;
  - la secuencia de maniobra *avance previo → pivote → realineación*;
  - las macros de inercia y los patrones de Gazebo, bridge y launch;
  - `maze_2` como caso de regresión con islas.
- **Qué no sirve tal cual:**
  - todo lo que depende del LiDAR de 360°, de la IMU con EKF o de la cinemática 2WD con rueda loca;
  - conocer de antemano la celda (0,45 m), los límites y la meta. Eso equivale a un mapa preprogramado, que el reglamento prohíbe.
- **Bugs que no hay que arrastrar:**
  - `-inf` convertido en 12 m (l.275);
  - GIRO_U sin timeout y con marcha atrás a ciegas;
  - el bloqueo `ultimo_giro`, que provoca giros en U falsos;
  - timeouts contados en ticks del LiDAR;
  - ningún watchdog.
- **Plan de migración.** 9 tareas, 8 persona‑días, en 3 columnas troncales más 3 tareas de unión. No suman trabajo nuevo: reemplazan partes de DESC‑01, SIM‑01/02 y NAV‑02/03/05/09/10 del plan anterior reutilizando código.

---

## (a) Arquitectura del nodo `wall_following_node`

### a.1 Interfaces

| Tipo | Nombre | Mensaje | Dónde |
|---|---|---|---|
| Suscripción | `/scan` | `sensor_msgs/LaserScan` (depth 10) | WFN l.36‑37 |
| Suscripción | `/odometry/filtered` (EKF ruedas + IMU) | `nav_msgs/Odometry` | WFN l.42‑43 |
| Publicación | `/cmd_vel` | `geometry_msgs/Twist` | WFN l.44 |
| Timers, servicios, parámetros ROS | ninguno | — | — |

- **No tiene lazo propio.** Todo el control corre en `scan_callback` (l.269) a la tasa del LiDAR: 10 Hz (GZX l.47). En TP2 eran 2 Hz.
- **`odom_callback` (l.105‑159)** corre a 30 Hz, la tasa del EKF (EKF l.8). Hace todo esto:
  - calcula el yaw desde el cuaternión (l.106‑110);
  - guarda la posición y acumula distancia y tiempo (l.112‑123);
  - marca la grilla de visitas (l.125‑135);
  - detecta la meta (l.137‑146);
  - captura la referencia de yaw (l.148‑152).
- **Arranque.** Se lanza a mano con `ros2 run` (README de TP2, paso 8). No tiene launch propio y corre sin `use_sim_time`.

### a.2 Parámetros (todos constantes en el código)

| Constante | Valor | Línea | Uso |
|---|---|---|---|
| distancia_pared | 0,20 m | 14 | referencia al seguir una sola pared |
| distancia_frontal | 0,25 m | 15 | frente libre si adelante > 0,25 |
| umbral_lateral | 0,40 m | 16 | lado libre (apertura) si mide más |
| umbral_centrado | 0,30 m | 20 | centrarse solo si ambas paredes < 0,30 |
| vel_lineal / vel_angular | 0,18 m/s / 0,9 rad/s | 21‑22 | crucero / giro |
| dist_peligro / umbral_yaw_torcido | 0,15 m / 7° | 24‑25 | cambio de pesos |
| umbral_pre_giro / umbral_cruce / dist_max_avance | 0,20 / 0,50 / 0,30 m | 54‑56 | avance previo DER |
| umbral_frontal_giro / dist_max_avance_izq / tol_alineacion | 0,23 m / 0,08 m / 3° | 61‑63 | avance previo IZQ |
| kp_paralelo / kp_centro_pg / tol_paralelo / tol_centro_pg / umbral_ve_pared / post_giro_tope | 5,0 / 0,6 / 0,015 / 0,05 / 0,60 / 50 ticks | 72‑77 | POST_GIRO |
| tam_celda | 0,45 m | 89 | grilla de visitas |
| meta_x, meta_y, radio_meta | 4,95; −6,75; 0,15 m | 98‑100 | meta en odom |
| kp_lidar / kp_yaw (locales) | 1,5 / 2 | 355‑356 | pasillo |
| kp_giro (local) | 1,5 | 472, 541 | giros |

### a.3 Cómo "ve" las paredes

`ranges` reemplaza todo valor no finito por 12,0 (l.275). Después toma tres conos por índice fijo (l.277‑279):

- `adelante` = mínimo de los índices 175..179 y 0..4;
- `derecha` = mínimo de 125..144;
- `izquierda` = mínimo de 35..54.

El LiDAR tiene 180 muestras entre 0 y 6,28 rad (GZX l.51‑56), y gz‑sensors calcula el paso como (max − min)/(N − 1) (Lidar.cc, `AngleResolution`). Eso da:

- **Paso:** 6,28/179 = 0,035084 rad = 2,0102°.
- **Conos:** frente −8,2°…+8,0°; derecha −108,7°…−70,5°; izquierda +70,4°…+108,5°, es decir ±19° alrededor del lateral.

Tomar el mínimo de un cono da la distancia **perpendicular** a la pared, sin importar el rumbo, mientras el error de rumbo sea menor que la mitad del cono. Por eso el control lateral era robusto. Con un solo ultrasonido por lado esta propiedad desaparece (ver b).

Las banderas se calculan en l.281‑283: `der_libre` si derecha > 0,40; `izq_libre` si izquierda > 0,40; `frente_libre` si adelante > 0,25.

### a.4 Máquina de estados

```
AVANZAR ──arbitro()──► 'RECTO': se queda en AVANZAR (control de pasillo)
  ├─ 'DER' ─► AVANCE_PREVIO_DER ─► GIRO_DER ──────────────┐
  ├─ 'IZQ' ─► AVANCE_PREVIO_IZQ (AVANZAR→ALINEAR) ─► GIRO_IZQ ─┤
  └─ None  ─► GIRO_U ──────────────────────────────────────┤
                                                           ▼
                                               POST_GIRO ─► AVANZAR
meta alcanzada ─► mision_cumplida: Twist() vacío para siempre (l.271‑273)
VERIFICAR_U: definido (l.553‑565) pero ninguna transición lo alcanza
```

| Estado | Qué hace | Sale cuando | Líneas |
|---|---|---|---|
| AVANZAR | llama a `arbitro()`; con 'RECTO' aplica el control de pasillo | la elección es DER, IZQ o None | 289‑386 |
| AVANCE_PREVIO_DER | avanza a 0,09 m/s corrigiendo hacia la cardinal (kp 1, ±0,36 rad/s) | adelante < 0,20; o adelante > 0,50 con dist ≥ 0,10; o dist ≥ 0,30 → GIRO_DER | 391‑419 |
| AVANCE_PREVIO_IZQ | fase AVANZAR a 0,072 m/s hasta adelante < 0,23 o dist ≥ 0,08; fase ALINEAR: pivotea hasta abs(err) < 3° | → GIRO_IZQ (tope 100 ticks) | 421‑456 |
| GIRO_DER / GIRO_IZQ | objetivo = cardinal más cercana ∓/± 90°; P con kp 1,5 saturado a 0,9 rad/s | abs(err) < 2° o 80 ticks → POST_GIRO | 458‑484 / 532‑551 |
| POST_GIRO | avanza a 0,045 m/s; se pone paralelo con 2 rayos por pared (kp 5) y se centra (kp 0,6); frena si adelante < 0,20 | paralelo (abs(Δ) < 0,015) y centrado (abs(der − izq) < 0,05), o 50 ticks → AVANZAR. Si quedó paralelo, **re‑sincroniza la cardinal** (l.524‑528). Sin pared confiable pasa a AVANZAR directo | 486‑530 |
| GIRO_U | objetivo cardinal + 180°; gira hacia la pared cercana a 0,9 rad/s hasta abs(err) < 45°, después P; va marcha atrás a 0,054 m/s mientras abs(err) > 90° y adelante el resto | abs(err) < 5° → POST_GIRO. **No tiene timeout** | 567‑593 |

Los timeouts están en ticks. A 10 Hz, 80 ticks son 8 s, 100 son 10 s y 50 son 5 s.

### a.5 Control en pasillo (rama RECTO, l.303‑368)

- **Error lateral** (l.312‑325), según el modo:
  - `centrado` (ambas paredes < 0,30): e = der − izq;
  - `sigue-der`: e = der − 0,20;
  - `sigue-izq`: e = 0,20 − izq;
  - `libre`: e = 0.
- **Error de rumbo** (l.331): e_yaw = −error_a_cardinal().
- **Ley de control:** ω = peso_lidar·(−1,5·e) + peso_yaw·(−2·e_yaw), saturada a ±0,45 rad/s, con v = 0,18 m/s (l.355‑364).
- **Pesos adaptativos** (l.341‑352):
  - evasión 0,1/0,9 si min(der, izq) < 0,15 y el yaw está bien (≤ 7°);
  - torcido 0,5/0,5 si el yaw supera 7°;
  - régimen 0,4/0,6 en el resto.
- **Modelo de los comentarios** (l.333‑340): d'' + k_θ·d' + v·k_d·d = 0, con ζ = k_θ/(2√(v·k_d)). Lo recalculé y coincide:
  - régimen: 0,8/(2√(0,18·0,9)) = **0,994**;
  - torcido: 1,0/(2√0,135) = **1,361**;
  - evasión: 0,2/(2√0,243) = **0,203**.
- **Hallazgo nuevo:** en modo `centrado` el error der − izq vale 2·d, la ganancia efectiva se duplica y ζ = 0,8/(2√(0,18·1,8)) = **0,703**. Es subamortiguado, no crítico como dicen los comentarios.
- **Qué rescatar:** el método de sintonía. Hay que recalcularlo con la velocidad y las ganancias del robot real.

### a.6 Detección de intersecciones y callejones

No hay clasificador de intersecciones. En cada tick de AVANZAR el árbitro mira las tres banderas (l.294):

- una **intersección** es cualquier momento en que un cono lateral supera 0,40 m;
- un **callejón** es cuando el árbitro devuelve `None` (l.378‑385). Pasa si las tres salidas están bloqueadas, excluidas por el bloqueo `ultimo_giro` o fuera del recuadro.

**Geometría** (calculada con pasillo de 0,45 m y robot centrado):

- El cono derecho recién supera 0,40 m cuando el centro del robot está 0,129 m dentro de la abertura (0,096 m antes de su centro), y deja de verla a 0,316 m.
- Con el avance mínimo de 0,10 m (l.411), el pivote cae a 0,229 m, prácticamente en el centro de la celda (0,225 m).
- Esa coincidencia es la que hace funcionar los giros, y depende del LiDAR centrado y de la celda de 0,45 m.

**Debounce:** después de doblar, `ultimo_giro` impide volver a doblar al mismo lado (l.220‑222) hasta acumular 15 ticks de RECTO (l.304‑306). Son 1,5 s, unos 0,27 m a 0,18 m/s. En TP2 eran 2 ticks a 2 Hz.

### a.7 Trémaux "acotado"

- **Grilla:** un dict `visitas{(i,j): n}` con celda = round(pos/0,45) en el marco odom (l.89‑90, 126‑127). Supone que el robot nace en el centro de una celda con los ejes de odom alineados al laberinto (comentario l.86‑88).
- **Marcado:** se cuenta al entrar a una celda nueva, no por tick (l.128‑130).
- **Vecinas:** `rumbo_grilla()` cuantiza el yaw a múltiplos de 90° (l.185‑189) y `visitas_vecina()` lo rota para DER e IZQ (l.199‑212).
- **"Acotado"** (l.191‑197): se excluyen las vecinas fuera de i∈[0,20], j∈[−19,1], que es el recuadro de maze_2 en odom. Hace falta porque el robot nace en la **única abertura del borde** de maze_2 (celda de mundo (10,−9)); sin esta frontera virtual podría salir.
- **`arbitro()`** (l.214‑235):
  - entre las direcciones libres, no bloqueadas y dentro del recuadro, elige la vecina con menos visitas;
  - en empate, prioridad DER > RECTO > IZQ (mano derecha);
  - volver atrás solo es posible con la U, cuando no hay opciones.
- **No es Trémaux clásico.** Trémaux marca **pasajes** (aristas) y tiene la regla de volver si se llega por un pasaje nuevo a una intersección ya visitada. Esto es una heurística de "celda menos visitada" con desempate por mano derecha. Funcionó en maze_2, pero no garantiza cubrir todo el laberinto ni terminar si la meta no existe.

### a.8 Detección de meta

- **Coordenadas fijas en odom** (l.96‑100): mundo (−0,45; 2,70) → odom (4,95; −6,75) → celda (11, −15).
- **Verificación:** el robot nace en (4,50; −4,05) con yaw π (LB l.36). Δ = (−4,95; 6,75); rotado −π da (4,95; −6,75) ✓.
  - Esto funciona porque la IMU de gz reporta la orientación **relativa a la pose inicial** (gz‑sim `Imu.cc` l.221‑232), así que odom queda rotado 180° respecto del mundo.
  - El comentario de l.87 ("ejes de odom paralelos al mundo") es impreciso: los ejes son paralelos a la pose inicial.
- **Condición de llegada:** distancia < 0,15 m (l.139‑141). Al cumplirse, imprime métricas y publica `Twist()` vacío para siempre (l.271‑273).
- **Marcadores invertidos en W2:** el robot nace sobre `goal_marker` (W2 l.5391‑5393) y la meta es `start_marker` (W2 l.1372‑1374).

### a.9 El "fix de GIRO_U" y la evolución desde TP2

**Qué cambió en GIRO_U.** Según el diff, el único cambio dentro del estado es la **salida**:

- en TP2 terminaba en AVANZAR (WFN‑TP2 l.312‑313);
- ahora pasa por POST_GIRO (WFN l.591‑593), que endereza contra las paredes reales y re‑sincroniza la cardinal antes de volver a decidir.

**Qué ya existía.** El objetivo cardinal + 180° (en TP2 estaba inline en l.294‑297; ahora usa `cardinal_mas_cercana()`, l.179‑183) y la marcha atrás en la primera mitad (TP2 l.308‑311).

**Qué cambió alrededor.** La U ahora también se dispara cuando el árbitro descarta todas las salidas por la frontera virtual.

Otros cambios entre TP2 y la versión nueva:

| Tema | TP2 | Nuevo |
|---|---|---|
| Fuente de yaw | `/odom` del DiffDrive (TP2 l.32‑33) | `/odometry/filtered` del EKF ruedas + IMU (l.42‑43; EKF l.23‑40) |
| Decisión | mano derecha pura DER > RECTO > IZQ (TP2 l.114‑189) | árbitro "menos visitada" acotado (l.214‑235) |
| Objetivo de giro | yaw_actual ± 90°; el error se acumula (TP2 l.223‑224) | cardinal ± 90° (l.464‑465, 536‑537) |
| POST_GIRO | 3 ticks rectos (TP2 l.245‑250) | 2 rayos por pared, centrado y re‑sync (l.486‑530) |
| Giro a la izquierda | directo (TP2 l.175‑180) | avance previo de 2 fases (l.421‑456) |
| Avance previo DER | v 0,18; umbral 0,30; tope 0,15 (TP2 l.44‑46, 202) | v 0,09; 0,20; 0,30; avance mínimo 0,10 (l.54‑56, 398, 411) |
| Pesos de control | 2 regímenes (TP2 l.151‑157) | 3 regímenes con análisis de ζ (l.333‑352) |
| Debounce | 2 ticks (TP2 l.123) | 15 ticks (l.305) |
| LiDAR | 2 Hz | 10 Hz |
| Meta y métricas | no había | distancia, tiempo, celdas y meta (l.116‑146) |
| Mundo | maze.world (de borde a borde) | maze_2.world (meta interior junto a una isla) |
| TF odom | puenteado desde gz | DiffDrive con `publish_odom_tf=false`; el EKF publica `odom→base_footprint` |

### a.10 Por qué hizo falta "Trémaux": maze_2 tiene islas

Analicé W2 con un script propio sobre las poses de los bloques. Resultados:

- **Tamaño:** 234 bloques de 0,45 × 0,45 × 0,28 m en una grilla de 21×21 (centros en ±4,5 m; exterior de 9,45 × 9,45 m). Hay 207 celdas libres y una sola abertura en el borde.
- **Ciclos:** el grafo de celdas libres tiene V = 207, E = 215 y C = 1, así que hay E − V + C = **9 ciclos independientes**. Son 4 "salas" de 2×2 y **5 islas** de pared (5, 7, 9, 11 y 13 bloques). Por grado: 173 celdas de pasillo, 25 T, 9 callejones y ninguna cruz.
- **Seguir paredes falla:** la meta (celda (−1,6)) solo toca la isla de 13 bloques. Simulando sobre la grilla desde la abertura, la mano derecha y la mano izquierda vuelven al estado inicial a los 295 pasos **sin llegar**. En W1 (TP2) las dos llegan (148 y 60 pasos), porque la meta está en el borde.
- **Costo de explorar:** el árbitro del equipo, simulado sobre la grilla (sin bloqueo ni ruido), llega tras **326 movimientos de celda (≈ 146,7 m)** y recorre las 207 celdas. El camino más corto (BFS) mide **44 celdas (19,8 m)**: explorar costó 7,4 veces más.

Esto respalda la política de la misión: mapear la primera vez y usar el camino más corto en los viajes siguientes.

> Verificación independiente (15/10/2026): un segundo script sobre la grilla ASCII de la sección e.3 reprodujo las cifras: 234 bloques, V = 207, E = 215, 9 ciclos, camino más corto de 44 celdas (19,8 m), y el seguimiento de pared con mano derecha o izquierda vuelve al estado inicial a los 295 pasos sin llegar a la meta.

---

## (b) Supuestos que no valen en la competencia

| Supuesto previo | Dónde | Por qué no vale | Qué lo reemplaza |
|---|---|---|---|
| LiDAR 360°, 180 rayos, 0,12‑12 m, σ 1 cm, a 0,107 m de altura | GZX l.41‑70; WFN l.275‑283 | El reglamento prohíbe LiDAR. El robot tiene 3 HC‑SR04: frente y esquinas delanteras | `range_filter` → estado de paredes (`/walls`) con semántica REP‑117 y aperturas sostenidas N lecturas. Un solo haz por lado (~15° según la hoja de datos, dato no re‑verificado) |
| El mínimo del cono da la distancia perpendicular sin importar el rumbo | l.277‑279 | Un ultrasonido lateral fijo mide d/cos θ y pierde el eco en paredes oblicuas | La distancia lateral solo es válida con el rumbo controlado. Descartar saltos. El rumbo sale de otra fuente (filas siguientes) |
| Paralelismo con 2 rayos por pared | l.237‑267, 486‑530 | Hay un solo sensor por lado | Rumbo relativo a la pared con θ ≈ atan(Δd/Δs) en ventanas de ≥ 5 cm de encoders, más escuadrado contra la pared frontal |
| Sensor en el centro del robot | MB l.199‑204 | Los laterales están en las esquinas delanteras, x_s por delante del eje | Corregir el evento "apertura" con x_s (de `dimensions.yaml`). El avance previo sale de geometría + encoders, no de 0,10/0,30 m fijos |
| Marcha atrás en la U (≈ 9,4 cm a ciegas) | l.587‑590 | No hay sensor trasero | U = 2 × 90° en el lugar con realineación. Marcha atrás solo acotada por encoders y con timeout |
| Yaw de la IMU fusionado por EKF | EKF; MB l.206‑219; GZX l.77‑92 | La plataforma no tiene IMU y una "IMU extra" está prohibida | Odometría con 4 encoders FC‑03 (un canal, sin sentido: el signo sale del comando) + snapping a k·90° contra paredes en `map→odom` |
| 2WD + rueda loca, separación 0,1605 m, r 0,033 m | MB l.53‑54; GZX l.6‑17 | El robot es 4WD con motores TT (skid‑steer): patina en los giros en el lugar | Cinemática diferencial con b_eff calibrada (mayor que la geométrica). En Gazebo, DiffDrive con 2 joints por lado y fricción lateral baja. Detectar patinaje comparando las ruedas del mismo lado |
| Celda de 0,45 m, pared de 0,45 × 0,28, grilla 21×21, robot nacido en el centro de una celda y alineado | l.89, 126‑127, 191‑197 | Las medidas no se conocen y cambian entre fases | Mapa topológico: nodos en intersecciones, aristas con largo medido y rumbo k·90°. Ancho de pasillo estimado online. Umbrales como fracción del ancho medido. `cell_size` solo como prior |
| Meta en coordenadas fijas | l.96‑100, 137‑146 | Es un mapa preprogramado, prohibido | Objetivos dinámicos desde percepción (cubo, depósito, zona de inicio) anotados en el grafo; decide `mission_manager` |
| Recuadro fijo ("acotado") | l.191‑197 | Mismo problema que la meta | Nodos BOUNDARY en el grafo (área abierta o zona de inicio) |
| Lazo sincronizado con `/scan` a 10 Hz, en tiempo de simulación | l.269 | Los datos reales llegan asíncronos y con latencia por Wi‑Fi | Timer fijo de 20 Hz, timeouts en segundos y watchdog en la PC, en el emulador y en la ESP32 |
| Robot de 0,185 × 0,145 m sin brazo ni cámara | MB l.55‑57 | El robot real es más grande y lleva brazo y cámara con pan | Radio barrido desde `dimensions.yaml`; verificar el ancho mínimo para girar |

La **cámara con pan** no reemplaza a los sensores de distancia. Aporta flechas (dirección, largo, color en bifurcaciones), cubos, depósitos y zona de inicio, que pasan a ser eventos y etiquetas del grafo. Las flechas sirven para acelerar la exploración.

---

## (c) Qué reutilizar, qué adaptar, qué reescribir

**Reutilizar tal cual** (con limpieza de estilo y tests):
- `normalizar_angulo` (l.161‑163) y el yaw desde el cuaternión (l.106‑110).
- `error_a_cardinal` y `cardinal_mas_cercana` (l.165‑183): snapping a múltiplos de 90° relativo a una referencia.
- El patrón "girar a cardinal ± 90°, nunca a yaw ± 90°" (l.461‑465, 535‑537) y el P saturado con tolerancia (l.467‑484).
- La re‑sincronización de la referencia cuando se confirma paralelismo (l.522‑528). En el diseño nuevo se aplica como corrección `map→odom`.
- Las macros de inercia (CP l.14‑34).
- Los patrones de launch y bridge (ver e).

**Adaptar:**
- **Control de pasillo** (l.308‑368): misma estructura y mismo análisis de ζ. Cambian las entradas (`/walls`), las ganancias (recalculadas para la v real) y el término de rumbo (encoders + Δd/Δs).
- **Primitivas de giro** (AVANCE_PREVIO_*, GIRO_*, POST_GIRO): se mantiene la secuencia avance → alinear → pivotear → realinear. Cambian a distancias de encoders, compensación de x_s, timeouts en segundos y un POST_GIRO basado en Δd/Δs.
- **GIRO_U:** 2 × 90° con realineación, con marcha atrás acotada.
- **Árbitro "menos visitado + mano derecha":** queda solo como desempate dentro de Trémaux sobre el grafo, con marcas 0/1/2 por arista.
- **Métricas de misión** (l.116‑135): distancia, tiempo y nodos explorados, publicadas en `/nav/metrics` y en el log para la memoria técnica.
- **URDF, gazebo.xacro, bridge, launch y display.launch** (ver e).
- **Mundos:** maze_2 como regresión con islas, convertido a ASCII/YAML para el generador.

**Reescribir:**
- **Arquitectura:** separar en `range_filter`, `maze_navigator` (comportamientos), `junction_detector` y `topo_map`/explorador. La lógica pura, sin rclpy, va en `caid_navigation/core/` con pytest.
- **Parámetros:** `declare_parameter` + YAML.
- **Mapa:** de dict de celdas a grafo topológico.
- **Meta y límites:** eventos de percepción.
- **Lazo de control:** timer + watchdog.

**No migrar:** las mallas STL del 2WD, el EKF, la IMU, la rueda loca, `Lidar_link` y los paquetes del curso (`paquete_cpp`, `paquete_py`, `tp1_*`).

---

## (d) Bugs y code smells

| ID | Sev. | Dónde | Problema | Consecuencia | Arreglo en el monorepo |
|---|---|---|---|---|---|
| B1 | Alta | WFN l.275 | Todo valor no finito pasa a 12,0 m. Por REP‑117, `-inf` significa "demasiado cerca", y el gpu_lidar de gz enmascara lo fuera de rango como ±inf (GpuLidarSensor.cc l.198‑202) | Una pared a menos de 0,12 m (range min, GZX l.59) se ve **libre**. Pasa con 2,75 cm de holgura al frente o 4,75 cm al costado | `-inf` → bloqueado, `+inf` → libre, `NaN` → inválido; con test |
| B2 | Alta | l.567‑593 | GIRO_U no tiene timeout (los demás estados sí: l.454, 482, 549, 521) y va marcha atrás a ciegas mientras abs(err) > 90° (≈ 9,4 cm) | Si la cola toca una pared, el robot queda trabado para siempre. En el robot real no hay sensor trasero | Timeout en segundos, U = 2 × 90° y marcha atrás acotada por encoders |
| B3 | Media | l.220‑222, 304‑306, 378‑385 | `ultimo_giro` excluye un lado durante 15 ticks (≈ 0,27 m). Si ese lado es la única salida, el árbitro devuelve None | Giro en U falso en curvas en S u horquillas | Debounce por distancia de odometría. "Bloqueado por debounce" nunca debe convertirse en U: esperar o avanzar |
| B4 | Media | l.126‑130 | Cuenta la celda al cambiar de celda, sin histéresis | Oscilar sobre un borde (POST_GIRO, marcha atrás de la U) infla las visitas y sesga el árbitro | Marcas por arista en el grafo, disparadas por eventos de intersección |
| B5 | Media | l.269 (sin timer); GZX l.6‑17 | El control solo corre cuando llega `/scan`, y el DiffDrive no tiene timeout de cmd_vel (DiffDrive.cc) | Si `/scan` se corta, el robot sigue con la última orden | Timer de 20 Hz + watchdog de datos viejos; parada en el emulador y en la ESP32 |
| B6 | Media | l.77/521, 305, 454, 482, 549 | Timeouts y debounce medidos en ticks del callback | 80 ticks son 40 s a 2 Hz y 8 s a 10 Hz: cambian solos con la tasa | Tiempos en segundos con `get_clock()` |
| B7 | Media (reglamento) | l.96‑100, 191‑197 | Meta y recuadro hardcodeados | Mapa preprogramado, no válido en CAID | Objetivos y bordes como eventos del grafo |
| B8 | Baja | l.168‑169, 202‑203 | Sin odometría, el error a la cardinal y las visitas valen 0 sin avisar | Modo degradado silencioso | Estado WAIT_INPUTS y diagnóstico |
| B9 | Baja | l.245‑248, 277‑279 (comentario l.69‑70) | Índices fijos que suponen 180 muestras desde 0 rad; el índice 135 es −88,6°, no −90° | El paralelismo converge ~1,4° torcido a la derecha y ~0,5° a la izquierda, y la re‑sync arrastra ese sesgo | Calcular los índices con `angle_min` y `angle_increment` (la lección vale para cualquier sensor) |
| B10 | Baja | l.188 vs. l.171 | `rumbo_grilla` usa el yaw absoluto; `error_a_cardinal` usa el relativo a la referencia | Solo funciona si odom nace alineado con el laberinto | Un único marco `map` con rumbo k·90° |
| B11 | Baja | l.31, 34, 388 (`frente_libre`), 553‑565 | Código muerto: `error_anterior`, `dist_retroceso_u`, un parámetro sin uso y el estado VERIFICAR_U, inalcanzable | Ruido al leer | Borrar |
| B12 | Baja | l.13‑100, 355‑356, 411, 472, 510, 541 | Todo hardcodeado, con números mágicos (0,20 en l.510 en vez de `umbral_pre_giro`) | Sintonizar exige editar el código | Parámetros ROS + YAML |
| B13 | Baja | l.386 | Al decidir DER, IZQ o U publica un Twist en cero ese tick | Tirón de 0,1 s en cada decisión | Que la maniobra arranque en el mismo tick |
| B14 | Baja | todo el archivo | flake8 con la config de ament (máx. 99 columnas) da 38 hallazgos: 24 E221, 5 E501, 2 E302, 2 E222, W291, W292, E305, E265 y E114 | `colcon test` falla (test_flake8 está activo), así que el CI del monorepo quedaría rojo | Formatear y pasar ament_flake8 y pep257 antes del PR |
| B15 | Baja | l.86‑88 | El comentario dice que la IMU ancla los ejes de odom "paralelos al mundo" | Confunde: gz reporta la orientación relativa a la pose inicial, y acá odom está rotado 180° | Documentar los marcos (REP‑105) |
| B16 | Baja | toda la clase | God class: percepción, mapa, decisión, control y métricas juntos | No se puede testear por partes | La separación descrita en (c) |

---

## (e) URDF, Gazebo y launch

### e.1 Patrones que conviene reutilizar

1. **Macros de inercia** (CP l.14‑34). Las fórmulas son correctas:
   - caja: m/12·(y² + z²) y equivalentes;
   - cilindro: m/12·(3r² + h²) en los ejes transversales e Izz = m·r²/2;
   - el inercial se gira con `o_rpy` igual que la colisión (MB l.110‑111).

   Destino: `caid_description/urdf/inertia.xacro`, agregando la esfera.
2. **`base_footprint` sin inercia → `base_link` fijo** (MB l.76, 244‑248), según REP‑105.
3. **Colisiones primitivas con visual STL** (MB l.86‑91, 104‑109).
4. **Bloque de propiedades geométricas** (MB l.53‑57), pero leídas de `hardware/cad/dimensions.yaml` con `xacro.load_yaml`.
5. **DiffDrive con `frame_id`, `child_frame_id` y `publish_odom_tf=false`** (GZX l.6‑17), con el TF del gz sin puentear (BR l.14‑18). Así hay una sola fuente de `odom→base_footprint`, que en el diseño nuevo publica `odom_node`. Para el 4WD:
   - agregar 2 `left_joint` y 2 `right_joint`. DiffDrive los acepta; todos reciben la misma velocidad y la odometría interna usa solo el primero de cada lado;
   - agregar `max_velocity` y `max_acceleration`.
6. **JointStatePublisher con lista explícita de joints** (GZX l.21‑29). Agregarle `<topic>/sim/joint_states</topic>` (parámetro soportado) para que el emulador de firmware calcule los ticks.
7. **gpu_lidar con `gz_frame_id` y ruido** (GZX l.41‑70), como plantilla de los 3 ultrasonidos: pocos rayos en un cono angosto, puenteados como `sensor_msgs/Range`. ros_gz_bridge convierte LaserScan → Range tomando el mínimo de los rayos, pero fija `radiation_type` en INFRARED, así que el emulador debe republicar con ULTRASOUND.
8. **Sistema Sensors con ogre2** (GZX l.33‑38).
9. **Bridge YAML** (BR): `/clock` GZ_TO_ROS, `/cmd_vel` ROS_TO_GZ y `lazy: false` en los sensores.
10. **Launch** (LB):
    - robot_state_publisher con `xacro` y `type="str"` (l.17‑22);
    - `gz_sim.launch.py` con `-r <mundo>` (l.29‑31);
    - `ros_gz_sim create -topic robot_description` (l.35‑36);
    - `parameter_bridge` con el YAML (l.39‑41);
    - `display.launch.xml` con `joint_state_publisher_gui`.
11. **Mundos SDF** con paredes `box` estáticas y piso claro (W2 l.21‑70): fáciles de generar por script.

### e.2 Qué corregir

- **Macro sin usar** (MB l.62‑72): `mesh_link` está definida y nunca se usa. Los args `package_name` y `mesh_dir` (l.5‑6) solo se usan ahí, y los links repiten `package://my_robot_description/...` a mano (l.82, 100, 118, 136, 157). En el robot nuevo, usar una macro de rueda instanciada 4 veces.
- **Detalles del URDF:** `<axis>` en un joint fijo (MB l.203) y un material con nombre vacío (MB l.181).
- **Rueda loca flotando:** con los valores de MB, la esfera queda 4,79 mm sobre el piso (0,04675 − 0,009 − 0,02296 − 0,01 = 0,00479 m) y el chasis se inclina ~3,0°. En TP2 eran 11,3 mm y 7,1°. El spawn a z = −0,0017 (LB l.36) lo disimula. Lección: calcular las alturas de contacto desde una sola fuente.
- **Masas poco realistas:** el chasis pesa 0,12 kg y cada rueda 0,1403 kg (MB l.92, 110, 128). Pesar el robot real.
- **Sistemas de mundo en el modelo:** Sensors e Imu están declarados dentro del robot (GZX l.33‑38, 78‑82). Moverlos al SDF del mundo.
- **Tópico de joint_state atado al mundo:** BR l.9 lleva el nombre `complex_maze_world`, así que cambiar de mundo rompe `/joint_states`. Usar un `<topic>` fijo.
- **`set_env` pisa `GZ_SIM_RESOURCE_PATH`** (LB l.3‑4). Usar el hook DSV del template oficial: `prepend-non-duplicate;GZ_SIM_RESOURCE_PATH;share;@CMAKE_INSTALL_PREFIX@/share`, registrado con `ament_environment_hooks`.
- **Launch incompleto:**
  - no tiene argumentos (mundo, pose, rviz);
  - no lanza el nodo de navegación;
  - RViz y el nodo corren sin `use_sim_time`.
- **RViz con fixed frame `base_footprint`** (urdf_config.rviz l.191): no se ve el movimiento. Usar `odom` o `map`.
- **package.xml incompletos** (bringup, description y navigation):
  - no declaran `exec_depend` de `ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, `xacro`, `rviz2`, `robot_localization` ni `joint_state_publisher_gui`, así que `rosdep install` (README de TP2, paso 4) no los instala;
  - la descripción y la licencia siguen en "TODO" (bringup l.6‑8, navigation l.6‑8, setup.py l.17‑19).
- **EKF e IMU:** no se migran, porque el robot real no tiene IMU.

### e.3 Datos de los mundos

| | maze_2.world (W2, el que usa LB l.14) | maze.world (W1, TP2) |
|---|---|---|
| Bloque de pared | 0,45 × 0,45 × 0,28 m (centro en z = 0,14) | igual |
| Celda / pasillo | 0,45 m; el espesor de pared es igual al ancho de pasillo | 0,45 m; grilla corrida 0,225 m en x |
| Grilla y tamaño exterior | 21 × 21; 9,45 × 9,45 m | 18 × 15; 8,1 × 6,75 m |
| Bloques / celdas libres | 234 / 207 | 145 / 125 |
| Aberturas en el borde | 1 (el spawn) | 8, y 3 regiones libres desconectadas |
| Ciclos / islas | 9 / 5 | 5 ciclos |
| ¿La mano derecha llega a la meta? | No | Sí |

Grilla ASCII de maze_2, lista para el generador de laberintos. Las filas van de y = +10 (arriba) a −10, las columnas de x = −10 a +10; `#` es pared y `.` libre. La abertura está en la fila −9, columna +10, y la meta del equipo en la fila +6, columna −1.

```
#####################
#.....#.............#
#####.#.#######.###.#
#.....#.......#.#...#
#.#####.#.#.###.#.###
#.#.....###.....#...#
#.#.###.....###.###.#
#.#...#.......#.....#
#.###.#.#####.#####.#
#...#.#.....#.....#.#
###.#.#####.#####.#.#
#...#.....#.....#.#.#
#.#######.#####.#.#.#
#.......#...#...#...#
#######.###.#.#######
#.....#.....#.......#
#.###.###.#######.###
#...#.......#.....#.#
###.###.###.#.#####.#
#.........#.#........
#####################
```

---

## (f) Plan de migración al monorepo

### Mapeo de archivos

| Legado | Destino | Acción |
|---|---|---|
| CP | `caid_description/urdf/inertia.xacro` | copiar y agregar la esfera |
| MB + `my_robot.urdf.xacro` | `caid_description/urdf/caid_bot.urdf.xacro` | reescribir: 4WD con primitivas desde `dimensions.yaml` |
| GZX | `caid_description/urdf/gazebo.xacro` | adaptar |
| `display.launch.xml`, `urdf_config.rviz` | `caid_description/launch/`, `caid_bringup/rviz/` | copiar y ajustar |
| BR | `caid_bringup/config/sim/bridge.yaml` | adaptar |
| LB | `caid_bringup/launch/sim.launch.xml` | adaptar (XML, que el equipo ya domina) |
| W2, W1 | `caid_gazebo/mazes/legacy_maze2.yaml` → `worlds/legacy_maze2.sdf` | convertir |
| WFN | `caid_navigation/core/*.py` + nodos | extraer y reescribir |
| EKF, IMU, rueda loca, Lidar, STL | — | no migrar |

### Tareas por columna

Cada tarea trabaja en una rama `feature/MIG-0x-<desc>` que se integra a `dev` con squash. El commit cita el origen (`franciscoNSoria/ros2-navegacionautonoma@ed523fc`).

**Columna A: descripción y simulación (troncal, puede empezar hoy)**

1. **MIG‑01 · Esqueleto de `caid_description`** (0,5 d)
   - **Qué:** `inertia.xacro` copiado, `display.launch.xml`, hook DSV, `package.xml` con todos los `exec_depend` y tests de linters.
   - **DoD:** `colcon build` y `colcon test` verdes.
   - **Depende de:** nada.
2. **MIG‑02 · `caid_bot.urdf.xacro` 4WD** (1 d)
   - **Qué:**
     - ruedas fl/fr/rl/rr (continuous);
     - `us_front/left/right_link`;
     - `camera_pan_link → camera_link → camera_optical_frame`;
     - colisiones primitivas;
     - cotas provisorias en `dimensions.yaml`, que se corrigen al medir el robot real.
   - **DoD:** `check_urdf` sin errores, árbol TF igual al contrato y robot a escala en RViz.
   - **Depende de:** MIG‑01.
3. **MIG‑03 · `gazebo.xacro`** (1 d)
   - **Qué:**
     - DiffDrive con 2 + 2 joints, `max_velocity`/`max_acceleration` y TF sin puentear;
     - JointStatePublisher en `/sim/joint_states`;
     - 3 gpu_lidar angostos con `gz_frame_id`;
     - cámara y JointPositionController del pan;
     - fricción lateral baja para skid‑steer.
   - **DoD:** teleop en un mundo vacío, el robot gira en el lugar, y Range y cámara publican.
   - **Depende de:** MIG‑02.

**Columna B: mundos (troncal, puede empezar hoy)**

4. **MIG‑04 · `legacy_maze2` en `caid_gazebo`** (0,5 d)
   - **Qué:**
     - grilla ASCII → YAML (celda 0,45 m, alto 0,28 m) → SDF por script;
     - sistema Sensors a nivel de mundo;
     - marcadores renombrados (`spawn`, `island_goal`);
     - una variante escalada (por ejemplo, celda de 0,30 m) para no sobreajustar.
   - **DoD:** `gz sim` carga los dos mundos y el script reproduce W2 bloque por bloque (234 bloques).
   - **Depende de:** nada.

**Columna C: núcleo de navegación (troncal, puede empezar hoy, no necesita simulador)**

5. **MIG‑05 · `caid_navigation/core` puro** (0,5 d)
   - **Qué:**
     - `angles.py`: normalizar, yaw, cardinal, snapping;
     - `turns.py`: P saturado a objetivo cardinal;
     - `corridor.py`: ley de control y cálculo de ζ;
     - `arbiter.py`: menos visitado + mano derecha.
   - **DoD:** pytest con los casos de l.161‑235 y los valores de ζ (0,994 / 1,361 / 0,203 / 0,703); flake8 limpio.
   - **Depende de:** nada.
6. **MIG‑06 · Filtro de ultrasonidos y estado de paredes** (1 d)
   - **Qué:**
     - semántica REP‑117 (corrige B1) y mediana;
     - aperturas sostenidas N lecturas;
     - rumbo relativo por Δd/Δs con encoders;
     - ancho de pasillo estimado.
   - **DoD:** tests con secuencias CSV sintéticas (sin eco, pared oblicua, `-inf`).
   - **Depende de:** MIG‑05 y del contrato de `/walls`.

**Tareas de unión** (se desbloquean cuando se cierran sus padres)

7. **MIG‑07 · `sim.launch.xml` y bridge** (0,5 d)
   - **Qué:**
     - puerto de LB con argumentos `world`, `x`, `y`, `yaw` y `rviz`;
     - hook DSV;
     - `bridge.yaml` sin nombre de mundo;
     - RViz con frame `odom` y `use_sim_time` en todos los nodos.
   - **DoD:** `ros2 launch caid_bringup sim.launch.xml world:=legacy_maze2` levanta todo con un solo comando.
   - **Depende de:** MIG‑03 y MIG‑04.
8. **MIG‑08 · Pasillo y primitivas de giro en simulación** (1,5 d)
   - **Qué:**
     - nodo con timer de 20 Hz, timeouts en segundos y watchdog (corrige B5 y B6);
     - avance previo con compensación de x_s;
     - giros a cardinal ± 90° con realineación por Δd/Δs;
     - U = 2 × 90° (corrige B2) y debounce por distancia (corrige B3).
   - **DoD:** 20/20 giros sin contacto en `legacy_maze2` y 2 m de pasillo sin contacto.
   - **Depende de:** MIG‑07 y MIG‑06, más la odometría por encoders del emulador (`odom_node`, de otra columna).
9. **MIG‑09 · Explorador Trémaux en el grafo + regresión de islas** (1,5 d)
   - **Qué:**
     - Trémaux por aristas (marcas 0/1/2), con el árbitro de MIG‑05 como desempate;
     - nodos BOUNDARY en lugar del recuadro fijo;
     - sin coordenadas de meta.
   - **DoD:**
     - en `legacy_maze2` llega a la celda junto a la isla, donde seguir paredes falla;
     - vuelve al inicio por el camino más corto (44 celdas = 19,8 m);
     - deja un log con distancia, tiempo y nodos.
   - **Depende de:** MIG‑08, más el mapa topológico si ya existe.

**Totales:**
- **Esfuerzo:** 0,5 + 1 + 1 + 0,5 + 0,5 + 1 + 0,5 + 1,5 + 1,5 = **8,0 persona‑días**.
- **Cadena más larga:** MIG‑01 → 02 → 03 → 07 → 08 → 09 = **6,0 días**. Entra con holgura antes del 31/10 si cada columna avanza en paralelo.

**Equivalencias con el plan anterior:**

| Tareas MIG | Tareas del plan anterior |
|---|---|
| MIG‑01 y 02 | DESC‑01 |
| MIG‑03 y 07 | SIM‑01 |
| MIG‑04 | parte de SIM‑02 |
| MIG‑06 | NAV‑02 |
| MIG‑05 y 08 | NAV‑03 y NAV‑09 |
| MIG‑09 | NAV‑05 y NAV‑10 |

---

## Fuentes consultadas (leídas en esta tarea)

- Archivos de los dos repos (en los commits indicados arriba): los abreviados, más los `package.xml`, `setup.py` y `test_*.py` de los dos repos, y el `diff -u` entre ambos.
- gz‑sim8:
  - `DiffDrive.hh` l.36‑44 y 88‑95; `DiffDrive.cc` l.459‑495 y 529‑531;
  - `Imu.cc` l.221‑232;
  - `JointStatePublisher.hh` l.49‑51;
  - `JointPositionController.hh` l.40‑92.

  URL base: `https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/src/systems/...`
- gz‑sensors8: `GpuLidarSensor.cc` l.198‑202 y `Lidar.cc` l.356‑361 y 516‑526 (`https://raw.githubusercontent.com/gazebosim/gz-sensors/gz-sensors8/src/`).
- ros_gz (Jazzy): `ros_gz_bridge/src/convert/sensor_msgs.cpp` l.538‑565, `ros_gz_bridge/README.md` l.65 y 314‑321, `ros_gz_sim/src/create.cpp` l.42‑54.
- `ros_gz_project_template`: `hooks/ros_gz_example_description.dsv.in` y `CMakeLists.txt` l.18.
- REP‑117 (`https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0117.rst`, l.25) y `sensor_msgs/msg/Range.msg` (Jazzy).
- `ament_flake8.ini` (Jazzy).
- Datos del HC‑SR04, la rueda TT y el disco de 20 ranuras: tomados de la investigación previa de hardware, sin re‑verificar.
