# Informe de estado del repositorio: Competencia-Robotica-CAID

> **Addendum del 2026-11-19 (revisión de issues, INF-09 #92).** Tras escribirse este informe se verificó en GitHub y se corrigió el tablero:
> el PR #87 **ya estaba mergeado** (no hay PR abiertos); el issue #78 (FW-06) se cerró; 33 issues quedaron con la etiqueta `en-curso`
> y se tildaron las subtareas verificadas; se crearon #92 a #100 para el trabajo que no estaba en el roadmap. El veredicto sobre cada
> desvío del plan está en [`docs/roadmap/divergencias.md`](../roadmap/divergencias.md).

> Fecha del análisis: 2026-11-19. Base: `origin/dev` (`a823df4`, con los PR #88 y #91 ya mergeados) más las ramas abiertas.
> Cómo se obtuvo cada dato: **G** = leído con `git`/GitHub, **T** = corrido en una copia limpia de `dev` en este entorno, **D** = dicho por el equipo en la conversación, **I** = inferencia mía (queda marcada). **No pude ver** el tablero de GitHub Projects, la memoria técnica completa ni el resultado de la competencia.

---

## 1. Resumen en 10 líneas

1. El repo es un monorepo ROS 2 Jazzy + firmware ESP-IDF + simulación Gazebo con **7 paquetes ROS**, ~11.400 líneas de Python/C en `dev` (ROS 5.800, firmware 3.800, herramientas 1.800) **(G)**.
2. **Lo que está en `dev` es la mitad del proyecto.** El código que corrió en el robot real (movimiento por ruedas, laberinto con cubo, puentes sonar/odometría, brazo) vive en ramas sin integrar: `feature/laberinto-cuenta-giros` (+45 commits sobre `dev`), `integracion` (+28), `feature/tremaux-sin-camara-y-panel-ruedas` (+37), `feature/MAN-brazo-sliders` (+16) **(G)**.
3. La **simulación en Gazebo existe y funciona** (mundos, robot, sensores, generador de laberintos con 38 pruebas que pasan), pero **no tiene el emulador del firmware** (SIM-01) y varios de sus parámetros son provisorios **(G, T)**.
4. La **visión** (flechas, cubos, validación entre cuadros, calibración de lente y de color) está terminada y en `dev` (PR #88). La cámara va girada 180° y eso ya se resuelve en el firmware **(G)**.
5. El **firmware** de movimiento (PI por rueda a 100 Hz, micro-ROS por Wi-Fi) funciona en banco; la versión con freno por sonar está solo en ramas. **La ESP32 clásica se quemó**: hay un port a ESP32-S3 sin compilar ni probar **(D, G)**.
6. **No hay CI** (`.github/` solo tiene la plantilla de PR) y **no hay ADR de navegación** (el último es el 0003) **(G)**.
7. El **tablero** quedó atrasado respecto del trabajo real: quedan ~62 issues abiertos de 72, y varios ya están hechos en la práctica (por ejemplo FW-02 a FW-05 y CAM-04) **(G, I)**.
8. **Brazo:** la cinemática y el driver existen en una rama, pero "el hombro no levantó solo" en las pruebas de banco (PR #85); quedó pendiente una fuente de 5 V más fuerte, que es parte del rediseño de potencia **(G)**.
9. La competencia ya ocurrió; **no sé qué rama corrió ni cómo le fue**. Es el dato que más falta para decidir qué conservar **(D)**.
10. Lo que más acerca al objetivo "simulación = robot real" no es escribir más navegación sino **calibrar la simulación con datos del robot en el piso** (guía aparte: [`calibracion-sim2real.md`](../guias/calibracion-sim2real.md)).

---

## 2. Qué hay en `dev`

### 2.1 Paquetes ROS 2 (`ros2_ws/src/`)

| Paquete | Qué hace | Líneas | Estado |
|---|---|---|---|
| `caid_description` | Robot en xacro/URDF, brazo articulado, plugin de Gazebo (`gazebo.xacro`) | 97 | Completo para el robot con el brazo en la posición original. **Hay que moverlo** (el brazo se desplaza hacia el frente) |
| `caid_gazebo` | 16 mundos (`e1`–`e6` con sus variantes `_simple`, `cudar`, `maze`, `maze_2`, `laberinto_chico`), `gen_maze.py` | 1.721 | Completo. 38 pruebas del generador pasan **(T)** |
| `caid_bringup` | `sim.launch.py`, `robot.launch.xml`, bridge ROS-Gazebo, `scan_to_range` | 241 | Funciona en simulación. Los nodos del robot **real** (`sonar_bridge`, `odom_ticks`, `calibracion_robot`) **no están en `dev`** |
| `caid_navigation` | Mapa de celdas, explorador Trémaux, misión de 3 cubos simulada, marcadores de RViz | 1.909 | Núcleo (`mapa.py`) con 11 de 12 pruebas pasando; la que falla pide `visualization_msgs`, que no hay en este entorno **(T)** |
| `caid_vision` | Detector de flechas y cubos, validador entre cuadros, `camara_node`, calibración de lente y color | 1.725 | Completo. `test_detector` se saltea: depende de `caid_navigation.simulacion`, que **no está en el repo** **(T, G)** |
| `caid_interfaces` | Acciones `Cargar` y otras | 0 líneas de código | Solo definiciones |
| `caid_pruebas` | `esp32_falsa` (simula la ESP32 para probar sin hardware) | 135 | Utilidad de pruebas |

### 2.2 Firmware (`firmware/`)

| Qué | Estado |
|---|---|
| `banco/robot_4ruedas/main.c` | PI por rueda, micro-ROS por Wi-Fi, marcha por distancia, giro en el lugar. **Versión de `dev`: anterior a la que tiene freno por sonar y brazo** |
| `banco/*` (motor_pi_1rueda, prueba_cableado, brazo/...) | Programas de banco, documentados |
| `esp32_cam/` | Firmware de la cámara con servo de giro en GPIO13 |
| Port a ESP32-S3 | Parche `port_s3.patch` aplicado solo en una rama local; **sin compilar** |

### 2.3 Documentación y herramientas
- Guías en `docs/guias/` (12), ADR 0001 a 0003, roadmap, reglamento transcrito, escenarios de prueba, y **solo la sección 09 de la memoria** (de 10).
- `tools/`: calibración de ruedas y teclado de banco, generadores de mundos, `prueba_giro.sh` (calibra el giro en simulación), carga masiva del tablero.
- `candidatos/` (nuevo, PR #91): script para congelar versiones. Todavía sin ninguna versión congelada.

---

## 3. Lo que NO está en `dev` pero existe (ramas)

| Rama | Contenido relevante | Adelanto sobre `dev` |
|---|---|---|
| `feature/laberinto-cuenta-giros` | Stack del robot real: `tools/ruedas/` (panel, laberinto con cubo, seguidor de flechas), `sonar_bridge`, `odom_ticks`, `calibracion_robot`, `caid_manipulation`, firmware de pruebas por componente, `tools/tremaux`, `tools/torneo` | +45 / −7 |
| `integracion` | Mezcla de visión y movimiento (el "sándwich" del día de la competencia, según la conversación) | +28 / −3 |
| `feature/tremaux-sin-camara-y-panel-ruedas` | Variante sin cámara | +37 / −7 |
| `feature/MAN-brazo-sliders` | Brazo con sliders | +16 / −12 |
| `feature/FW-esp32` | Estructura PlatformIO propuesta en otro momento (ADR 0003 eligió ESP-IDF) | +3 / −29 |
| `feature/CAM-04-calibracion`, `feature/CAM-fps` | Reemplazadas por el PR #88 | — |

**Riesgo principal:** el conocimiento de lo que anda en el robot está en ramas que divergen de `dev`. La unión de `laberinto-cuenta-giros` con la visión de `dev` tiene conflictos solo en dos archivos (`tools/ruedas/laberinto_real.py` y `laberinto_ruedas.py`) **(G, simulado con `git merge-tree` antes)**.

---

## 4. Estado por columna del roadmap

Estados: **Hecho**, **Avanzado**, **Parcial**, **No iniciado**. "Evidencia" es lo que vi; el estado en el tablero puede diferir.

| Columna | Estado | Evidencia |
|---|---|---|
| INF (7) | Parcial | README, CONTRIBUTING, ADR 0001–0003, tablero parte A cerrada (#5). Falta CI (#6), tablero parte B (#7), plantilla de memoria (#8), maqueta (#9), consultas a la organización (#3) |
| DESC (4) | Hecho, salvo el brazo | STL, xacro, Gazebo, generador de laberintos: cerrados (#2, #10, #11, #12). **#75 (controladores del brazo en Gazebo) abierto** |
| FW (6) | Avanzado, desordenado | FW-06 (PID por rueda) con PR #84 mergeado pero issue #78 abierto. FW-02 a FW-05 resueltos en la práctica por el firmware de banco, con el issue abierto |
| CAM (5) | Casi hecho | CAM-01 a CAM-03 cerrados. CAM-04 (calibración) cubierto por el PR #88. CAM-05 es un plan B opcional |
| PER (7) | Avanzado | Detector de flechas/cubos/canastos, validador. Faltan: dataset real (PER-06), verificación de agarre (PER-05), YOLO (PER-07, opcional) |
| NAV (7) | Avanzado, con otra arquitectura | Navegación por celdas con Trémaux (no el simulador abstracto de grilla del roadmap original) |
| MAN (5) | Parcial | Cinemática y servos en rama; banco de servos con PR #85 mergeado. Brazo real sin levantar |
| SIM (6) | Parcial | Mundos y sensores sí. **SIM-01 (emulador de firmware) no existe**: es la pieza que hace falta para que el simulador se comporte como el robot |
| VIS (3) / MIS (5) | Parcial | Hay una misión de 3 cubos simulada en `caid_navigation` (`mision_sim.launch.py`); no están los servidores reales de agarre |
| REAL (8) | Parcial | Banco de motores, odometría por ruedas, calibración parcial en el piso (3–4/11). Sin alimentación medida (REAL-03) ni calibración completa (REAL-04) |
| CIE (6) | No iniciado | Memoria, presentación, inspección y ensayo general sin hacer |

---

## 5. Lo medido en el robot real (lo que sirve de dato)

| Dato | Valor | Fuente |
|---|---|---|
| Flancos de encoder por vuelta | 40 (verificado con video) | `caracterizacion-motores.md` |
| Diámetro efectivo de rueda | 64,5 mm (geométrico 67) | calibración del 3/11 (D) |
| Trocha efectiva (giro en el lugar) | ≈ 291 mm ±7 % (6 de 10 giros de 360° válidos; los otros 4 salieron muy cortos) | D |
| Repetibilidad de 1 m | ±1,3 cm | D |
| Desvío lateral | ~5 cm por metro, sin signo fijo | D |
| Traseras vs delanteras | Las traseras giran al 39–45 % de las delanteras (otro tipo de motor) | `caracterizacion-motores.md` |
| Velocidad mínima que sigue el PI | 25 flancos/s ≈ 0,13 m/s; por debajo de 12,5 flancos/s, la orden se trata como 0 | firmware |
| Velocidad máxima útil por rueda | 75 flancos/s ≈ 0,395 m/s | firmware |
| Sonar lateral | Perdió el 32 % de las lecturas a 5,5 cm; el frontal no vio una tela en diagonal | D |
| Huecos de Wi-Fi | 200–400 ms | D |
| Cámara | 640×480; ~12 detecciones/s, ~44 ms de retraso; fx ≈ 947 px, cx ≈ 293, k1 ≈ 0,092 | PR #88 y `calibracion_esp32cam.yaml` |

---

## 6. Diferencias entre la simulación y el robot (las que importan)

Esta tabla es la base de la guía de calibración. Cada fila es una cosa que hoy **no coincide**.

| Aspecto | Simulación (`dev`) | Robot real | Consecuencia |
|---|---|---|---|
| Control de ruedas | `DiffDrive` ideal: mismas velocidades izq./der., aceleración ±1 m/s² | PI por rueda, piso de 0,13 m/s, zona muerta, traseras al 45 % | La sim sigue la orden mejor de lo que lo hace el robot |
| Separación efectiva | 0,204 m (calibrada con la física de Gazebo) | 0,291 m (medida con encoders) | Con el mismo `/cmd_vel`, los encoders de la sim y los del robot cuentan distinto |
| Odometría | `/sim/odom` casi perfecta | Por ticks, con cuantización de 5,3 mm y deriva | La navegación se prueba con datos demasiado buenos |
| Sonares | Láser de 5 rayos, ruido gaussiano σ = 5 mm, 20 Hz, sin pérdidas | Cono de ~15°, 32 % de pérdidas a distancia corta, frontal cada 120 ms y laterales cada 240 ms | La sim nunca "pierde" una pared |
| Cámara | 640×480, FOV 60°, 15 Hz, sin distorsión | fx ≈ 947 px → **FOV horizontal ≈ 37°**, distorsión, retraso ~44 ms | Lo que ve la visión en sim y en real es muy distinto |
| Red | Instantánea | Huecos de 200–400 ms, watchdog de 1 s | La sim no prueba los cortes |
| Mundo | Celda 0,32 m en el generador, pero `maze_2` usa 0,45 m (provisorio) | Pasillo de 320 mm, bloques de 170 mm de alto | Verificar qué mundo se usa |
| Masa e inercia | `m_base` = 1,2 kg, ruedas de 0,04 kg (provisorias) | Sin pesar | El patinaje depende de la masa |
| Brazo | En la posición original | Se desplaza hacia el frente | URDF desactualizado |

---

## 6.1 Qué podría estar mal o falta verificar (marcado como inferencia)
- **I:** los 291 mm de trocha efectiva salen de giros de 360°; el laberinto gira 90°, donde el patinaje del arranque pesa más. Hay que medir giros de 90° (el registrador de calibración los cubre).
- **I:** si el PR #87 sigue abierto aunque sus commits ya entraron con el #88, conviene cerrarlo para no confundir el tablero.
- **I:** `README.md` y `docs/README.md` no listan `candidatos/`, `tools/ruedas/` ni los paquetes nuevos.

---

## 7. Qué falta hacer, por prioridad

**A. Antes de seguir desarrollando (desbloquea todo lo demás)**
1. Decidir qué rama corrió en la competencia y cómo le fue; congelarla en `candidatos/` (issue de INF).
2. Integrar a `dev` el código del robot real (`laberinto-cuenta-giros`): un issue y una rama de integración, resolviendo los dos conflictos conocidos.
3. Cerrar o actualizar los issues que ya están hechos (FW-06, FW-02 a FW-05, CAM-04, #87).

**B. Calibración sim ↔ real (ver la guía)**
4. Medir en el piso: avance, giro, respuesta de velocidad, sonares, cámara, retrasos (REAL-04).
5. Escribir el emulador de firmware (SIM-01) alimentado por un archivo de calibración único.
6. Validar con un informe automático sim-vs-real.

**C. Robot**
7. Potencia: probar el XL4015 con el pack bajo, medir corrientes (REAL-03), armar el cableado nuevo y después la PCB.
8. Compilar y probar el port a ESP32-S3.
9. Actualizar el URDF cuando el brazo cambie de lugar.

**D. Navegación (informe aparte: [`navegacion-capas.md`](../pruebas/navegacion-capas.md))**
10. Capas 1 a 3 y calibración de cámara; se prueban primero en simulación.

**E. Cierre del proyecto**
11. CI mínimo (INF-04), memoria técnica (CIE-01), ADR de navegación por celdas, kit de inspección.

---

## 8. Riesgos

| Riesgo | Por qué importa | Mitigación |
|---|---|---|
| Código real en ramas sin integrar | Se pierde el conocimiento de lo que anda | Integrar y congelar en `candidatos/` |
| Sin CI | Cualquier PR puede romper algo sin aviso | INF-04 |
| Sim muy distinta del robot | Se afinan parámetros de navegación contra una física que no existe | Guía de calibración |
| Potencia sin medir | El brazo no levanta; riesgo de reinicios | REAL-03 y rediseño |
| Tablero desactualizado | El equipo no sabe qué falta | Revisar issues con este informe |
