# caid_vision: cámara, detección y calibración

Lee la ESP32-CAM (o una webcam), detecta **cubos, flechas, canastos y zonas rojas** con OpenCV clásico y publica lo que ve para la navegación.

## Cómo lanzar

Siempre en una terminal **sin ESP-IDF cargado**:

```bash
cd ~/Competencia-Robotica-CAID/ros2_ws
colcon build --packages-select caid_vision caid_bringup caid_navigation
source install/setup.bash
```

**Cámara + visión + visor** (la ventana con las detecciones, los fps y el retraso):

```bash
ros2 launch caid_bringup robot.launch.xml fuente:=http://192.168.18.66:81/stream visor:=true
```

| Argumento | Por defecto | Qué hace |
|---|---|---|
| `fuente` | `0` | `0` = webcam, o `http://IP:81/stream` = ESP32-CAM |
| `visor` | `false` | `true` abre la ventana del visor |
| `fps` | `15` | cuadros por segundo que publica `camara_node` |
| `colores` | `config/colores.yaml` | rangos de color calibrados y ajustes de la cámara |
| `calibracion` | vacío | YAML de la lente (`config/lente.yaml` de `calibrar_lente`): `camera_info` y distancias exactas |
| `distancia_flecha` | `0.5` | m: a esta distancia o menos se decide dirección y tamaño de una flecha |
| `ventana_flecha` | `10` | dirección y tamaño = mayoría de estas últimas lecturas cercanas |

Ejemplo: `ros2 launch caid_bringup robot.launch.xml fuente:=http://192.168.18.66:81/stream visor:=true distancia_flecha:=0.6`

**Herramientas** (con el launch corriendo, en otra terminal):

```bash
ros2 run caid_vision visor                    # reabrir el visor si se cerró
ros2 run caid_vision calibrar_colores         # calibrar rojo / verde / azul con los objetos reales
ros2 run caid_vision calibrar_lente --ros-args -p cuadro:=0.025   # lente, con tablero de ajedrez
ros2 run caid_vision medir_fx --ros-args -p ancho:=0.10 -p distancia:=0.50   # comprobar fx
```

La guía paso a paso de las calibraciones está en [docs/guias/calibracion-camara.md](../../../docs/guias/calibracion-camara.md).

## Nodos y tópicos

**`camara_node`**: un hilo lee el stream sin parar y se queda solo con el último cuadro (sin la cola de cuadros viejos de OpenCV). Al conectarse le manda a la ESP32-CAM la sección `camara` de `colores.yaml` (brillo, exposición...).

| Publica | Tipo | |
|---|---|---|
| `/camera/image_raw/compressed` | `CompressedImage` | JPEG (~40 KB): lo que lee `vision_node` |
| `/camera/image_raw` | `Image` | cruda (900 KB), solo si alguien la mira |
| `/camera/camera_info` | `CameraInfo` | K y D de la calibración de lente |

**`vision_node`**: corre el detector (`detector.py`) y valida las flechas entre cuadros (`validador.py`). Recarga `colores.yaml` solo cuando cambia.

| Publica | Tipo | |
|---|---|---|
| `/vision/detecciones` | `String` (JSON) | todo lo de cada cuadro, en píxeles: flechas (`punta`, `cuello`, `cola`, `hombros`, `angulo`), cubos, canastos, zonas. **Lo usa el explorador** |
| `/vision/flecha` | `String` | `color,direccion,angulo,tamaño` de cada flecha anotada (`derecha`/`izquierda`, `P1`/`P2`/`P3`) |
| `/vision/cubo/<color>` | `Point` | centro (x, y) y área del cubo |
| `/vision/debug/compressed` | `CompressedImage` | imagen con todo dibujado (lo muestra el visor) |

Parámetros en vivo (`ros2 param set /vision_node ...`): `v_min`, `v_min_azul`, `s_min` (-1 = lo de `colores.yaml`), `area_min_cubo`, `area_min_flecha`, `area_min_canasto`.

## Flechas (reglamento nov. 2026)

Cuerpo de **10 mm** de ancho, punta de **29 mm** de base y **10 mm** de alto. Largo total **30 / 60 / 95 mm** = **P3 / P2 / P1** (cuerpo de 20 / 50 / 85 mm).

- **De lejos** solo se ve que hay una flecha ("flecha azul a 0,62 m").
- **A `distancia_flecha` o menos** queda **anotada** y no se borra más. Su **dirección** (solo derecha o izquierda, aunque esté inclinada) y su **tamaño** se siguen reevaluando: vale la mayoría de las últimas `ventana_flecha` lecturas.
- La **distancia** sale del ancho de la base de la punta (29 mm) y depende de fx: sin calibrar la lente se supone FOV de 60°. El **tamaño** sale de comparar el cuerpo con esa base: es una proporción y **no depende de la calibración**.
- El **explorador** (`caid_navigation`) hace lo mismo por su cuenta con `/vision/detecciones`, ubicando cada flecha en su celda del mapa; publica qué busca, las flechas y la última decisión de giro en **`/nav/estado`**.

## Detector

- La punta se decide con las dos entradas de la flecha: cada una queda entre el hombro (pegado al cuello) y la esquina de la cola; la punta está del otro lado de la cola.
- Primero se prueba si es cubo (macizo y cuadrado, acepta girado 45°), después flecha (nunca maciza: solidez ≤ 0,88, dos hombros, borde liso).
- Un objeto que toca el borde izquierdo o derecho de la imagen se descarta (la caja no sirve para medir).
- Si el MDF, la piel o las sombras aparecen como color, el rango quedó ancho: recalibrar con recuadros chicos en el centro del objeto.
