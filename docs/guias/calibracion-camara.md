# Calibración de la cámara (ESP32-CAM): colores y lente

Son dos calibraciones distintas:

| | Colores (`calibrar_colores`) | Lente (`calibrar_lente`) |
|---|---|---|
| Qué ajusta | Qué tono / saturación / brillo es rojo, verde o azul | fx, fy, cx, cy y distorsión |
| Afecta | **Si** se detectan cubos, flechas y canastos | **Dónde** calcula el robot que están |
| Cuándo repetirla | Cada vez que cambia la luz (en la pista, antes de competir) | Una vez por cámara (si no se cambia la lente ni la resolución) |
| Archivo | `caid_vision/config/colores.yaml` | `caid_vision/config/lente.yaml` |

Las dos herramientas guardan el YAML en `install/` (lo usan los nodos al instante) y en `src/` (para subirlo a git).

## Qué tener a mano

- La ESP32-CAM encendida en el robot, en su lugar y con su ángulo de siempre.
- **La luz de la pista** (o lo más parecido): prendida, fuerte y pareja. Con poca luz el sensor mete ruido verde y azul y ninguna calibración sirve.
- **Todos los objetos reales**: cubos rojo, verde y azul, flechas de los tres colores (si hay de distinto largo, todas) y los canastos.
- Para la lente: un **tablero de ajedrez impreso** de 10 x 7 cuadros (9 x 6 esquinas internas), pegado sobre algo rígido y plano, y una regla para medir un cuadro.

## 0. Levantar la cámara

Terminal 1 (sin ESP-IDF cargado):

```bash
cd ~/Competencia-Robotica-CAID/ros2_ws
source install/setup.bash
ros2 launch caid_bringup robot.launch.xml fuente:=http://192.168.18.66:81/stream
```

## 1. Colores

Terminal 2:

```bash
cd ~/Competencia-Robotica-CAID/ros2_ws && source install/setup.bash
ros2 run caid_vision calibrar_colores
```

1. **Fijar la cámara (tecla `f`)** con la escena ya armada y la luz definitiva. Congela la exposición y la ganancia en los valores de ese momento: si no, la cámara las cambia sola según lo que tenga adelante y el mismo cubo cambia de color. Se guarda en `colores.yaml` y `camara_node` se lo vuelve a mandar a la cámara cada vez que se conecta (aunque se reinicie). `u` las vuelve a automático.
2. **Tomar muestras de cada color.** Con el mouse, arrastrá un recuadro **adentro** del objeto (sin fondo ni borde) y apretá `r` (rojo), `v` (verde) o `a` (azul). Sumá varias por color:
   - el cubo y la flecha de ese color (y el canasto, si es del mismo color);
   - de frente y de costado (las caras del cubo con sombra son más oscuras);
   - cerca (~20 cm) y lejos (lo más lejos que tenga que detectar).
   Cada muestra recalcula el rango y muestra en la barra de abajo `H` (tono), `S>=` y `V>=`.
   - **con el robot andando**: mové el robot (o el objeto) a la velocidad de la misión, apretá `p` para congelar un cuadro movido, marcá el objeto y `r`/`v`/`a`; `p` otra vez para volver al vivo. El borde movido mezcla el color con el fondo y queda más apagado: si no hay muestras así, el rango queda justo y el objeto se pierde al moverse.
3. **Revisar las máscaras** con `1`, `2`, `3`: se ve en color solo lo que el detector toma como rojo, verde o azul. Tiene que aparecer el objeto entero y nada del fondo.
   - Objeto con agujeros o cortado → falta muestra de esa parte (sombra, reflejo): tomá otra ahí.
   - Fondo pintado → el rango quedó muy ancho: `b` borra las muestras de ese color y volvé a tomarlas con recuadros más ajustados.
   - Si en la terminal sale *"El X% de las muestras de rojo cae en el rango de verde"*, esos dos colores se van a confundir: retomá las muestras del que esté mal.
4. **Revisar las detecciones** con `0`: cubos y canastos con recuadro, flechas con su dirección. Mové los objetos, giralos, acercalos y alejalos.
5. **Guardar con `g`.** `vision_node` lo recarga solo en un segundo, sin reiniciar nada. `q` para salir.

### Ver los objetos con el robot en movimiento

El problema al moverse es el **desenfoque**: mientras la cámara toma el cuadro, el robot avanza y el objeto sale corrido. Probado con imágenes sintéticas, el detector aguanta hasta ~17 px de corrimiento sin perder nada; con 30 px pierde la mitad de las flechas. Lo que lo achica:

1. **Más luz**: con poca luz la cámara alarga la exposición sola y todo sale movido.
2. **Exposición más corta** (teclas `[` y `]` en `calibrar_colores`): `[` la acorta un 20 % y deja la ganancia en automático para compensar el brillo. Bajala hasta que, con el robot andando, el cubo se vea nítido en la vista `0`, sin que la imagen se llene de ruido. Se guarda con `g` y `camara_node` la aplica sola.
3. **Muestras en movimiento** (`p`, paso 2 de arriba).

No se "recuerdan" detecciones entre cuadros: la navegación ubica canastos y flechas con la pose del robot en el momento de cada imagen y cuenta votos por imagen, así que repetir una detección vieja la anotaría en la celda equivocada. Para eso ya pide varias imágenes iguales (`confirmar_flecha`, `confirmar_canasto`).

## 2. Lente

Solo hace falta una vez por cámara. Terminal 2:

```bash
ros2 run caid_vision calibrar_lente --ros-args -p cuadro:=0.025
```

`cuadro` es el lado de un cuadro medido con regla, **en metros**. Si tu tablero no es de 9 x 6 esquinas internas, agregá `-p columnas:=N -p filas:=M`.

1. Mostrá el tablero entero a la cámara. Cuando lo encuentra, las esquinas se dibujan en color: **espacio** guarda la vista.
2. Juntá **15 a 25 vistas** variadas: cerca y lejos, en el centro y en las cuatro esquinas de la imagen (los recuadros amarillos muestran dónde ya hay), inclinado a los costados y arriba/abajo. **Quieto** en cada una.
3. `c` calcula. Mirá el **error**: menos de 0,5 px está bien; más de 1 px, sumá vistas o retomalas.
4. `d` muestra la imagen corregida: las líneas rectas (bordes de la pared, del tablero) tienen que verse rectas.
5. `g` guarda `lente.yaml`.

Para usarla:

```bash
ros2 launch caid_bringup robot.launch.xml fuente:=http://192.168.18.66:81/stream \
  calibracion:=$(ros2 pkg prefix caid_vision)/share/caid_vision/config/lente.yaml
```

y en el explorador `-p calibracion:=` con la misma ruta.

### Comprobar la lente con una medida

Con un rectángulo de color de ancho conocido (ej. 10 cm) de frente y a una distancia medida con cinta:

```bash
ros2 run caid_vision medir_fx --ros-args -p ancho:=0.10 -p distancia:=0.50
```

El `fx` que mide tiene que dar parecido al de `lente.yaml` (±5 %) a 2 o 3 distancias distintas.

## 3. Probar todo junto

```bash
ros2 launch caid_bringup robot.launch.xml fuente:=http://192.168.18.66:81/stream visor:=true
```

El visor muestra las detecciones, los fps y el retraso. Pasá cada objeto por delante a distintas distancias y ángulos. Para ver lo que el robot realmente usa (flechas confirmadas varios cuadros seguidos):

```bash
ros2 topic echo /vision/flecha
```

### Flechas en el visor

Con el visor abierto (`visor:=true`), acercá la cámara a cada flecha:

| Cómo se ve | Qué significa |
|---|---|
| Línea blanca fina, "flecha azul a 0,62 m" | La ve, pero está lejos: todavía no decide |
| Línea verde gruesa y cartel grande "IZQUIERDA P2" | Anotada (a `distancia_flecha` o menos, 50 cm por defecto). Abajo: distancia y cuántas de las últimas lecturas dijeron esa dirección y ese tamaño |
| Línea amarilla con el cartel | Anotada, pero ahora se ve de lejos: no vota |

Si la distancia que muestra no coincide con la real, calibrá la lente (sección 2) y lanzá con `calibracion:=...lente.yaml`. El tamaño (P1 / P2 / P3) no depende de eso.

## 4. Guardar en git

```bash
cd ~/Competencia-Robotica-CAID
git add ros2_ws/src/caid_vision/config/colores.yaml ros2_ws/src/caid_vision/config/lente.yaml
git commit -m "calib(vision): colores y lente con la luz de <lugar>"
```

## Si algo falla

| Síntoma | Qué hacer |
|---|---|
| Aparecen flechas o cubos donde no hay nada | Más luz. Retomar muestras con recuadros más ajustados. En vivo: `ros2 param set /vision_node area_min_flecha 600` |
| Un objeto no se detecta nunca | Mirar su máscara (`1`/`2`/`3`): si sale cortado, tomar más muestras de ese objeto |
| Los colores cambian solos al mover cosas | Fijar la cámara con `f` (paso 1.1) |
| Se detecta bien de cerca pero no de lejos | Tomar muestras con el objeto lejos (sale más oscuro y menos saturado) |
| `calibrar_lente` no encuentra el tablero | Más luz, tablero entero en la imagen, plano y sin reflejos; revisar `columnas` y `filas` |
| Cambió la luz (otro lugar, de día / de noche) | Repetir solo la parte de colores (paso 1) |
| El MDF de las paredes aparece como rojo | El rango rojo quedó ancho (tono hasta 20+ o saturación baja). `1`, `b`, y volver a tomar muestras chicas en el centro del cubo. El MDF real da tono 21-29 y saturación 36-65 |
| La imagen sale lavada (colores pálidos) con mucha luz | Bajar `brightness` a 0 en la sección `camara` de `colores.yaml` y recalibrar |
