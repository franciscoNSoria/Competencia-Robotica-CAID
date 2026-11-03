# Brazo con sliders desde la PC (v3)

Estados: **M** medido, **C** calculado, **P** propuesta o dato sin comprobar, **S** simulado en la PC.

**Estado de esta versión:** probada en la PC con el firmware real y servos simulados (**S**). **Falta probarla en el robot.**

## Qué es

Cuatro servos (base, hombro, codo, pinza) que se mueven desde una ventana con sliders en la PC, por Wi-Fi. La ESP32 corre micro-ROS,
se conecta al agente micro-ROS de la PC y la ventana es un nodo de ROS 2. Es la forma en que se va a comandar el brazo el día de la competencia.

## Archivos

| Archivo | Dónde va | Qué es |
|---|---|---|
| `servos_brazo.h` | `main/` del ejemplo `int32_publisher` de micro-ROS | Grados a pulso, rampa de velocidad, límites y tope de cada servo |
| `enlace_uros.h` | `main/` del mismo ejemplo | Conexión con el agente (reconecta sola) y los tópicos `/brazo/cmd` y `/brazo/estado` |
| `main.c` | `main/` del mismo ejemplo | Pines, límites de arranque, reposo seguro y la tarea de control |
| `../../tools/brazo/brazo_sliders.py` | la PC | La ventana de sliders |

Los tres archivos de C van **juntos** en `main/` (reemplazan el `main.c` del ejemplo).

## Qué hay de nuevo en la v3

- **Límites y reposo de cada servo editables desde la ventana**, sin regrabar la ESP32. Se mandan dentro de cada orden y se guardan con
  GUARDAR LIMITES en `~/caid/config/brazo_calibracion.json` (fuera del repo, es de cada PC).
- **Tope propio de cada servo en la ESP32** (la PC no puede pasarlo): base, hombro y pinza 180° (2500 µs); codo 200° (2722 µs).
- **Reposo seguro obligatorio antes de girar la base**, impuesto por la ESP32 (ver abajo).
- Botón **REPOSO** (antes CENTRAR): lleva los 4 sliders a su reposo. Valor en µs junto a cada slider.
- La ESP32 no confía en la PC: recorta límites al tope de cada servo, descarta rangos de menos de 1° y cualquier orden con NaN.

## Cómo se compila, graba y corre

Requisitos: ESP-IDF 5.5 y el componente micro-ROS (rama `jazzy`) ya configurados (`docs/guias/micro-ros.md`: red Wi-Fi de **2,4 GHz**
e IP de la PC como IP del agente, puerto 8888, se cargan con `idf.py menuconfig`), ROS 2 Jazzy, Docker y `python3-tk`.
**La red, la clave y la IP quedan en el `sdkconfig` del ejemplo, fuera de este repo: no se suben nunca.**

**1. Agente** (terminal normal, antes de encender el robot):
```
docker run -it --rm --net=host docker.io/microros/micro-ros-agent:jazzy udp4 --port 8888
```
`--net=host` hace que el contenedor use la red de la PC: así llegan los paquetes UDP de la ESP32 y los nodos de ROS 2 de la PC ven los tópicos.

**2. Compilar y grabar** (terminal limpia de ROS: el componente de micro-ROS lo exige). Dos pasos separados:
```
env -i HOME="$HOME" USER="$USER" TERM="$TERM" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin bash --noprofile --norc
```
y, ya dentro de esa terminal:
```
. ~/esp/esp-idf-v5.5/export.sh
echo "$AMENT_PREFIX_PATH"
```
El `echo` tiene que salir vacío. Después, poniendo en `REPO` la carpeta de tu repo y en `EJEMPLO` la del ejemplo `int32_publisher`:
```
REPO=/ruta/a/tu/repo
EJEMPLO=/ruta/al/ejemplo/int32_publisher
cp "$EJEMPLO"/main/main.c "$EJEMPLO"/main/main.c.respaldo
cp "$REPO"/firmware/brazo_sliders/main.c "$REPO"/firmware/brazo_sliders/servos_brazo.h "$REPO"/firmware/brazo_sliders/enlace_uros.h "$EJEMPLO"/main/
cd "$EJEMPLO"
idf.py build
```
El `build` tiene que mostrar líneas `Building C object` y un enlazado. Para grabar, con la ESP32 **fuera del shield** (nunca el USB y el riel de 5 V a la vez):
```
idf.py -p /dev/ttyUSB0 -b 115200 flash
```
Salió bien si aparece `Hash of data verified.` tres veces y `Hard resetting via RTS pin...`.
**No uses `idf.py monitor` ni pegues logs del monitor:** el componente imprime la clave del Wi-Fi. No repitas `set-target`: borra la configuración.

**3. Ventana** (terminal con ROS):
```
source /opt/ros/jazzy/setup.bash
python3 /ruta/a/tu/repo/tools/brazo/brazo_sliders.py
```
El LED azul parpadea mientras la ESP32 busca al agente y se apaga al conectar. Para confirmar que tiene la v3:
```
ros2 topic echo /brazo/estado --once --qos-reliability best_effort
```
(`--qos-reliability best_effort` es necesario: la ESP32 publica así y el `echo` por defecto no recibiría nada). Tienen que salir **23 números**:
ángulos iniciales `95, 60, 180, 150` y topes `180, 180, 200, 180` en las posiciones 16 a 19.

## La ventana

- Cada servo arranca **suelto** (sin pulso, sin fuerza). Con *activo* tildado el servo sigue a su slider. **Al activarlo salta a la posición del slider.**
- *Velocidad* (10 a 360 °/s) limita qué tan rápido se acerca al objetivo. *Salto directo* manda el objetivo de golpe: más fuerza, sin rampa.
- *ACTIVAR TODOS* / *SOLTAR TODOS* / *REPOSO*.
- Sección **LIMITES Y REPOSO**: casillas *min*, *max* y *reposo* por servo y el rango de pulsos en µs. El máximo no pasa del tope del servo.
- **Antes de cerrar la ventana, apretá SOLTAR TODOS.** Cerrarla no afloja los servos: la ESP32 mantiene la última posición y un servo
  que empuja contra su tope se calienta.
- La ventana arranca tomando lo que informa la ESP32 (no suelta ni mueve nada al abrir). Si la ESP32 se reinicia, conserva los límites de la ventana y se los vuelve a mandar.

## Reposo seguro antes de girar la base

La base choca con la cámara si el brazo no está alto. Por eso la ESP32 no deja girar la base hasta que el brazo está en el **reposo seguro**
(el *reposo* del hombro y del codo en la ventana; de fábrica hombro 60°, codo 180°):

1. Se pide mover la base (cualquier cambio de más de 0,5°).
2. El hombro y el codo van al reposo seguro, a la velocidad configurada. La base espera quieta.
3. Cuando los dos llegaron, se esperan además `ESPERA_REPOSO_MS` (1000 ms) y la base gira. El brazo se queda en el reposo.
4. Al llegar la base, el brazo vuelve a lo que piden los sliders.

- Si el hombro o el codo están **sueltos**, la base no gira (la ventana avisa en ámbar). Reactivar la base con otro objetivo también cuenta como girar.
- Si la PC se arrepiente a mitad de camino (el objetivo vuelve a donde está la base), el brazo vuelve a seguir los sliders.
- Los servos no informan su posición: "llegó" significa "el pulso llegó". El segundo de espera es una estimación (**P**): hay que subirlo si el hombro tarda en levantar el brazo.
- Parámetros en `main.c`: `BLOQUEO_BASE_ACTIVO` (0 = sin bloqueo, solo para probar la base sin brazo), `POSE_SEGURA_*`, `TOL_REPOSO_DEG`, `ESPERA_REPOSO_MS`, `BASE_UMBRAL_DEG`.

## Valores de arranque del firmware (se pueden cambiar desde la ventana)

| Servo | Pin | Límites | Tope | Objetivo inicial | Reposo de la ventana |
|---|---|---|---|---|---|
| base | GPIO18 | 0–180 | 180 | 95 | 95 (alineada con el robot, **M** a ojo) |
| hombro | GPIO19 | 10–125 | 180 | 60 | 60 (reposo seguro) |
| codo | GPIO21 | 80–200 | 200 | 180 | 180 (reposo seguro) |
| pinza | GPIO22 | 100–180 | 180 | 150 | 114 (cierra apretada en 114°, **M** a ojo) |

Los grados son **nominales**: 500 µs = 0° y 2500 µs = 180° (11,1 µs por grado). No son los grados reales de la articulación
(`docs/guias/brazo-medidas.md`). PWM de 50 Hz con el periférico LEDC (16 bits).

## Protocolo

`/brazo/cmd` (PC → ESP32, `std_msgs/Float32MultiArray`, QoS best effort, 10 Hz, siempre completo):

| Índices | Contenido |
|---|---|
| 0–3 | Ángulo objetivo: base, hombro, codo, pinza |
| 4 | Máscara de servos activos (bit 0 base, 1 hombro, 2 codo, 3 pinza) |
| 5 | Velocidad máxima en °/s (0 = salto directo) |
| 6–13 | (opcional) mínimo y máximo de cada servo |
| 14–15 | (opcional) reposo seguro: hombro y codo |

Con 6 o 14 números la orden sigue valiendo (los límites o el reposo no cambian). Con más de 16 el mensaje se descarta (el buzón es de 16).

`/brazo/estado` (ESP32 → PC, 10 Hz):

| Índices | Contenido |
|---|---|
| 0–3 | Ángulo actual de cada servo (el de la rampa, no el medido) |
| 4, 5 | Máscara de activos, velocidad en uso |
| 6 | Contador de mensajes (si baja, la ESP32 se reinició) |
| 7 | ms desde la última orden (antes de la primera orden vale el tiempo desde el arranque) |
| 8–15 | Mínimo y máximo de cada servo |
| 16–19 | Tope absoluto de cada servo |
| 20 | Fase de la base: 0 libre, 1 esperando el reposo, 2 girando |
| 21–22 | Reposo seguro en uso (hombro, codo), acotado a sus límites |

La ventana v3 con un firmware viejo funciona sin fase ni topes (usa 180 para todos). Un firmware v3 con la ventana vieja también funciona.
Una ventana v2 o v3 con un firmware **v1** no: el buzón de 8 números descarta las órdenes más largas.

## Plan de prueba (para quien la prueba)

Antes: robot con espacio libre, brazo sin nada que pueda chocar, batería cargada, una mano cerca del interruptor.

1. Grabar el firmware, abrir el agente y la ventana. **Esperado:** "ESP32 conectada | ultima orden recibida hace X ms" con X chico, todos los servos sueltos.
2. `ros2 topic echo ... --once`: 23 números, topes `180, 180, 200, 180`.
3. Activar la **base** sola y moverla despacio. **Esperado:** se mueve sin avisos (el hombro y el codo todavía están sueltos: si se pide girar, aparece "BASE EN ESPERA: activa el hombro y el codo").
4. Activar el **hombro y el codo de a uno** (saltan a 60° y 180°: dejarles lugar). Verificar que quedan en el reposo.
5. Con los tres activos, llevar la base a otro ángulo. **Esperado:** "BASE EN ESPERA", el brazo queda en el reposo, la base empieza a girar poco después de que el brazo llegó ("BASE GIRANDO"), y al terminar el brazo vuelve a sus sliders.
6. Cancelar a mitad: pedir un giro y volver el slider de la base a su lugar. **Esperado:** la base no se mueve y el brazo vuelve a sus sliders.
7. Codo: subir el *max* de a 5° desde 180 hasta 200 mirando el brazo. Anotar a partir de qué valor deja de avanzar o zumba.
8. Velocidad 90 °/s contra 47 °/s: ¿se ve a escalones con la lenta? (**P**: zona muerta del servo de unos 5 µs).
9. Cambiar límites, GUARDAR LIMITES, cerrar (SOLTAR TODOS antes) y abrir: **esperado:** vuelven los límites guardados.
10. **Reportar:** qué pasó en cada paso, el valor donde el codo deja de avanzar, cuánto tarda el hombro en llegar al reposo y si 1 s de espera alcanza, y cualquier calentamiento de un servo.

## Problemas conocidos y pendiente

- Esta versión no se probó en el robot (**S** solamente).
- El tiempo de espera del reposo (1000 ms) y que 200° en el codo sea alcanzable son estimaciones (**P**): 2722 µs pasa de los 2500 µs que dan los vendedores del MG90S.
- En el banco, el hombro no respondía en el conector de GPIO19 con tres firmwares distintos, mientras que el mismo servo anda en el conector de GPIO22 (**M**, `brazo-medidas.md`). Se están revisando las soldaduras del shield.
- Sin realimentación de posición, el reposo seguro es por tiempo. Una opción futura es un sensor o la corriente del servo.
- Pendiente: pasar los límites y el reposo (grados nominales) a grados reales y actualizar el URDF y Gazebo (el brazo en modo `actuated` se desarma sin controladores).