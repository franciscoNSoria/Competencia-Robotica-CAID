# Torneo: laberinto con flechas verdes hasta el cubo verde

El robot real explora el laberinto con los sonares, sigue las flechas verdes con la cámara, frena frente al cubo verde, espera
que lo agarres con los sliders (tecla `y`) y vuelve solo a la partida. **El agarre es a mano por ahora** (prueba previa al
agarre autónomo).

| Archivo | Qué es |
|---|---|
| `torneo.py` | El programa que corre en la PC (ROS 2 + teclado). Habla directo con la ESP32 |
| `torneo_logica.py` | La navegación, sin ROS (la usan `torneo.py` y el simulador) |
| `simular_torneo.py` | Simulador 2D con el modelo del firmware y del robot de `tools/autonomo/simular.py` |

## Por qué es distinto del explorador anterior

El firmware de ruedas no sostiene una rueda por debajo de **25 flancos/s (~0,13 m/s)** y redondea a **0** lo que se le pida por
debajo de la mitad (`VEL_MIN_FLANCOS` y `ms_a_flancos` en `firmware/banco/robot_4ruedas/main.c`). El explorador anterior pedía
velocidades y giros chicos para "afinar": una rueda se apagaba, la otra seguía a 0,13 y el robot pivoteaba contra la pared (lo
que viste: "tantea, frena casi en el lugar y apunta a la pared"). Esto hace lo mismo que `tools/autonomo` (que anda):

- Avanza siempre a 0,20 m/s; para corregir fuerte sube hasta 0,27 m/s, así la rueda de adentro nunca baja del mínimo.
- El rumbo y la posición en el pasillo salen de los **dos sonares laterales** (filtro de Kalman), no del rumbo de la odometría.
- La pared de enfrente la frena **la guardia de la ESP32** (`d_stop` 7,5 cm = centro de la celda), sin el retardo del Wi-Fi
  (~0,4 s ida y vuelta = 8 cm a 0,20 m/s). Cerca del cubo baja a 5 cm (el mínimo que acepta el firmware).
- Los giros los hace la ESP32 (orden 7, con encoders) y después **verifica con los sonares** si quedó corto (4 de 10 giros
  medidos el 3/11 patinaron) y corrige. Las medias vueltas son dos giros de 90° verificados.
- Si va a girar con el robot corrido más de 2,5 cm del centro (girar en una celda de 40 cm deja 3,9 cm de margen), retrocede
  25 cm y vuelve a entrar centrándose.
- Cruza los pasillos sin frenar en cada celda (cada arranque desde quieto tuerce el robot) y frena solo donde decide o gira.

## Cómo correrlo

**No hace falta reflashear**: usa el firmware fusionado que ya tenés. **No corras `prueba_sin_camara.launch.py` ni
`mision_real.launch.py` al mismo tiempo**: su `calibracion_robot` manda otra guardia (20 cm) cada segundo y se pisan. Tampoco
`robot_autonomo.py`, `explorar_sin_camara.py` ni el panel de ruedas (todos mandan `/cmd_vel`).

Robot en el **centro de la celda de entrada, derecho, mirando hacia adentro del laberinto**. Cuatro terminales:

**1. Agente micro-ROS**

```bash
docker run -it --rm --net=host docker.io/microros/micro-ros-agent:jazzy udp4 --port 8888
```

**2. Cámara y visión** (cambiá `IP_DE_LA_CAMARA` por la IP de la ESP32-CAM en la red del robot):

```bash
source /opt/ros/jazzy/setup.bash
source ~/caid/Competencia-Robotica-CAID/ros2_ws/install/setup.bash
ros2 run caid_vision camara_node --ros-args -p fuente:=http://IP_DE_LA_CAMARA:81/stream &
ros2 run caid_vision vision_node
```

Para ver qué detecta (flechas y cubos marcados en la imagen): `ros2 run rqt_image_view rqt_image_view /vision/debug`.

**3. El robot**

```bash
source /opt/ros/jazzy/setup.bash
source ~/caid/Competencia-Robotica-CAID/ros2_ws/install/setup.bash
cd ~/caid/Competencia-Robotica-CAID
python3 tools/torneo/torneo.py
```

Esperá a que la línea de estado muestre `guardia 1` y apretá **g**. **ESPACIO** lo frena en cualquier momento (con **g**
vuelve a arrancar desde donde está). Sin cámara: `python3 tools/torneo/torneo.py --sin-camara`.

**4. El brazo** (cuando el robot frene frente al cubo):

```bash
source /opt/ros/jazzy/setup.bash
source ~/caid/Competencia-Robotica-CAID/ros2_ws/install/setup.bash
cd ~/caid/Competencia-Robotica-CAID
python3 tools/brazo/brazo_sliders.py
```

Agarrás el cubo y en la terminal 3 apretás **y**: retrocede, da media vuelta y vuelve a la partida.

## Qué mirar si algo sale mal

- Cada corrida queda en `~/caid/config/torneo_AAAAMMDD_HHMMSS.csv` (estado, celda, órdenes, sonares, guardia, posición lateral
  y rumbo estimados a 20 Hz). Mandalo junto con lo que imprimió la terminal 3.
- La terminal 3 imprime cada decisión (`celda (2, 0) rumbo +x: frente pared izq pared der paso`) y cada flecha que reconoce.
- Si no reconoce las flechas: mirar `/vision/debug`. Los colores (`RANGOS` en `caid_vision/detector.py`) se ajustan con la luz
  real. Si las ve pero las ubica mal, probar `--hfov` con el campo de visión real de la cámara (por defecto 60°).
- Parámetros: `--v` (velocidad), `--parada-cubo` (distancia al cubo), `--color`. El resto está en `Parametros` de
  `torneo_logica.py`, con el porqué de cada valor.

## Simulador

```bash
python3 tools/torneo/simular_torneo.py --laberinto casa --semillas 20
python3 tools/torneo/simular_torneo.py --laberinto casa --semillas 1 --detalle
```

Modela lo medido el 3/11 (ver `tools/autonomo/simular.py`): mínimo y zona muerta de las ruedas, la guardia real (el mismo C),
arranque tardío de 0,25-0,9 s, giros con ±7 % y 1 de cada 10 patinando, sonar con haz y ecos perdidos, Wi-Fi con demoras y
cortes. La cámara es un oráculo (prueba la conducción, no la visión). Resultado del 5/11 con 20 semillas:

| Laberinto | Llega al cubo | Vuelve a la partida | Corridas con algún choque |
|---|---|---|---|
| `casa` (pasillo con curvas, como el de tu foto) | 18 / 20 | 14 / 20 | 5 / 20 |
| `torneo` (cruces y flechas, más largo) | 9 / 20 | 9 / 20 | 8 / 20 |
| `ramas` (muchas ramas para explorar) | 0 / 20 | 0 / 20 | 14 / 20 |

El riesgo crece con la cantidad de giros (~2-3 % por giro en el simulador). La causa es geométrica: el robot (27,8 × 16,3 cm)
girando en el lugar barre un radio de 16,1 cm, y en una celda de 40 cm quedan 3,9 cm de margen.

## Pendiente

- Probarlo en el piso y comparar con el simulador (sobre todo cuánto tuerce el arranque y cuánto patinan los giros reales).
- Si el arranque tuerce mucho: en el firmware, las delanteras arrancan con el empujón más flojo (`nivel_kick_ini = 0`, 0,70 de
  duty) y tardan ~0,8 s (M); las traseras usan el nivel 2. Igualarlas emparejaría el arranque de los dos lados (hay que
  reflashear y probarlo con cuidado).
- La cámara como sensor de distancia a la pared (pedido): hace falta ver imágenes reales del laberinto (color de paredes y piso)
  para hacerlo bien. Hoy la cámara se usa para flechas y cubo; las paredes, con sonar y guardia.
- Agarre autónomo y entrega en el canasto.
