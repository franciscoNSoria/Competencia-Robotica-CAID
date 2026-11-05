# Laberinto con flechas hasta el cubo (rama `integracion`)

Trémaux por celdas de `laberinto_ruedas.py` (commit 0354aef: callejón = dos giros de 90°) + la visión de `esp-vision-5`
(49d5d2f: `/vision/proximo_giro`). **Objetivo por ahora: seguir las flechas y frenar al llegar al cubo. Sin vuelta a la partida.**

## Qué hace

1. En cada celda mide con los sonares, decide, gira y avanza una celda (igual que antes).
2. **El próximo giro lo decide la flecha.** `vision_node` publica `/vision/proximo_giro` (`{"giro": "derecha"|"izquierda", "color", "tamano"}`)
   cuando una flecha queda **FIJA** (a 30 cm). `Recorrido.recibir_flecha` la guarda y en la próxima decisión la pasa como
   `preferida` a `mapa.decidir(..., insistir=True)`.
   - Una flecha manda **un solo giro**. Después del giro se ignora la visión `SORDO_TRAS_GIRO_S` (2,5 s), porque sigue mandando la flecha vieja hasta 2 s.
   - Si el lado que pide tiene pared y el frente está libre, la guarda para el cruce siguiente (la flecha está en una pared más adelante).
     Si el frente también está cerrado, la ignora y sigue Trémaux.
   - Sin flecha vigente (`VIGENCIA_FLECHA_S` = 3 s): Trémaux, solo para moverse.
3. **Color.** Sigue siempre el **primer color** que llega y se lo manda a la visión por `/nav/estado` (`{"busca": color}`, latched).
   Así, con las tres flechas a la vista, la visión solo manda la de ese color. Al arrancar (sin color), `vision_node.elegir_proximo`
   prefiere la **P3** (la más chica = camino más corto), después P2 y P1, y a igual tamaño la más cercana. `--color` lo fija desde el inicio.
4. **Cubo.** `/vision/cubo/<color>` (Point, `z` = área en px²). Distancia ≈ `fx · 0,02 / √área`. Si es del color seguido (o todavía
   no hay color) y está a `--dist-cubo` (0,20 m) o menos: `parar()` en el acto, estado `fin`, imprime `== LLEGUE AL CUBO ... ==`.
   Un cubo de otro color no lo frena. Si recorre todo sin encontrarlo, se detiene y lo dice.

## Cómo se corre (robot real)

Hacen falta 3 terminales. En ninguna puede estar cargado ESP-IDF, y hay que cerrar el panel de ruedas y cualquier otro programa que mande `/cmd_vel`.

```bash
# 1) Agente micro-ROS (antes de encender el robot)
docker run -it --rm --net=host docker.io/microros/micro-ros-agent:jazzy udp4 --port 8888

# 2) Cámara + visión. OJO: esp32_falsa:=false, si no levanta la ESP32 falsa y se pisa con la real
cd ~/Competencia-Robotica-CAID/ros2_ws
colcon build --packages-select caid_vision caid_bringup caid_navigation
source install/setup.bash
ros2 launch caid_bringup robot.launch.xml fuente:=http://<IP-ESP32-CAM>:81/stream esp32_falsa:=false visor:=true

# 3) El recorrido (desde la raíz del repo)
source /opt/ros/jazzy/setup.bash
python3 tools/ruedas/laberinto_real.py            # g arranca, ESPACIO o p frena, q sale
```

Poné el robot en el **centro** de la celda de partida mirando hacia adentro del laberinto.
El script lee los sumadores del giro de 90° y de 180° de `~/caid/config/calibracion.json`, que son los que quedaron en el panel de ruedas.

| Opción | Por defecto | Qué hace |
|---|---|---|
| `--color verde` | (vacío) | sigue ese color desde el principio, en vez de adoptar el primero |
| `--dist-cubo 0.25` | 0.20 | distancia (m) a la que frena al ver el cubo |
| `--fx 950` | 950 | foco de la cámara en px para medir el cubo (`config/calibracion_esp32cam.yaml`, 640x480) |
| `--sin-flechas` | | Trémaux puro, sin escuchar la visión (como en 0354aef) |
| `--v`, `--vel-giro`, `--frenada` | 0.16, 153, 0.03 | como antes |

## Qué mirar en el log

- `flecha: sigo el color X (P3)`: adoptó el color.
- `flecha X P3: el proximo giro es a la IZQUIERDA`: la visión fijó la flecha. **Tiene que salir antes** de la línea `celda ... mirando ...`
  del cruce. Si no sale, la flecha no se fijó a tiempo (luz, `distancia_fija`, colores) y en ese cruce va por Trémaux.
- `sigo la flecha X: giro a la ...`: obedeció.
- `flecha X a la ... ignorada: ese lado tiene pared`: la flecha pedía un lado cerrado.
- `== LLEGUE AL CUBO X ==`: terminó.

Para probar la visión sola: `ros2 topic echo /vision/proximo_giro` y `ros2 topic echo /vision/cubo/verde` (el área está en `z`).

## Sin robot (simulador)

```bash
python3 -m pytest -q tools/ruedas/test_flechas.py tools/ruedas/test_ruedas_logica.py tools/ruedas/test_guiado.py
```

`test_flechas.py` usa `sim_laberinto.correr(..., cada_tick=...)` para inyectar flechas y cubos en una T. Sin flecha, Trémaux va a la
derecha. Con la flecha "izquierda" va a la izquierda y frena en el cubo. Además prueba que ignora los otros colores, que un cubo
de otro color no lo frena y que la flecha de más adelante no se gasta. `test_ruedas_panel.py` necesita pantalla (Tk): con `pytest tools/ruedas`
entero se cuelga sin display; correlo con `xvfb-run`.

## Dónde está el código

| Archivo | Qué |
|---|---|
| `tools/ruedas/laberinto_ruedas.py` | `Recorrido.recibir_flecha`, `recibir_cubo`, `_preferida_flecha`, y la decisión en `_medir` |
| `tools/ruedas/laberinto_real.py` | `conectar_flechas`: suscripciones a `/vision/proximo_giro` y `/vision/cubo/*`, y publica `/nav/estado` |
| `ros2_ws/src/caid_vision/caid_vision/vision_node.py` | `elegir_proximo`: el color buscado, o la P3 al arrancar |

## Pendiente / no verificado en el robot

- Los **20 cm** del cubo salen de la cuenta (fx ≈ 950 y cubo de 2 cm), no se midieron. Comparar con `ros2 topic echo /vision/cubo/<color>` a distancia conocida y ajustar `--dist-cubo` o `--fx`.
- No se probó en el piso que la flecha quede FIJA antes de que el robot decida en el cruce.
- El tamaño P2/P1 (cuántos tramos rectos faltan) **no se usa** todavía: solo la dirección del próximo giro.
- Merge: `detector.py` quedó como en la rama de visión. `explorador.py`, `camara.py` y `gen_maze` quedaron como en 0354aef.
  El navegador ROS (`caid_navigation`) no se tocó y **no** usa esto.
