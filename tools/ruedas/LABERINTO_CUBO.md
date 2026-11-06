# Laberinto con cubo: primer cubo y vuelta por giros

Todo corre desde la PC. **No hace falta reflashear la ESP32** (firmware `firmware/banco/robot_4ruedas/main.c`, dominio ROS 71).
Rama: `feature/laberinto-cuenta-giros`.

## Cargar todo (cada comando en su propia terminal)

Cada terminal empieza con el entorno (dominio 71, solo esta PC):

```bash
export CAID_DOMINIO=71; source ~/caid/env_ros.sh
```

1. **Agente micro-ROS** (antes mata el viejo: `pkill -f micro-ros-agent`):
   ```bash
   docker run -it --rm --net=host docker.io/microros/micro-ros-agent:jazzy udp4 --port 8888
   ```
2. **Cámara + visión** (la visión publica `/vision/cubo/<color>`):
   ```bash
   ros2 launch caid_bringup robot.launch.xml fuente:=http://172.16.0.57:81/stream esp32_falsa:=false visor:=true \
     calibracion:=$(ros2 pkg prefix caid_vision)/share/caid_vision/config/calibracion_esp32cam.yaml distancia_flecha:=0.8 distancia_fija:=0.60
   ```
3. **Laberinto** (CERRAR el panel de ruedas: no pueden estar los dos):
   ```bash
   cd ~/caid/Competencia-Robotica-CAID
   python3 tools/ruedas/laberinto_real.py                  # primer cubo, vuelta por el MAPA (Trémaux)
   python3 tools/ruedas/laberinto_real.py --cuenta-giros   # primer cubo, vuelta A CIEGAS repitiendo al reves los giros anotados
   ```
   Robot en el centro de la celda de partida, derecho, mirando hacia adentro. Teclas: **g** arranca, **Y** = OK (3 veces por ciclo), ESPACIO o p frena, q sale.

El **seguidor de flechas** es otro programa (`python3 tools/ruedas/seguidor_real.py`, opcional `--volver`); el laberinto no usa flechas.

## Primer cubo (los dos modos)

Explora celda por celda con los sonares (Trémaux). Cuando la cámara ve un cubo a ~50 cm o menos: deja de explorar, se acerca despacio y frena a **10 cm
o más** del cubo (el tope de 15 cm del frontal es solo para paredes). Espera **Y**; con la Y gira 180°, imprime el **mapa del camino** (celdas, giros y
dibujo de paredes) y vuelve a la partida.

## Vuelta (la parte que cambia)

| Modo | Cómo vuelve |
|---|---|
| sin flag | Camino más corto que conoce en el mapa (`mapa.decidir(destino=inicio)`). Si no conoce camino: **pausa** y espera OK en vez de detenerse. |
| `--cuenta-giros` | A ciegas. Durante la ida anota los rectos (odometría) y cada giro. Tras el 180° repite todo **al revés**: los giros se invierten (último giro der → primero izq) y cada giro se hace en el **primer espacio libre del lado** del giro invertido (lateral abierto 0,2 s, sigue 12 cm y dobla) o en la pared de frente. No mira mapa, cámara ni bifurcaciones. Riesgo: una salida del mismo lado que no tomó a la ida la toma a la vuelta. |

Al llegar a la partida frena y espera **Y** (cubo entregado). Después **PAUSA**: con otra **Y** gira 180° y vuelve a entrar a buscar el resto, **ignorando
el color ya entregado** (se avisa a la visión por `/nav/estado`). Si el camino se perdió, la pausa no da el cubo por entregado y no ignora su color.

## Ajustes que ya están aplicados

- **Giros** (panel de ruedas, fila "VELOCIDAD DE GIRO por lado", guardada en `~/caid/config/calibracion.json`): izquierda/derecha por separado, 30 a 200 %. Valen para el giro de 90°, el 180° y los pivotes.
- **Giro frente a paredes** a ~15 cm del frontal (`FRENTE_GIRO_M = 0.19` en `laberinto_ruedas.py`; frena a 19 y por inercia queda en ~15).
- **Enderezado**: más fuerza (`K_PSI`, `W_TOPE`, corrección brusca), tanteo de pivotes, y **prevención de choque** (una pared que se achica respecto de la otra gira 30° sin retroceder).
- **Retroceso**: 10 cm + giro por sonar (como el botón ESCAPE del panel), **máximo 2 seguidos** (`MAX_RETROCESOS`).
- **Seguidor**: flecha más larga primero (P1 > P2 > P3).

## Pruebas

```bash
python3 -m pytest -q tools/ruedas      # simulador, sin ROS ni pantalla
```
