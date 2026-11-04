# Freno por sonar y autonomía básica ("vagabundo")

> Estados: **M** medido, **C** calculado, **P** supuesto o propuesta sin probar, **S** simulado.
> Versión del 4/11/2026. Todo lo de esta guía está **sin probar en el piso**: lo único medido con el robot es el sonar de banco
> (frente 35,2 cm y derecha 53,3 cm con 0 % de pérdidas; izquierda a 5,5 cm con 32 % de pérdidas, M).

## 1. Qué hace

Dos capas. La PC decide a dónde ir; el freno no depende del Wi-Fi (huecos de 200–400 ms entre órdenes, M).

```
PC (tools/autonomo/robot_autonomo.py)                     ESP32 (firmware/banco/robot_4ruedas/main.c)
  vagabundo: avanza, gira al ver un obstáculo    /cmd_vel   guardia: topa la velocidad hacia adelante con el sonar frontal
  lee /wheel_state [26..33]  <------------------ 10 Hz ---  sonares: frontal cada ~120 ms, laterales cada ~240 ms
  pide la guardia cada 1 s  --- /robot_cmd id 8 -------->   (apagada al arrancar; la PC la activa)
```

## 2. La guardia (firmware, `firmware/pruebas/comun/guardia.h`)

Cada 10 ms mira el sonar frontal y topa la velocidad **lineal hacia adelante**. Girar en el lugar y retroceder siguen permitidos.

| Estado ([29]) | Cuándo | Tope |
|---|---|---|
| 0 apagada | la PC no la activó (es el estado al arrancar) | ninguno |
| 1 libre | frente ≥ `d_lento` | ninguno |
| 2 lenta | `d_stop` < frente < `d_lento` | 25 flanc/s (0,13 m/s, C), lo menos que sostienen los 4 motores |
| 3 frenada | frente ≤ `d_stop` | 0 |
| 4 falla | sin medición terminada en 400 ms, o sin ningún eco hace más de `ciego_s` | 0 |

- Se libera con 2 mediciones válidas seguidas por encima de `d_stop` + 3 cm. Una medición **sin eco no libera** (un obstáculo muy cerca
  o en ángulo también da "sin eco"), y tampoco borra de golpe la última distancia válida: se conserva 600 ms.
- Orden `/robot_cmd`: `[8, activa, d_stop_m, d_lento_m, ciego_s]` (el último es opcional; por defecto 5 s). Valores fuera de rango
  (`d_stop` 0,05–0,60; `d_lento` ≥ `d_stop` + 0,05 y ≤ 2; `ciego_s` 1–600) se rechazan y no cambian nada.
- Un movimiento de distancia hacia adelante (id 3) se aborta si la guardia frena. El duty directo (id 1, calibración) no se toca.
- `/wheel_state` pasa de 26 a 34 valores: `[26]` frontal, `[27]` izquierdo, `[28]` derecho (m, −1 = sin eco), `[29]` estado,
  `[30]` `d_stop`, `[31]` `d_lento`, `[32]` mediciones terminadas del frontal, `[33]` `ciego_s`. Los lectores viejos indexan por posición y no se enteran.
- Sonares: frontal TRIG 5 / ECHO 13, derecho 15 / 4, izquierdo 14 / 12. Un disparo cada 60 ms en la secuencia frontal, derecho, frontal,
  izquierdo.

`firmware/pruebas/test_guardia.c` prueba la lógica en la PC (142 verificaciones):
`gcc -Wall -Wextra -Werror -o /tmp/test_guardia firmware/pruebas/test_guardia.c -I firmware/pruebas/comun -lm && /tmp/test_guardia`

## 3. El vagabundo (PC, `tools/autonomo/`)

| Estado | Qué hace |
|---|---|
| ESPERA | quieto hasta que se arma con `g` |
| AVANZAR | derecho a 0,20 m/s; si un costado está a menos de 22 cm, se corre hacia el otro lado (0,35 rad/s) |
| GIRAR | en el lugar (1,2 rad/s) hacia el lado con más espacio; termina con 2 mediciones del frente ≥ 70 cm, después de 0,6 s como mínimo y de un tiempo extra al azar de 0 a 1 s (pasada media vuelta, 4 s, se conforma con 55 cm) |
| DETENIDO | quieto: encajonado, giro de 8 s sin salida, sonar frontal mudo, telemetría perdida más de 3 s, o espacio/Esc. Se vuelve a armar con `g` |

- Empieza a girar con algo a menos de 45 cm del frente: la cadena PC → Wi-Fi → ESP32 tarda ~0,5 s, y a 0,13–0,20 m/s son 7–10 cm.
- No gira en el lugar con un lado a menos de 12 cm: girar barre las dos esquinas diagonales hacia afuera hasta 16 cm de radio, 8 cm más
  que el costado del robot (C). En ese caso queda DETENIDO.
- Un lado o el frente sin eco cuentan como libres al avanzar, pero **no** terminan un giro (hace falta una distancia válida).
- Sin telemetría 0,6 s se frena; si vuelve antes de 3 s sigue, si no hay que armar con `g`. Tope de 10 min por corrida.
- Guarda `~/caid/config/autonomo_AAAAMMDD_HHMMSS.csv` (estado, órdenes, sonares, guardia y velocidad de las 4 ruedas a 10 Hz).
- Parámetros: `--v`, `--giro`, `--evitar`, `--libre`, `--lateral`, `--d-stop`, `--d-lento`, `--ciego`, `--minutos`, `--semilla`.

## 4. Cómo probarlo, en este orden

**A. Grabar** (ESP32 fuera del shield, terminal limpia de ESP-IDF, sin monitor):

```bash
cd ~/caid/Competencia-Robotica-CAID/firmware/pruebas
./compilar.sh ../banco/robot_4ruedas/main /dev/ttyUSB0
```

El mensaje final del script que nombra `prueba.py` no corresponde a este firmware: ignorarlo.

**B. La guardia sola** (robot en el piso, con 1,5 m de pista hacia una pared o una caja grande): agente micro-ROS, y en otra terminal con
ROS `python3 tools/autonomo/robot_autonomo.py --solo-guardia` (muestra los sonares y "guardia libre"). Con el panel de ruedas
(`tools/ruedas/ruedas_panel.py`): "Marchar" a 0,20 m/s hacia la pared.
Esperado (S): baja a 0,13 m/s cuando el frente pasa los 60 cm y se detiene entre 17 y 20 cm; con el panel mandando adelante, el robot se
queda quieto. Medir con regla la distancia del frente del robot a la pared: si el sonar no está al ras del frente, la diferencia hay que
sumarla a `d_stop`. Probar también una mano que aparece de golpe, y un giro en el lugar con la guardia frenada (tiene que girar).

**C. El vagabundo** (un cuarto despejado, escaleras tapadas, sin patas finas ni cables en el piso, una mano en ESPACIO):
`python3 tools/autonomo/robot_autonomo.py` y `g`.

**D. El cono real del sonar** (3 min con la prueba 13): un objeto fino (un marcador, una botella de 4 cm) a 30 cm del frente y corrido
0, 5, 10 y 15 cm al costado; y una pared (un libro) inclinada 30°, 45° y 60° a 30 cm. Anotar cuáles se ven. De ahí sale cuánto vale
`haz_grados` del simulador (hoy 18°, P).

## 5. Qué esperar (S) y qué no

Simulador 2D (`tools/autonomo/simular.py`, usa el mismo C de la guardia por ctypes): 30 corridas de 5 min, cuartos de 3–5 × 2,5–4 m con
3–7 obstáculos, una pata de silla de 4 cm, y una pared inclinada. Con los errores del robot medidos el 3/11 y el resto supuesto:

| Variante | Corridas de 5 min sin choque | Detenciones |
|---|---|---|
| Modelo nominal, con guardia | 28 / 30 | 9 en 150 min |
| Modelo pesimista (haz de 14°, eco más débil, más pérdidas y huecos de Wi-Fi) | 24 / 30 | 12 |
| Nominal **sin** la guardia (solo la PC) | 19 / 30 | 7 |

La guardia suma unos 30 puntos: la PC sola, con su demora, no alcanza. Frente a una pared de frente, la guardia sola (a 0,20 m/s) deja
el robot a 1–3 cm de `d_stop`, sin choques en 40 corridas (S).

**Lo que sigue chocando** (S): objetos finos (patas) en el sector diagonal que no cubren ni el sonar frontal ni los laterales, y
el arranque de un giro con un objeto fino a 12 cm del costado. No es un fallo de la lógica: el hardware no lo ve. Tampoco ven escaleras,
vidrio ni telas gruesas, y una pared muy inclinada devuelve poco eco. Con 3 sonares, "no choca" no se puede garantizar: se puede
bajar la frecuencia y la fuerza de los toques.

## 6. Pendiente

- Probar B, C y D en el robot y reemplazar los supuestos (P) del simulador: haz, fuerza del eco, arranque tardío al invertir una rueda
  (en el simulador 0,25–0,9 s), barrido real de un giro, posición de los sonares en el chasis.
- Si D confirma el sector ciego: sonares laterales girados 30–40° hacia adelante, o un barrido de rumbo mientras avanza. El montaje tiene
  que cumplir la inspección de la competencia (Art. 10).
- Para el laberinto (pasillos de 32 cm) estos valores no sirven: `d_stop`, `d_lento`, `d_evitar` y `d_barrido` tienen que bajar mucho y el
  giro tiene que cerrarse con las paredes. La guardia ya acepta esos valores por la orden 8.
- Reconexión rápida del agente (hoy 6–7 s de detección, P) y nivel de empujón de las ruedas delanteras: siguen pendientes en el firmware.
