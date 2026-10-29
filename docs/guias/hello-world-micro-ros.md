# micro-ROS en la ESP32 con ESP-IDF: "hola" por Wi-Fi

> Borrador del 2026-10-24. Destino sugerido: `docs/guias/micro-ros.md` (lo pide FW-03, #15).
> Estados: **V** verificado en la fuente · **M** medido o probado en el banco · **C** calculado.

## 1. Qué se probó y resultado

Una ESP32 (ESP-D0WD-V3) con el ejemplo `int32_publisher` del componente `micro_ros_espidf_component` (rama `jazzy`,
versión 24.0.0) se conecta por Wi-Fi a un router, habla por UDP con el **agente micro-ROS** que corre en la PC y publica un
entero que sube de a uno cada segundo en `/freertos_int32_publisher`. En la PC, `ros2 topic list` lo muestra y
`ros2 topic echo` recibe los datos (M, 2026-10-24).

Datos de la prueba (M): red 2,4 GHz, WPA2-PSK, canal 1, RSSI −45 dBm; la ESP32 obtuvo su IP por DHCP; el binario ocupó
0xcc070 bytes (20 % libre de la partición de 1 MB).

```
ESP32 --UDP 8888--> agente micro-ROS (imagen en podman/docker, PC) --> tópico ROS 2 /freertos_int32_publisher
```

## 2. Requisitos

- ESP-IDF 5.5. El componente está probado con ESP-IDF 5.2 a 6.0 y la línea **24.x corresponde a Jazzy** (V, README del componente).
- Paquetes de Python en el entorno de ESP-IDF: `pip3 install catkin_pkg colcon-common-extensions lark` (V, README).
- Imagen del agente descargada de antemano: `docker pull docker.io/microros/micro-ros-agent:jazzy` (524 MB, M).
  En la PC de desarrollo `docker` es en realidad **podman**. **La PC de competencia no tendrá internet**: la imagen tiene que
  estar en esa computadora antes. Para llevarla sin internet: `docker save -o micro-ros-agent-jazzy.tar docker.io/microros/micro-ros-agent:jazzy`
  y, en la otra PC, `docker load -i micro-ros-agent-jazzy.tar` (comandos sin probar).
- La ESP32 solo usa Wi-Fi de **2,4 GHz**.

## 3. Pasos

1. **Agente** (terminal normal, la deja corriendo): `docker run -it --rm --net=host docker.io/microros/micro-ros-agent:jazzy udp4 --port 8888`.
   Va **antes** de la ESP32: si no hay agente, la tarea de micro-ROS falla y se rinde. Para uso normal alcanza sin `-v6`
   (el nivel de detalle `-v6` imprime cada paquete).
2. **Terminal limpia de ROS** (el README del componente lo pide): `env -i HOME="$HOME" USER="$USER" TERM="$TERM" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin bash --noprofile --norc`,
   luego `. ~/esp/esp-idf-v5.5/export.sh` y comprobar que `echo "$AMENT_PREFIX_PATH"` sale vacío.
   (`bash --noprofile --norc` solo no alcanza: hereda las variables de ROS de la terminal que lo abre.)
3. **Clonar el componente fuera del repo del proyecto:** `git clone -b jazzy https://github.com/micro-ROS/micro_ros_espidf_component.git`
   y `cd micro_ros_espidf_component/examples/int32_publisher`. El ejemplo usa `override_path: "../../../"`, así que se compila **en esa carpeta**.
4. `idf.py set-target esp32` y `idf.py menuconfig`, dentro de `micro-ROS Settings`:

   | Menú | Opción | Valor |
   |---|---|---|
   | `micro-ROS network interface select` | `WLAN interface` | elegir (no trae valor por defecto) |
   | `WiFi Configuration` | `WiFi SSID` / `WiFi Password` | red y clave (solo aparece después de elegir WLAN) |
   | `XRCE-DDS Config` | `micro-ROS Agent IP` | **la IP de la PC** (por defecto trae `192.168.1.100`) |
   | `XRCE-DDS Config` | `micro-ROS Agent Port` | `8888` |

   Guardar con la **letra `S`** (no Ctrl+S) y salir con `Q`. Comprobar sin mostrar la clave:
   `grep -E "CONFIG_ESP_WIFI_SSID|CONFIG_MICRO_ROS_AGENT_(IP|PORT)|CONFIG_MICRO_ROS_ESP_NETIF_WLAN" sdkconfig`.
5. `idf.py build` (la primera vez tarda y necesita internet) y `idf.py -p /dev/ttyUSB0 -b 115200 flash monitor`.
6. En una terminal normal con ROS: `ros2 topic list` y `ros2 topic echo /freertos_int32_publisher`.

## 4. Problemas que aparecieron

| Síntoma | Causa | Solución |
|---|---|---|
| La terminal "se cuelga" en menuconfig | Se apretó **Ctrl+S**, que pausa la salida de la terminal | **Ctrl+Q** para reanudar; guardar con la letra `S` |
| `bind error ... port: 8888, errno: 98` | Ya hay un agente escuchando en ese puerto (un contenedor anterior sigue vivo) | `docker ps` y `docker stop <ID>`, o usar el que ya corre |
| La ESP32 no encuentra al agente | La IP del agente quedó en el valor por defecto `192.168.1.100` | Corregirla en menuconfig (o en `sdkconfig`) con la IP real de la PC |
| Hay que compilar sin ROS | El README del componente lo pide | Terminal limpia con `env -i` (paso 2) |

## 5. Lo que hay que tener en cuenta

- **El log imprime la clave del Wi-Fi.** Lo hace el componente (`network_interfaces/uros_wlan_netif.c`, línea 117:
  `connected to ap SSID:%s password:%s`). **No pegar el log del monitor en chats, issues, la memoria ni el repo**,
  y el firmware real no debe imprimir la clave (V, código del componente).
- **Ahorro de energía del Wi-Fi.** El log de arranque dice `wifi:pm start, type: 1`, es decir **MIN_MODEM**, el valor por
  defecto de ESP-IDF (M): la ESP32 se despierta en cada baliza (DTIM = 1, unos 102 ms). La llamada del componente a
  `esp_wifi_set_ps(WIFI_PS_MAX_MODEM)` está dentro de `#ifdef CONFIG_PM_ENABLE` y **no se activó** (V, `uros_wlan_netif.c`,
  líneas 96-99). (Una versión anterior de esta guía decía lo contrario; era un error.) FW-03 pide desactivar el ahorro
  (`WIFI_PS_NONE`) para teleoperar.
- **La IP y la red están fijas al compilar** (menuconfig). El firmware real las necesita como parámetros que se cambien **sin recompilar**
  (ADR 0002), porque en competencia la red es la de un módem de la organización, sin internet.
- La PC necesita IP estable (reserva en el router o IP fija) para que el firmware sepa dónde está el agente.

## 6. Segunda prueba: la ESP32 reacciona a `/cmd_vel` (M, 2026-10-24)

Se reemplazó `main.c` del ejemplo por una versión que **se suscribe a `/cmd_vel`** (`geometry_msgs/msg/Twist`) con
`rclc_subscription_init_best_effort`, imprime cada orden, enciende el LED de la placa (GPIO2) si la orden pide moverse y lo
apaga si es cero, y tiene un **perro guardián de 300 ms**: si no llega ninguna orden en ese tiempo imprime `FRENAR` y apaga el LED.
Solo cambió `main.c`, así que no se recompiló la librería de micro-ROS. `geometry_msgs` viene incluido porque el componente
compila `common_interfaces` de Jazzy (V, `libmicroros.mk`).

Probado con `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"` y con `teleop_twist_keyboard`:
las órdenes llegan con su signo (±0,5 m/s y ±1,0 rad/s, los valores por defecto del teleop), el LED responde y al cortar
las órdenes se imprime `FRENAR`.

**Hallazgo:** `teleop_twist_keyboard` publica al apretar teclas y, al mantenerlas, espera un retardo antes de repetir
(típicamente ~500 ms; no se midió en el banco). Ese retardo supera los 300 ms del perro guardián, por eso aparece `FRENAR`
entre ráfagas. En el sistema real la PC debe **republicar la última orden a frecuencia fija** (compuerta de `/cmd_vel`,
NAV-03) y no depender del teclado.

## 7. Tercera prueba: un motor TT con L9110S y `/cmd_vel` (M, 2026-10-24)

**Montaje.** ESP32 por USB. El módulo de baterías (celdas igualadas en 4,22 V, salida de 5,12 V sin carga) alimenta solo
el L9110S derecho: canal B (rueda delantera derecha), GPIO27 → `B-IA`, GPIO16 → `B-IB`, `A-IA` y `A-IB` a GND, GND común
con la ESP32 y el pin 5V de la ESP32 sin conectar. Rueda en el aire.

**Firmware.** Tarea de control a 100 Hz **independiente de micro-ROS** (el freno no depende del enlace): rampa con salto
inicial de 0,45 y +0,03 cada 10 ms (de 0 a 100 % en 200 ms, C), paso por 0/0 al invertir y freno a los 300 ms sin
órdenes. `duty = linear.x / 0,5`. PWM de 5 kHz y 10 bits; la señal fija del puente H queda siempre en 0.

| `linear.x` | duty | Resultado |
|---|---|---|
| 0,2 | 0,40 | no gira, solo un silbido |
| 0,3 | 0,60 | arranca con dificultad (a veces silba y arranca, a veces hubo que ayudarlo con la mano) |
| 0,4 | 0,80 | gira, velocidad moderada |
| 0,5 | 1,00 | gira bastante rápido |
| −0,2 | −0,40 | no gira, solo un silbido |
| −0,3 | −0,60 | gira |

- **Zona muerta, medida con repeticiones** (script `banco_cmdvel.py`, escalones de 0,04 en `linear.x` de 3 s cada uno, rueda en
  el aire; `duty = x / 0,5`). **Arranque desde parado:** adelante x = 0,32 / 0,24 / 0,32 y atrás x = 0,32 / 0,28 / 0,24; en
  las seis, x medio 0,287, o sea **duty medio 0,57 (rango 0,48–0,64)**. No hay diferencia clara entre sentidos (la
  asimetría de las primeras pruebas sueltas no se confirmó). **Mantener ya girando:** a x = 0,20 (duty 0,40) sigue y a
  x = 0,15 (duty 0,30) se detuvo, las tres veces: **duty de mantenimiento ~0,30–0,40**. La diferencia de ~0,2 entre
  arrancar y mantener es la fricción estática de la caja 1:48.
- **El arranque no es determinista:** con el mismo x = 0,2 (duty 0,40) arrancó "de una" en una prueba y no arrancó
  (silbido) en otra. Depende de la posición del rotor y de la caja; el umbral es una distribución, no un número.
- **No es la alimentación** (tester, CC): shield 5,09–5,11 V y `VCC` del L9110S 5,07–5,08 V con el motor arrancando,
  trabado intentando arrancar y a 100 %. Caída shield → `VCC` de 0,01–0,03 V, y bajada respecto de los 5,12 V sin carga
  de 0,01–0,03 V (promedios de un tester; no captura transitorios rápidos). Con rueda en el aire; con el robot apoyado
  los umbrales serán mayores.
- A 100 % durante ~1 minuto el chip del L9110S no se calentó y no hubo reinicios (ESP32 alimentada por USB, así que no
  se probó la caída del 5 V).
- **Frenadas periódicas, antes:** cada ~14 s (C: ~70 líneas `duty=` de 200 ms entre dos `FRENAR`) el perro guardián
  detectaba un hueco de más de 300 ms entre órdenes y frenaba. **Con el ahorro de energía del Wi-Fi desactivado**
  (`esp_wifi_set_ps(WIFI_PS_NONE)`) y `ros2 topic pub -r 10`: huecos medidos de 226, 261, 284, 263, 213, 389, 260 y
  277 ms (mediana 262 ms); **uno solo superó los 300 ms** y produjo un `FRENAR`. Mejoró, pero el enlace sigue
  teniendo huecos de 2 a 4 períodos. Causa sin determinar (candidatos: radio o router, la PC por Wi-Fi, el agente).
  Diagnóstico propuesto: `ping -i 0.2 -c 300 <IP de la ESP32>` desde la PC, y saber si la PC va por cable o por Wi-Fi.

- **Ping PC → ESP32** (la PC va por Wi-Fi, sin cable de red disponible; 273 paquetes cada 0,2 s; M, 2026-10-25): 0 % de
  pérdida; mínimo 3,1 ms, mediana 15,3 ms, promedio 41,2 ms, **máximo 404 ms**; percentil 90 = 119 ms y percentil 95 = 156 ms;
  39 paquetes (14 %) pasaron de 100 ms, 11 (4 %) de 200 ms y 1 de 300 ms. Hay secuencias donde varios paquetes llegan juntos
  después de una demora (404 → 197 → 6,6 ms). Es compatible con los huecos de 200 a 390 ms entre órdenes. **Origen sin
  determinar** (candidatos: el tramo PC–router, el ahorro de energía de la placa Wi-Fi de la PC, la saturación del canal 1,
  el router).

- **Ping PC → router** (muestra corta, 25 paquetes en ~5 s; M): mín 1,6 / prom 9,0 / máx 51 ms; se respondieron 24 de 25 (el
  paquete 25 seguía en vuelo al cortar con Ctrl+C: no es pérdida real). Es muy poco tiempo para comparar con los 273 paquetes
  a la ESP32; hace falta una medición larga y simultánea.
- **Redes que ve la PC** (`nmcli`): la PC probablemente está en **5 GHz, canal 52 (canal DFS), señal 46 %**; la banda de 2,4 GHz de
  `Personal-443` (canal 1) la ve a 25 %. En 2,4 GHz el canal 1 está poco cargado (vecinos a 2–12 %); el canal 11 tiene redes más
  fuertes (hasta 35 %). Entonces la saturación del canal 1 no es el primer sospechoso.

### Velocidad contra duty (encoder, rueda en el aire, M, 2026-10-25)

Firmware con empujón de 0,70 durante 100 ms, ventana de 500 ms, PWM de 5 kHz. Promedio de los últimos 5 valores de cada escalón.

| duty | flancos/s subiendo (desde parado) | flancos/s bajando (sin parar) |
|---|---|---|
| 0,40 | 31,6 | 32,0 |
| 0,50 | 58,0 | 64,8 |
| 0,60 | 87,2 | 93,2 |
| 0,70 | 119,2 | 118,8 |
| 0,80 | 132,4 | 133,2 |
| 0,90 | 175,6 | 172,4 |
| 1,00 | 217,6 | 213,0 |

- **Casi lineal con zona muerta:** ajuste flancos/s ≈ 299·(duty − 0,31), R² = 0,986 (C). El corte en duty ≈ 0,31 coincide con el
  mantenimiento medido: a duty 0,40 sigue girando y a 0,30 se detiene (30 → 12 → 0 flancos/s).
- **Con empujón arranca desde duty 0,36–0,40** (antes, sin empujón, desde 0,48–0,64).
- **Histéresis pequeña** entre subir y bajar (hasta +7 flancos/s a duty 0,5–0,6).
- **Salto raro entre duty 0,70 y 0,80** (+13 flancos/s, contra +26 a +43 en los otros tramos), reproducible en las dos
  pruebas. Causa sin determinar; repetir con más puntos (0,72, 0,75, 0,78) y con el robot en el suelo.
- **Constante de tiempo gruesa ~0,5 s** (con ventanas de 500 ms, una estimación). Sirve para elegir el período de control.
- **Velocidad máxima: ~217 flancos/s** a duty 1,0. La propuesta estimaba ~115 flancos/s.
- **Resolución de la ventana:** a la velocidad mínima que se sostiene (31,6 flancos/s), 1 flanco de diferencia es ±32 % con
  ventana de 100 ms, ±16 % con 200 ms y ±6 % con 500 ms. A velocidad máxima, ±4,6 % con 100 ms (C).

### Flancos por vuelta de rueda: 40 (resuelto, M, 2026-10-25)

Dudábamos entre 40 y 80 porque a 217 flancos/s la rueda resultaba más rápida que la especificación del kit. Se midió de dos formas:

- **Conteo con el motor andando:** `banco_cmdvel.py 0.20:20` (duty 0,40). La rueda dio **16,5 vueltas** en 20 s (conteo a ojo, con una
  marca en la cubierta) y el encoder acumuló ~650 flancos (642 a 658 según se cuente la última ventana): **39–40 flancos por
  vuelta**. Con 80 habrían sido ~8 vueltas.
- **Cuarto de vuelta a mano:** al girar la rueda un cuarto de vuelta, el disco gira un cuarto. Están en el mismo eje.

**Conclusión: 40 flancos por vuelta de rueda** (20 ranuras × 2 flancos) y **5,26 mm por flanco** con una rueda de 67 mm (C). La
sospecha de 80 estaba equivocada. Lo que no coincide es la velocidad: a duty 1,0 se miden 217 flancos/s = 5,4 vueltas/s = 326 rpm
= 1,14 m/s (rueda en el aire, sin carga), unas 2,4 veces lo que da la especificación del kit a ~4,3 V (~138 rpm; P). El motor
del equipo es más rápido que esa ficha, o la ficha es un mínimo; para el diseño vale lo medido. Con el ajuste de la sección
anterior, **0,5 m/s equivalen a ~95 flancos/s, es decir duty ≈ 0,63** (C), y la velocidad mínima que se sostiene (duty 0,40) es
**~0,17 m/s** (C). Sin carga: con el robot apoyado los números van a cambiar.

## 8. Qué falta para el firmware real (FW-03)

- Suscripción best-effort a `/cmd_vel` y reacción de la ESP32.
- Reconexión y frenado ante pérdida de enlace.
- Parámetros de red en la memoria no volátil.
- Proyecto propio que dependa del componente (`idf.py add-dependency "micro-ros/micro_ros_espidf_component^24"`) en vez de compilar dentro del clon.
- Decidir y registrar en un ADR el paso de PlatformIO a ESP-IDF (el texto actual de FW-01 a FW-03 habla de PlatformIO).
