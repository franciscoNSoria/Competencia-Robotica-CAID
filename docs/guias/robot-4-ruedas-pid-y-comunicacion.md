# Robot de 4 ruedas: lazo de velocidad (PID) y comunicación por micro-ROS

> Documenta lo hecho en el banco para FW-06 (#78) y la comunicación de FW-03 (#15).
> Estados: **M** medido en el banco · **C** calculado · **P** hipótesis o propuesta sin comprobar · **S** solo simulado en la PC (modelo simple de motor, no es el robot).
> Código: `firmware/banco/robot_4ruedas/main.c` (ESP32) y `tools/banco/robot_teclado.py` (PC).

## 1. Qué hay

```
PC (ROS 2 Jazzy)                                       ESP32 (ESP-IDF 5.5, micro_ros_espidf_component)
 robot_teclado.py  ──/cmd_vel, /robot_cmd──►  agente micro-ROS (UDP 8888)  ──►  PI por rueda (4) a 100 Hz
                   ◄──/wheel_ticks, /wheel_state──                               encoders por PCNT, motores L9110S
```

Todo es inalámbrico: la ESP32 se conecta por Wi-Fi al agente (guía `micro-ros.md`). No hace falta ningún botón en el robot.

## 2. Tópicos

| Tópico | Tipo | Sentido | Contenido |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | PC → ESP32 | `linear.x` en m/s y `angular.z` en rad/s (cinemática diferencial). **Es m/s de verdad**: en la versión de una rueda (v8d) 0,5 equivalía a 75 flancos/s |
| `/robot_cmd` | `std_msgs/Float32MultiArray` | PC → ESP32 | `[id, argumentos...]`, ver abajo |
| `/wheel_ticks` | `std_msgs/Int32MultiArray` | ESP32 → PC | 4 flancos acumulados **con signo** `[FL, FR, RL, RR]`, 20 Hz |
| `/wheel_state` | `std_msgs/Float32MultiArray` | ESP32 → PC | 24 valores, 10 Hz, ver abajo |

El encoder es de un solo canal: no da el sentido de giro. El signo de los flancos es el del último duty distinto de 0 (**P**: con la rueda frenando por inercia en sentido contrario al comando, el signo puede equivocarse unos flancos).

Los tipos son de `std_msgs` para no recompilar micro-ROS con mensajes propios; los define este documento hasta que INF-02 (#4) fije el contrato de interfaces.

### `/robot_cmd`

| id | Argumentos | Qué hace |
|---|---|---|
| 1 | `d0 d1 d2 d3` | Duty directo por rueda, en lazo abierto (calibración). 0 deja esa rueda libre |
| 2 | `v0 v1 v2 v3` | Velocidad por rueda en m/s, con PI (orden `FL, FR, RL, RR`) |
| 3 | `dist_m vel` | Mover `dist_m` (con signo) con rampas. `vel` en flancos/s; 0 usa 40 |
| 4 | — | Parar todo |
| 5 | `diametro trocha frenado` | Configuración en metros: diámetro efectivo, trocha efectiva, adelanto de frenado |
| 6 | `rueda d_min n v1 d1 v2 d2 ...` | Curva de prealimentación de una rueda (hasta 10 puntos) |

### `/wheel_state` (24 valores)

| Índices | Contenido |
|---|---|
| 0 a 3 | Velocidad medida, flancos/s con signo |
| 4 a 7 | Velocidad pedida, flancos/s |
| 8 a 11 | Duty con signo que sale al motor |
| 12 a 15 | Rueda bloqueada (1) o no (0) |
| 16 | Movimiento de distancia activo |
| 17 | Recorrido actual, m con signo |
| 18 | Recorrido final, m |
| 19 | Contador de movimientos terminados |
| 20 | Diámetro efectivo, mm |
| 21 | Adelanto de frenado, mm |
| 22 | Contador de configuraciones y curvas aceptadas |
| 23 | Trocha efectiva, mm |

El componente micro-ROS (rama `jazzy`) trae por defecto 2 publicadores y 2 suscriptores (leído en su `colcon.meta`, **V**), que es justo lo que usa el firmware. Un tópico más pide cambiar ese límite.

## 3. Control de velocidad por rueda

Una instancia por rueda, a 100 Hz, independiente de micro-ROS (el freno no depende del enlace):

- **Medición:** flancos de los últimos 300 ms (30 ciclos). Con 40 flancos/s son unos 12 flancos por ventana, o sea ±8 % de cuantización por muestra (**C**).
- **Prealimentación:** el duty que dio esa velocidad en la curva medida de cada motor (los dos delanteros iguales; RL y RR cada una la suya, porque son otro motor, `caracterizacion-motores.md`).
- **PI** cada 50 ms: `KP = 0,0020` y `KI = 0,0030` (duty por flanco/s de error), integral acotada a ±0,40, cambio de duty máximo de 0,15 por actualización, anti-windup cuando el duty satura.
- **Duty mínimo que sostiene el giro** (**M**): FL y FR 0,36; RL 0,40; RR 0,50.
- **Empujón de arranque progresivo:** duty 0,70 / 0,85 / 1,00 / 1,00 durante 100 / 150 / 250 / 400 ms. Los traseros empiezan en el nivel 3 (duty 1,00). Se dispara por velocidad medida igual a cero con orden activa; tras 6 empujones sin flancos, la rueda queda **bloqueada** y se detiene.
- **Arranque escalonado:** cada rueda empieza 100 ms después de la anterior, para no hundir el 5 V con 4 empujones juntos.
- **Rango de velocidad:** mínimo 25 flancos/s (unos 0,13 m/s) y máximo 75 flancos/s (0,395 m/s) por rueda. Por debajo de la mitad del mínimo la orden se trata como 0. Si un giro pide más de lo posible, se escalan las dos ruedas manteniendo la curvatura.
- **Watchdog:** sin órdenes por más de 1000 ms, frena (banco; en la competencia lo cubre el freno local con el sonar, FW-04).

Equivalencias (**C**, rueda de 67 mm del CAD): 40 flancos por vuelta, 190,04 flancos por metro, 40 flancos/s = 0,2105 m/s.

### Marcha de distancia (id 3)

Rampa de arranque y de frenado de 100 flancos/s², velocidad constante en el medio, y se corta un **adelanto de frenado** antes de la meta para que la inercia complete el recorrido. El recorrido se calcula con el promedio de los flancos de las 4 ruedas y se informa 1,5 s después de parar. Valor inicial del adelanto: 30 mm; el programa de PC lo corrige solo.

## 4. Herramienta de PC (`tools/banco/robot_teclado.py`)

| Tecla | Qué hace |
|---|---|
| `c` | Calibración de las 4 ruedas, **en el aire**. Barre el duty de 1,00 a 0,36 en 9 niveles de 2,8 s por rueda, mide la velocidad en los últimos 1,2 s (pendiente de mínimos cuadrados de los flancos, robusta al retardo del Wi-Fi), compara con la curva programada, arma la curva de cada rueda y la manda a la ESP32. Verifica con las 4 a una velocidad común: **IGUALADAS** si la dispersión es de 5 % o menos |
| `f` / `b` | Marcha de 1 m adelante / atrás en el piso. Ajusta el adelanto de frenado hasta que el recorrido calculado sea el pedido, y pide los metros medidos con cinta para corregir el diámetro efectivo: `diámetro nuevo = diámetro × real / calculado` |
| `w s a d`, Espacio | Manejo manual y parada |
| `e`, `q` | Estado / salir |

La calibración y el diámetro se guardan en `~/caid/config/calibracion.json` y se reenvían solos cada vez que la ESP32 se conecta (la ESP32 no guarda nada al apagarse). Las marchas se registran en `~/caid/config/marchas.csv`. **Ese archivo hay que llevarlo a la PC de la competencia.**

`tools/banco/calibracion_ruedas.py` es la versión por pruebas sueltas (`aire`, `piso`, `metro`, `escalones`), con CSV crudo de cada corrida.

## 5. Cómo se corre

1. Agente (primero): `docker run -it --rm --net=host docker.io/microros/micro-ros-agent:jazzy udp4 --port 8888`.
2. Firmware (terminal limpia de ESP-IDF, con el cable del 5 V a la ESP32 desconectado): copiar `firmware/banco/robot_4ruedas/main.c` a `main/main.c` del ejemplo `int32_publisher` y `idf.py -p /dev/ttyUSB0 -b 115200 flash`. Pasos de `micro-ros.md`.
3. Robot encendido: el LED parpadea mientras espera al agente y se apaga al conectar. Después, en otra terminal con ROS cargado: `python3 tools/banco/robot_teclado.py`.

## 6. Resultados

- **M (2026-10-28):** conexión inalámbrica ESP32 ↔ agente ↔ `robot_teclado.py` funcionando. La rueda **FL** calibró bien en dos corridas: a duty 1,00 dio 210,6 y 211,7 flancos/s (banco: 221,6), y a duty 0,50 dio 66,6 y 68,3 (banco: 73,2); queda entre 2 y 10 % por debajo del banco (salvo el nivel de 0,36: −15 % y +22 %), dentro de la repetibilidad ya medida (±8 %) salvo ese nivel. Motor, encoder, PWM y enlace de esa rueda están bien.
- **M:** la rueda **FR no contó ningún flanco a duty 1,00** en dos corridas, y las dos veces se cortó el enlace después. Hipótesis (**P**): sobrecorriente de ese motor que hunde el 5 V y reinicia la ESP32. Sin diagnosticar: faltan las pruebas de encoder (girarla a mano), tensión en bornes y cambio de motor con la FL.
- **M:** la RL "no paraba" por un error de cableado de las señales (ya resuelto).
- **S:** con un modelo simple de motor, el PI converge a 40 flancos/s en las 4 ruedas, también con carga (85 % de velocidad y +0,05 de duty). El movimiento de 1 m se pasa unos 6 cm si el adelanto de frenado es de 10 mm, por la inercia del modelo; por eso el adelanto se ajusta solo en la PC. **No hay medida en el piso todavía.**

## 7. Pendiente

- Diagnosticar la FR y repetir `c` hasta que las 4 den IGUALADAS.
- Pruebas en el piso: `f` y `b` de 1 m con cinta; resbalamiento (el encoder no lo ve, solo la medición real); trocha efectiva con un giro (el robot es de tracción por patinaje y la trocha efectiva es mayor que la geométrica de 131 mm).
- Potencia: el elevador del módulo de baterías (3 A según vendedores) alimenta todo junto; un motor trabado puede reiniciar la ESP32. Alternativas en `hardware/electronica/propuesta-v0.2.md` (segundo módulo) y el driver DRV8833 (0,36 Ω de resistencia contra la caída de 1,0 a 1,15 V del L9110S).
- **Estructura del firmware:** hoy el código vive en `firmware/banco/` como `main.c` sueltos que se copian al ejemplo de micro-ROS. La rama `feature/FW-esp32` propone otra estructura (PlatformIO, dos núcleos). Hay que acordar cuál es `firmware/esp32_motion/` y decidirlo en el ADR 0003 antes de mover el código.
- Seguridad: el ejemplo de micro-ROS imprime la contraseña del Wi-Fi en cada arranque. No pegar nunca el log del monitor en chats, issues ni el repo.
