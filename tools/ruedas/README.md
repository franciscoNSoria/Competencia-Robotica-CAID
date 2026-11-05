# Panel de las 4 ruedas: barras, 1 m, giros y odometría

Ventana de la PC para probar las 4 ruedas con sus encoders por Wi-Fi (micro-ROS), como el día de la competencia.
Estados: **M** medido, **C** calculado, **P** propuesta sin comprobar, **S** simulado en la PC (modelo simple de motor, no es el robot).

> **Laberinto con flechas hasta el cubo:** ver [FLECHAS_Y_CUBO.md](FLECHAS_Y_CUBO.md) (cómo se corre, qué mirar en el log).

## Qué hay

| Archivo | Qué es |
|---|---|
| `ruedas_panel.py` | La ventana (Tk) y el nodo ROS 2. Es lo que se corre |
| `ruedas_logica.py` | Todo lo que decide: barras con rampa de aceleración, maniobras, odometría, correcciones y archivos. Sin ventana ni ROS |
| `test_ruedas_logica.py`, `test_ruedas_panel.py` | Pruebas con un robot simulado (sin ROS ni ESP32) |
| `../../firmware/banco/robot_4ruedas/main.c` | El firmware que va en la ESP32 (el de siempre, más el giro en el lugar: orden 7) |

El protocolo (`/robot_cmd`, `/cmd_vel`, `/wheel_ticks`, `/wheel_state`) está en `docs/guias/robot-4-ruedas-pid-y-comunicacion.md`.

## Cómo se corre

1. **Firmware** (una vez; la ESP32 con las ruedas **no puede tener a la vez** el firmware del brazo: es uno u otro). Terminal limpia de ROS
   (`env -i HOME="$HOME" USER="$USER" TERM="$TERM" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin bash --noprofile --norc`),
   `. ~/esp/esp-idf-v5.5/export.sh`, y en la carpeta del ejemplo `int32_publisher` del componente micro-ROS:
   copiar `firmware/banco/robot_4ruedas/main.c` sobre `main/main.c`, `idf.py build` y, con la ESP32 **fuera del shield** (nunca USB y riel de 5 V a la vez),
   `idf.py -p /dev/ttyUSB0 -b 115200 flash`. No uses `monitor`: el componente imprime la clave del Wi-Fi.
   No repitas `set-target`: borra la configuración (red, IP del agente) del `sdkconfig`.
2. **Agente** (terminal normal, antes de encender el robot): `docker run -it --rm --net=host docker.io/microros/micro-ros-agent:jazzy udp4 --port 8888`
3. **Panel** (otra terminal): `source /opt/ros/jazzy/setup.bash` y `python3 tools/ruedas/ruedas_panel.py`. Necesita `python3-tk`.

El LED azul parpadea mientras la ESP32 busca al agente y se apaga al conectar; la ventana dice "ESP32 conectada".
Todo lo que pasa se escribe también en la terminal. **Esc** o el botón rojo frenan todo.

## Qué hace cada cosa

- **Barras (una por rueda).** Solo mueven las ruedas con **ARMADO** tildado (al armar o desarmar vuelven a 0).
  - *Duty*: la potencia va directo al motor, sin PI (orden 1). Sirve para ver cómo responde cada motor y si su encoder cuenta.
  - *Velocidad (PI)*: m/s por rueda con el PI del firmware (orden 2). Mínimo sostenible ~0,13 m/s; por debajo de 0,07 m/s el firmware lo toma como 0.
  - *Aceleración*: cuánto puede cambiar cada barra por segundo (% del fondo de escala). La PC sube el valor de a poco para no pegar 4 arranques
    a fondo juntos contra el elevador de 5 V (en el banco, un motor trabado reinició la ESP32). *Enlazar las 4* mueve las cuatro barras juntas.
- **Tabla.** Por rueda: lo pedido por la PC, el objetivo del firmware, la velocidad medida (m/s y flancos/s), el error contra el objetivo, el desvío contra
  el promedio de las 4 ruedas (ámbar si pasa de 15 %), el duty que sale, los ticks desde el último cero y si la rueda quedó bloqueada.
  **Odometría**: x, y, giro (sin envolver: un giro de 360° marca 360), avance y recorrido, integrados con los ticks (`flancos / 190,04 por metro`
  con rueda de 67 mm, **C**). El encoder es de un solo canal: el sentido sale del último duty, y el resbalamiento no se ve.
- **1 m adelante / atrás, Giro 180° / 360°** (izquierda o derecha), o distancia y ángulo a elección: la ESP32 los ejecuta con rampas y los mide con
  el promedio de los flancos de las 4 ruedas. Al terminar se imprime lo pedido, lo calculado por la ESP32 y por la odometría de la PC, y los ticks de cada rueda.
  - Medís en el piso (cinta para el metro, transportador o marca para el giro, y cuánto se corrió el centro) y lo cargás abajo.
  - **REGISTRAR** lo anota en `~/caid/config/ruedas_maniobras.csv`. **REGISTRAR Y CORREGIR** además corrige y manda a la ESP32:
    - *metro o marcha*: `diámetro nuevo = diámetro × real / calculado`;
    - *giro*: `trocha nueva = trocha × calculado / real` (el robot patina: gira menos de lo que dicen los encoders).
  - Casilla *ajustar solo el adelanto de frenado*: si el robot se pasa (o se queda corto) de la meta, el adelanto se corrige solo, como en `robot_teclado.py`.
  - Orden recomendado: primero el diámetro con el metro, después la trocha con los giros (el diámetro entra en la cuenta del arco).
- **Marchar**: `/cmd_vel` (m/s y rad/s) durante unos segundos, a 10 Hz; frena solo al final. Compara `v × t` contra la odometría y contra tu cinta.
- **Grabar CSV**: guarda a 10 Hz lo pedido, el objetivo, la velocidad medida, el duty, los ticks y la odometría en `~/caid/config/ruedas_AAAAMMDD_HHMMSS.csv`,
  para comparar los comandos contra los tacómetros en una planilla.

Si la ESP32 se reinicia o se reconecta, el panel le reenvía lo guardado en `~/caid/config/calibracion.json` (diámetro, trocha, adelantos de frenado y las curvas
de `robot_teclado.py`); la ESP32 no guarda nada al apagarse.

## Orden sugerido para la primera vez

1. **En el aire** (robot sobre un soporte): modo *Duty*, una rueda por vez con aceleración baja (25 a 50 %/s). Mirar que gire hacia adelante con duty positivo
   y que sus ticks suban. Si una gira al revés, cambiar `polaridad` en `W[]` de `main.c` o dar vuelta los cables del motor.
   La rueda FR no contó flancos en la calibración del 28/10 y se cortó el enlace (**P**: sobrecorriente que hunde el 5 V): probarla sola y mirar la ventana por si la ESP32 se reinicia.
2. En el aire, modo *Velocidad*: pedir 0,15 y 0,25 m/s a las 4 y mirar la columna "Error" y "vs media".
3. **En el piso**, con 2 m libres: *1 m adelante*, medir con cinta y *REGISTRAR Y CORREGIR*; repetir 3 veces para ver la dispersión. Después *1 m atrás*.
4. *Giro 360°* a la izquierda y a la derecha: ver con una marca cuánto gira de verdad y cuánto se corre el centro; *REGISTRAR Y CORREGIR* para la trocha efectiva.
5. *Marchar* a 0,15 y 0,25 m/s durante 4 s y comparar con la cinta.

## Pruebas del código

```
python3 -m pytest tools/ruedas/test_ruedas_logica.py                  # lógica: robot simulado, sin pantalla
xvfb-run -a python3 -m unittest tools/ruedas/test_ruedas_panel.py     # ventana (necesita tkinter; en una PC con pantalla basta sin xvfb-run)
```

Lo que prueban (**S**): el protocolo, las rampas, la odometría, las correcciones y los archivos. Lo que **no** pueden probar: que el robot real frene donde se espera,
cuánto patina en un giro, ni el nodo ROS 2 (`crear_enlace_ros`: en la nube no hay ROS 2; usa los mismos tipos y QoS que `robot_teclado.py`).
