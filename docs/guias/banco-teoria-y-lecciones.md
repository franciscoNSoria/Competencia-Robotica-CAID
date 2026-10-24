# Banco de pruebas: teoría y lecciones aprendidas

> Borrador del 2026-10-24. Destino sugerido: `docs/guias/banco-teoria-y-lecciones.md`.
> Se actualiza a medida que se mide. Sirve de insumo para `hardware/electronica/mediciones.md` (FW-01)
> y para la memoria (`03-hardware.md`, `04-firmware-y-comunicacion.md`).

**Estados de cada dato** (mismos que `propuesta-v0.2.md`): **V** verificado en una fuente · **P** parcial
(vendedor, hoja de otro fabricante) · **C** calculado, con la fórmula al lado · **M** medido en el banco por el equipo.

## 0. Cómo leer los IDs del tablero

Cada tarea se llama `[PREFIJO-NN]`. El prefijo es la **columna** del tablero y el número, el orden dentro de ella.
Ejemplo: `FW-02` es la segunda tarea de la columna Firmware: "Motores L9110S: 2 PWM por canal, rampa, tope de giro,
corte por bloqueo y watchdog". Otros prefijos: INF (infraestructura), CAM, PER, NAV, MAN, SIM, VIS, MIS, REAL (hardware real).
La columna y el orden están en la primera línea del cuerpo de cada issue.

## 1. Ultrasonido (HC-SR04 frontal, RCWL-9610A laterales)

- **Cómo mide.** El módulo emite un pulso de 40 kHz y devuelve en ECHO un pulso cuyo ancho es el tiempo de ida y vuelta.
  distancia = t × 0,343 mm/µs / 2 = t × 0,1715 mm/µs (C, con 343 m/s a 20 °C).
- **Entrega una sola distancia:** la del eco más cercano dentro del cono. No informa el ángulo (M).
- **Ángulo del haz.** HC-SR04: 15° nominal según la hoja de ElecFreaks, sin definir cómo se midió (P). RCWL-9610A: no se
  encontró el dato; hay que medirlo (pendiente).
- **Eco de fondo.** Cualquier superficie dentro del cono (mesa, teclado, pared) devuelve eco. Si el objetivo es chico, está
  inclinado o fuera del cono, **gana el fondo** y la lectura parece "pegada" (en el banco: ~461 mm sin objetivo y ~497 mm
  contra el escritorio) (M). Lección: objetivo plano y grande, sensor elevado, sin apuntar al piso.
- **Precisión contra exactitud.** Con objetivo fijo el desvío fue de 0,06 a 0,09 mm (M): muy repetible. La exactitud **no
  está evaluada**: con el objetivo posicionado a mano (±1 cm) la lectura quedó 3–7 % por debajo de lo nominal a 20–60 cm.
  Pendiente: test con montaje rígido (cinta en el piso, pared plana) en 10, 20, 30, 50, 80 y 100 cm.
- **Velocidad del sonido.** c ≈ 331,3 + 0,606·T (m/s, T en °C): 343,4 a 20 °C y 352,5 a 35 °C (C). El efecto es de ~1 %
  a temperatura ambiente, así que no explica un sesgo del 6 %.
- **RCWL-9610A.** Pines por nombre: `Trig_Rx_SCL_I/O` (TRIG) y `Echo_Tx_SDA` (ECHO). Según el README de una librería de
  terceros (P), el modo GPIO es el de fábrica con los puentes M1 y M2 abiertos; M1 cerrado = UART, M2 cerrado = I2C.
  Funcionó con pulso de disparo de 10 µs (M). Los dos laterales anduvieron con el mismo firmware.
- **Nivel del ECHO.** Con 5 V es 5 V y el GPIO de la ESP32 admite hasta 3,6 V: divisor 2,2 k / 3,3 k. Medido: 5V0 = 4,69 V
  y punto medio = 2,81 V (C: 4,69 × 3,3 / 5,5 = 2,81 V) (M).
- **Para navegación.** Un salto aislado por encima de 1000 mm justo antes de perder el eco es típico: usar **filtro de
  mediana de 3 a 5 muestras** y tratar "sin eco" como "libre hasta el alcance máximo", no como dato faltante.

## 2. Encoder óptico (módulo MH-Sensor-Series, ranura tipo FC-03)

- **Pines** (serigrafía de atrás, izquierda a derecha): VCC, GND, D0, A0. Dos LED: alimentación (siempre encendido) y
  conmutación (encendido con D0 = 0) (M).
- **D0** (comparador): 3,216 V / 0,088 V alimentado con 3V3 (M). Compatible con la ESP32: VIH ≥ 0,75·VDD = 2,475 V (propuesta
  v0.2) y VIL ≤ 0,25·VDD = 0,825 V (verificar en la hoja de datos). Como da niveles limpios, la placa trae su propio
  pull-up y no hace falta uno externo en GPIO34 a 39 (que no tienen interno).
- **A0** (analógica): 0,117 V con la ranura libre y 3,2 V tapada (M). D0 = 1 corresponde, con alta probabilidad, a ranura
  tapada (deducción a partir de A0; confirmar).
- **Conteo.** El PCNT cuenta **los dos flancos**: 20 ranuras = **40 flancos por vuelta**. Un solo canal **no distingue
  el sentido**: girar hacia atrás también suma. El firmware debe tomar el signo de la orden de dirección.
- **Medido a mano:** 5 vueltas del disco = 42, 45, 40, 42, 42 flancos (media 42,2; el exceso es el error de frenar a ojo).
  4 vueltas de rueda = 75, 43, 18, 30 (no concluyente: pausas, retrocesos y rueda en inercia). Girar a mano de corrido
  no es posible con una reductora 1:48.
- **Disco en la rueda.** El kit tiene motores de doble eje y el disco va en el eje interno, el mismo que mueve la rueda
  (propuesta v0.2, P). Consistente con ~40 flancos por vuelta de rueda.
- **Rueda:** diámetro 67 mm (M); π × 67 = 210,5 mm; **5,262 mm por flanco** (C). Con el hilo se midió 215 mm
  (5,375 mm por flanco, +2,1 %). Se usa 5,262 mm hasta calibrar con 1 m real (REAL-04).
  (Los 5,105 mm de la propuesta eran para una rueda de 65 mm.)
- **Aviso inofensivo:** `gpio_pullup_en ... input-only pad has no internal PU` lo imprime el driver del PCNT al intentar
  activar un pull-up que GPIO34 no tiene; el conteo funciona igual (M).
- **Estimar velocidad.** A velocidad máxima hay ~115 flancos/s (propuesta v0.2, C). En una ventana de 10 ms entra ~1
  flanco; en 50 ms, 5,75 (±17 %); en 100 ms, 11,5 (±8,7 %). Para el lazo de velocidad conviene una **ventana de ~100 ms**.

## 3. Motor TT y por qué el arranque es el problema

- **Datos.** TT amarillo, reducción 1:48; la propuesta cita 3–6 V, ≥ 90 rpm a 3 V y ≥ 200 rpm a 6 V (V, SunFounder). La
  publicación del vendedor dice 3–12 V: **usar ≤ 6 V**.
- **Un motor de continua parado se comporta como una resistencia de unos 3–4 Ω** (3 V / 1,1 A = 2,73 Ω y 6 V / 1,5 A = 4,0 Ω
  en las curvas de Adafruit, según la propuesta). Al girar genera una tensión que se opone a la fuente (**fuerza
  contraelectromotriz**) y la corriente cae a 0,15–0,30 A en crucero.
- **Arranque sin rampa:** pasar de 0 a 100 % de duty de golpe. Con el motor parado la corriente es solo (5 V − caída del
  L9110S) / R = 0,74–1,23 A **por motor**. Cuatro motores: 2,96–4,92 A, más 0,455–0,705 A de la electrónica = **3,4–5,6 A** (C).
- **Por qué importa.** Todo cuelga de **un único elevador de 5 V** que entrega 2,2 A nominales y 3 A como máximo. Si se le
  pide más, se protege o su salida cae. Al caer el 5 V, la ESP32 se **reinicia por brownout** y con ella la ESP32-CAM y los
  servos, porque comparten el riel.
- **Rampa.** Subir el duty de a poco. Propuesta de FW-02: salto inicial ≤ 0,45 (0,45 × 3,1 V / 3,75 Ω = 0,37 A por motor
  parado) y luego +0,03 cada 10 ms. A medida que el motor acelera crece la fuerza contraelectromotriz y baja la corriente.
  Con rampa, el arranque de los 4 pide 2,1–2,9 A (C: 4 × (0,4…0,6) + 0,455): al límite del módulo, pero sin reinicio.
  **Para revisar en FW-02:** "0 a 100 % en unos 330 ms" vale solo si la rampa parte de 0. Con el salto inicial a 0,45
  son (1 − 0,45) / 0,03 ≈ 18 ciclos = ~183 ms.
- **Forma de dar PWM al L9110S.** Adelante: IA = PWM, IB = 0. Atrás: IA = 0, IB = PWM. Parar: 0 / 0. La señal fija queda
  siempre en 0 (propuesta, sección 3.2).
- **Mapa A o B-rueda.** Mapa A: un PWM por lado (2 motores en paralelo, 4 pines). B-rueda: un PWM por rueda (8 pines,
  "sin margen"). Se decide con las curvas velocidad–duty de cada motor (ver FW-06).

## 4. Baterías 18650 y el módulo de baterías

- **Celdas del equipo:** XTAR INR 18650H, 2600 mAh, 3,7 V nominal, **20 A continuos y 30 A de pico** según el vendedor,
  **sin circuito de protección** (P). Qué quiere decir "20 A": la celda puede entregar hasta 20 A sin dañarse. Los
  2,45–2,85 A por celda con 3 A de salida de 5 V dejan ~7 veces de margen (C: 20 / 2,85).
- **Tensiones de una celda de iones de litio:** carga hasta 4,20 ± 0,05 V y corte de descarga entre 2,5 y 2,75 V según
  fichas de otras XTAR de 2600 mAh (P; no se encontró la hoja de la 18650H sin protección). Fuera de ese rango hay daño
  o riesgo.
- **Dos celdas en paralelo (1S2P) deben tener la misma tensión, con menos de 0,05 V de diferencia** (propuesta v0.2). Si no,
  se igualan por sí solas con una corriente que solo limita su resistencia interna.
- **Uso del módulo** (P, descripción del vendedor): un clic enciende y doble clic apaga; viene encendido por defecto; **apagar
  antes de sacar las celdas**, porque si no se activa la protección y "no vuelve a prender" (se recupera colocando
  las celdas y conectando el cargador). Salida 5 V / 2,2 A nominal y 3 A máximo; 3 V / 1 A; entrada micro-USB o Type-C
  hasta 6,5 V; carga hasta 1,8 A. Nunca cargar con el robot andando.
- **Carga:** cualquier cargador de 5 V de 1–2 A sirve (el de celular, en 5 V) o el USB de la PC (más lento).
  Con 0,6–0,8 A, dos celdas de 2,5 Ah tardan 6,3–8,3 h en la fase de corriente constante (C: 5,0 Ah / 0,8 A y 5,0 Ah / 0,6 A).
  Lo ideal es un cargador individual de 18650.
- **Incidente 2026-10-24 (M):** se midieron celdas en 4,3 V y 2,4 V (4,3 V excede el máximo y 2,4 V está bajo el mínimo);
  después, 3,55 V y 4,05 V (0,50 V de diferencia). No se juntaron en el módulo. Regla del equipo: **medir las dos
  celdas con el tester, fuera del módulo, antes de colocarlas**.
- **Alimentar la ESP32:** por el pin 5V desde la salida de 5 V del módulo y **nunca a la vez con el USB**. No usar la salida de 3 V.

## 5. Red de competencia (dato del equipo, 2026-10-24)

La red de la competencia es la de **un módem que la organización destina en exclusiva y sin acceso a internet**.
Consecuencias para `docs/decisiones/0002-topologia-de-red.md`:

- Se resuelve la duda abierta del ADR: la red **no** la crea la ESP32-CAM. La PC, la ESP32 de control y la ESP32-CAM se
  conectan como **estaciones** al módem. El modo "punto de acceso de la ESP32-CAM" deja de ser la configuración de
  competencia.
- Un solo firmware con parámetros sin recompilar (SSID, clave, IP del agente) sigue siendo correcto. Cambia el segundo
  perfil: en vez de "AP de la CAM", **"módem de la organización"**.
- La PC **no tendrá internet**: todo debe arrancar sin conexión (imagen del agente micro-ROS ya descargada, dependencias
  instaladas).
- Hay que acordar cómo se asignan las IP (DHCP o IP fija de la PC) para que el firmware sepa dónde está el agente.
- Tareas afectadas: **CAM-03 (#20)**, prueba de la red por AP, y **CAM-05 (#22)**, plan B por UART. Conviene avisar a Martín.
- Pendiente: la confirmación por escrito de la organización (INF-01, #3).

## 6. Flujo de trabajo del banco

- ESP-IDF 5.5 y ROS 2 van en **terminales separadas**: el `pytest` de ESP-IDF choca con el de ROS en `colcon test`.
- `idf.py` solo existe después de `. ~/esp/esp-idf-v5.5/export.sh`; hay que repetirlo en cada terminal nueva.
- **Flashear con el 5V0 del sensor desenchufado.** Con el circuito conectado el flash se cortaba a mitad de camino (M, causa
  no determinada); sin el circuito terminó bien.
- Un solo programa por puerto serie: si el `monitor` está abierto, el graficador no recibe datos.
- El PCNT cuenta bien a mano, pero los números de vueltas manuales sirven solo para el orden de magnitud.
- En la PC de desarrollo `docker` es en realidad **podman** (`Emulate Docker CLI using podman`) y el agente micro-ROS no
  está instalado como paquete de ROS; se usa la imagen `microros/micro-ros-agent:jazzy`.
