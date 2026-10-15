# Hardware y electrónica v0.2

> Propuesta del 2026-10-14 para la placa de movimiento (ESP32 NodeMCU de 38 pines, ESP32-WROOM-32), el driver L9110S y la alimentación con el módulo de dos celdas 18650. Reemplaza el pin map v0.1, que suponía drivers TB6612FNG y una batería 2S con reguladores buck. **No soldar nada** hasta cerrar las preguntas a la organización y las mediciones de la sección 8.

Estados usados en las tablas:

- **V**: verificado en esta tarea, leyendo la fuente primaria (datasheet, documentación oficial de Espressif, reglamento o BOM del repo).
- **P**: parcial. Sale de extractos de búsqueda, fichas de vendedores o del datasheet de otro fabricante del mismo chip.
- **C**: calculado acá; la fórmula está al lado.
- **M**: sin verificar. Hay que medirlo en el banco.

## 1. Resumen

1. **El límite que manda es el elevador de 5 V del módulo de baterías, no el L9110S.** Los vendedores lo anuncian como 5 V / 3 A, y algunos como 2,2 A continuos con 3 A de pico. Cuatro motores TT arrancando o trabados piden entre 3,0 y 4,9 A. Si el 5 V cae, se reinician juntas la ESP32, la ESP32-CAM y los servos, porque todo cuelga del mismo elevador.
2. Por eso el firmware **debe** tener rampas de PWM, corte por motor trabado y servos de a uno, y el brazo solo se mueve con el robot quieto. Sin eso, el robot se va a reiniciar en los arranques.
3. **Un solo pin map (mapa A)** sirve para el módulo de 2 canales y también para un driver de 4 canales usado "por lado" (las entradas de las dos ruedas de cada lado puenteadas). Solo hace falta otro mapa (B-rueda, 8 pines) si se quiere controlar cada rueda por separado.
4. Las dos entradas de cada canal del L9110S (IA e IB) van a pines con PWM (LEDC). Adelante: IA = PWM, IB = 0. Atrás: IA = 0, IB = PWM. Parar: las dos en 0.
5. **Preguntar por escrito a la organización antes de soldar** si se permiten los pasivos (divisores de los ecos, capacitores, pull-downs), una segunda fuente y otro driver. El reglamento descalifica por "alteración del hardware del robot estándar" (Art. 10 y tabla de penalidades).

## 2. Datos verificados

| Componente | Dato | Valor | Estado | Fuente |
|---|---|---|---|---|
| L9110 (chip) | Tensión de alimentación | 2,5–12 V (típ. 6 V) | V | [1] |
| | Corriente continua por canal | 750 / 800 / 850 mA (mín. / típ. / máx.) | V | [1] |
| | Corriente de pico | 1500 mA típ., 2000 mA máx. | V | [1] |
| | Salida en alto / en bajo (Vcc 9 V, 750 mA) | 7,50–7,70 V / 0,35–0,55 V | V | [1] |
| | Caída total del puente a 750 mA | 1,65–2,05 V (típ. 1,85 V) = (9 − V_alto) + V_bajo | C | [1] |
| | Entrada alta / baja | 2,5–9 V / 0–0,7 V | V | [1] |
| | Protecciones | Diodos de clamp internos; 0–80 °C. El datasheet **no** menciona protección térmica ni por corriente | V | [1] |
| | Frecuencia de PWM | El datasheet no da tiempos de conmutación ni frecuencia máxima | V | [1] |
| L9110S de otros fabricantes | Rangos | HGSEMI 2019: 2,3–12 V, 800 mA por canal, 1,5 A de pico. HGSEMI 2015: 2,2–6,5 V (máx. absoluto 8 V). Haixin: caída 0,75–0,85 V a 200 mA y 1,00–1,15 V a 500 mA. Diagrama de bloques con resistencias de 5 k y 10 k en las entradas (no se sabe si a VCC, a GND o en serie) | P | [2] |
| Módulo L9110S de 2 canales (HG7881) | Contenido y pines | 2 chips L9110S (un puente H cada uno), 800 mA por canal, 2,5–12 V. Pines B-IA, B-IB, GND, VCC, A-IA, A-IB. Borneras Motor A y Motor B. 29,2 × 23 mm, LED de encendido | V | [3][4][5] |
| Placa "HG7881 4-Channel" | Contenido y pines | 4 × L9110S. Entradas A1, A2, B1, B2, C1, C2, D1, D2. Salidas MOTORA–MOTORD. Alimentación "+" / "−", 2,5–12 V. 0,8 A por salida (dato del vendedor) | P | [6] |
| Motor TT amarillo | Datos del fabricante del kit | 1:48, 3–6 V, 150 mA ± 10 % en vacío, ≥ 90 rpm a 3 V, ≥ 200 rpm a 6 V, par de bloqueo 0,4 kg·cm (3 V) y 0,8 kg·cm (6 V) | V | [7] |
| | Corriente de bloqueo | 1,1 A (3 V), 1,2 A (4,5 V), 1,5 A (6 V) | P | [8] |
| Kit 2WD "smart car" | Contenido típico | 2 motores 1:48 de doble eje, 2 ruedas de 65–66 mm (algunos kits, 70 mm), 2 discos de 20 ranuras en el eje interno, rueda loca, portapilas | P | [9] |
| BOM de la organización | Cantidades | CHASIS 4WD × 2, MOTOR-RUEDA 4WD × 4, SERVOS SG90 × 5 | V | [24] |
| Módulo de baterías (tipo "18650 Battery Shield V8") | Configuración | 2 celdas **en paralelo** (1S2P), 3,2–4,2 V | P | [10] |
| | Salida de 5 V | "3 A". Otras fichas: 2,2 A nominal y 3 A máx., o 3 A nominal y 3,6 A máx. | P | [10] |
| | Resto | Salida 3 V / 1 A; carga por micro-USB a 600–800 mA; IC de protección (sobrecorriente, sobre y subtensión); "95 %" de eficiencia (vendedor). Algunas versiones tienen interruptor NORMAL/HOLD: en NORMAL apaga la salida a los ~10 s si se consumen menos de 30 mA. Una reseña reporta un corte de ~0,5 s al enchufar el cargador | P | [10] |
| | Publicación MLA2050868757 | No se pudo leer (sitio bloqueado). No está confirmado que sea este modelo, ni si expone la tensión cruda de las celdas | M | [11] |
| Celda XTAR 18650 "20 A" | Capacidad y protección | No hay hoja oficial accesible. Un revendedor lista "XTAR 18650 2500 mAh, 20 A, protegida". Las XTAR 2600 mAh protegidas figuran con 4,5–5,2 A máx. | P / M | [12] |
| ESP32-DevKitC V4 (38 pines) | Cabeceras | J1: 3V3, EN, 36, 39, 34, 35, 32, 33, 25, 26, 27, 14, 12, GND, 13, 9, 10, 11, 5V. J3: GND, 23, 22, 1, 3, 21, GND, 19, 18, 5, 17, 16, 4, 0, 2, 15, 8, 7, 6 | V | [13] |
| | Restricciones | GPIO6–11 son la flash: no usar. GPIO16 y 17 solo quedan libres en placas WROOM o SOLO | V | [13] |
| | Alimentación | Por USB, por el pin 5V **o** por el pin 3V3: **una sola a la vez**; si no, se puede dañar la placa o la fuente | V | [13] |
| ESP32: GPIO | Restricciones | Strapping: 0, 2, 5, 12, 15. 34–39 solo entrada, sin pull-up ni pull-down. ADC2 no funciona con Wi-Fi. 12–15 son JTAG. 1/3 son la UART de programación | V | [14] |
| | GPIO12 (MTDI) | Con flash de 3,3 V y regulador interno (WROOM) debe estar en **bajo** al reset; tiene un pull-down interno débil | V | [15] |
| | GPIO36 y 39 | Bajan ~80 ns cuando se encienden SARADC1, SARADC2, AMP o HALL | V | [16] |
| ESP32: periféricos | PCNT | 8 unidades de 2 canales; filtro de glitches de hasta 1023 ciclos APB = 12,79 µs | V + C | [17] |
| | MCPWM | 2 grupos, cada uno con 1 timer de captura de 3 canales. La captura **no** tiene filtro de glitches (solo prescaler, flancos, pull-up/down e invertir) | V | [17][18] |
| | LEDC | Modos de alta y baja velocidad, 8 canales y 4 timers por modo, timers de 20 bits | V | [17] |
| | ADC1 a 11 dB | Mide de 150 a 3100 mV | V | [19] |
| Arduino-ESP32 | LEDC en el core 2.x | Canal n → grupo n/8 y timer (n/2) % 4: los canales 2k y 2k+1 comparten timer | V | [20] |
| | LEDC en el core 3.x | `ledcAttachChannel(pin, frec, bits, canal)` reusa un timer del mismo grupo con igual frecuencia y resolución, o toma uno libre. Solo conoce los timers asignados por la API de Arduino | V | [20] |
| esp32-camera | Reloj XCLK | LEDC en `LEDC_LOW_SPEED_MODE`. Los ejemplos usan `LEDC_TIMER_0` y `LEDC_CHANNEL_0` a 20 MHz | V | [21] |
| ESP-IDF Wi-Fi | Potencia de transmisión | `esp_wifi_set_max_tx_power(p)`: p en pasos de 0,25 dBm, de 8 a 84 (2–20 dBm), llamada después de `esp_wifi_start` | V | [22] |
| Espressif | Fuente | Al menos 500 mA y al menos 10 µF en la entrada de alimentación | V | [23] |
| Consumos de terceros (investigación previa, no releídos hoy) | | ESP32: TX ~240 mA, RX ~95–100 mA. ESP32-CAM: ~180 mA con el flash apagado, ~310 mA con el flash al máximo. SG90: ~10 mA en reposo, 360–750 mA trabado. HC-SR04: 15 mA. FC-03: ~15 mA | P | [8] |
| Reglamento | Art. 8 | "4 ruedas traccionadas (4WD) con motores DC independientes — control diferencial de velocidad". "Batería 18650 recargables — recarga entre rondas garantizada" | V | [25] |
| | Art. 10 y penalidades | Prohibido: "Alteración del hardware del robot estándar (motores, sensores, estructura mecánica)". Una modificación no autorizada verificada en la inspección se sanciona con descalificación definitiva | V | [25] |

## 3. Cómo manejar el L9110S

### 3.1 Tabla de verdad

| IA | IB | OA | OB | Datasheet [1] | SunFounder [3] | Banana Robotics [5] |
|---|---|---|---|---|---|---|
| H | L | H | L | adelante | gira en sentido horario | adelante |
| L | H | L | H | atrás | gira en sentido antihorario | atrás |
| L | L | L | L | parado: las dos salidas a GND (freno) | "Brake" | "Soft stop (coast)" |
| H | H | L | L | parado: igual que L/L | "Stop" | "Hard stop (brake)" |

Las fuentes no coinciden en qué estado frena y cuál deja la rueda libre. Según el datasheet, los dos frenan. Se mide en el banco (sección 8, punto 3).

### 3.2 Cómo dar PWM

- **Propuesta: dos PWM por canal.** Adelante: IA = PWM con duty d e IB = 0. Atrás: IA = 0 e IB = PWM. Parar: 0/0. Durante la parte apagada del ciclo el estado es siempre 0/0, así que la curva velocidad–duty es igual en los dos sentidos, frene o no frene 0/0. Sobran canales LEDC: hay 16 y se usan 4 u 8.
- Alternativa que ahorra canales: IA = PWM e IB = dirección, invirtiendo la duty (1 − d) cuando IB = 1, como en el ejemplo de Banana Robotics. Si 0/0 y 1/1 no frenan igual, la marcha atrás responde distinto. No hace falta usarla.
- Al cambiar de sentido, pasar por 0/0 durante al menos un ciclo de control (10 ms) y respetar la rampa.
- **Frecuencia.** El datasheet no da tiempos de conmutación, y la salida del chip es bipolar. Dejar `MOTOR_PWM_HZ` como parámetro, arrancar en **5 kHz** y probar 1, 5 y 20 kHz en el banco. Pasar a 20 kHz (que no se escucha) solo si el chip no calienta más y no sube la duty mínima de arranque.
- Resolución de 10 bits en todos los casos. El máximo es 13 bits a 5 kHz y 11 bits a 20 kHz (C: bits = ⌊log2(80 MHz / f)⌋).
- Las entradas aceptan 3,3 V: el umbral alto mínimo es 2,5 V [1], y los módulos se venden como compatibles con microcontroladores de 3,3 V [3].

### 3.3 Dos motores por canal o uno por canal

| Situación (corriente por canal) | 2 canales: 2 motores en paralelo | 4 canales: 1 motor | Límite del L9110 |
|---|---|---|---|
| Crucero, 0,25 A por motor (estimado, M) | 0,5 A | 0,25 A | 0,8 A continuos |
| Giro en el lugar, 0,5–0,8 A por motor (estimado, M) | 1,0–1,6 A | 0,5–0,8 A | 0,8 A continuos |
| Arranque o motor trabado con duty 100 % (C) | 1,33–2,20 A | 0,74–1,23 A | 1,5 A típ. / 2,0 A máx. de pico |
| Tensión en bornes del motor en crucero (C, caídas de Haixin) | 3,85–4,0 V | 4,15–4,25 V | — |

Cálculo del bloqueo (C): I = (5 V − caída) / R.

- R sale de los puntos de bloqueo de Adafruit: 3 V / 1,1 A = 2,73 Ω; 4,5 V / 1,2 A = 3,75 Ω; 6 V / 1,5 A = 4,0 Ω.
- Un motor por canal (caída de 1,65–2,05 V): de (5 − 2,05) / 4,0 = 0,74 A a (5 − 1,65) / 2,73 = 1,23 A. El valor central es (5 − 1,85) / 3,75 = 0,84 A.
- Dos motores en paralelo (R/2, con una caída supuesta de 2,0–2,5 V): de 1,33 A a 2,20 A.
- Cuatro motores: 2,95–4,91 A.

Conclusión:

- Con el módulo de 2 canales, los arranques y los giros en el lugar llevan al chip por encima de su corriente continua y cerca del pico. Sirve solo con rampas, un tope de duty en los giros, corte por motor trabado y un control de la temperatura del chip después de una corrida de 2 minutos.
- Con 4 canales (la placa HG7881 de 4 canales o dos módulos de 2 canales), cada chip lleva la mitad de la corriente. Pero el límite del módulo de baterías sigue siendo el mismo.
- El Art. 8 habla de "motores DC independientes". Antes de cambiar el driver hay que preguntar a la organización cuál es el estándar.

## 4. Módulo de baterías

Lo más probable (P) es que sea un "18650 Battery Shield V8" o similar: dos celdas en paralelo, un elevador a 5 V, un regulador de 3 V, protección y un cargador por micro-USB. Consecuencias:

- Las celdas están entre 3,0 y 4,2 V, y todo lo que es 5 V sale de **un único elevador**. No existe una rama de motores separada de la lógica.
- Reglas de uso:
  1. Colocar las dos celdas juntas y con la misma tensión: diferencia menor a 0,05 V, medida con tester. Al estar en paralelo, una llena y una vacía se igualan con mucha corriente.
  2. No enchufar el cargador con el robot andando. Una reseña reporta un corte de ~0,5 s, que reinicia todo.
  3. Cargar con un cargador externo de 18650 si se puede. Por micro-USB, a 0,6–0,8 A, dos celdas de 2,5 Ah tardan 6,3–8,3 h solo en la fase de corriente constante (C: 5,0 Ah / 0,8 A y 5,0 Ah / 0,6 A). Tener un segundo par cargado. La organización garantiza la recarga entre rondas (Art. 8).
  4. Si el módulo tiene interruptor NORMAL/HOLD, dejarlo en HOLD.
  5. No alimentar la ESP32 desde la salida de 3 V, porque queda en el borde del rango del módulo WROOM (M). Alimentar la DevKit por su pin 5V.
  6. **Nunca** conectar a la vez el USB de la DevKit y el 5 V de la batería: Espressif pide una sola fuente [13]. Para programar por USB, desenchufar el conector 5V_ESP y dejar GND conectado.
  7. Las celdas protegidas son más largas. Verificar que entren bien y que los resortes aprieten, y sujetarlas con cinta o un precinto para que una vibración no corte la alimentación.

## 5. Pin map v0.2

### 5.1 GPIO de la DevKit de 38 pines

| Grupo | GPIO | Uso en esta propuesta |
|---|---|---|
| Solo entrada, sin pull-up ni pull-down | 34, 35, 36, 39 | Encoders. El filtro del PCNT tapa el glitch de 36 y 39 |
| Libres sin restricciones | 4, 13, 16*, 17*, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33 | Motores, servos, ecos y ADC1 (32, 33) |
| JTAG, sin strapping | 14 | Trigger |
| Strapping | 0 (modo de arranque), 2 (en bajo para programar), 5, 12 (en bajo al reset), 15 | 0: botón BOOT de la placa. 2: LED. 5 y 15: triggers. 12: sin conectar |
| UART0 | 1 (TX), 3 (RX) | Programación y log: no usar |
| Flash | 6–11 | Nunca |

\* GPIO16 y 17 solo en placas WROOM; el módulo del equipo es un ESP32-WROOM-32. En total hay 26 GPIO accesibles.

### 5.2 Mapa A (recomendado): módulo de 2 canales, o de 4 canales "por lado"

| Señal | GPIO | Periférico | Notas |
|---|---|---|---|
| M_IZQ_IA | 25 | LEDC alta velocidad, canal 0 | Lado izquierdo, adelante (PWM) |
| M_IZQ_IB | 26 | LEDC alta velocidad, canal 1 | Lado izquierdo, atrás (PWM) |
| M_DER_IA | 27 | LEDC alta velocidad, canal 2 | Lado derecho, adelante (PWM) |
| M_DER_IB | 16 | LEDC alta velocidad, canal 3 | Lado derecho, atrás (PWM) |
| SERVO_BASE | 18 | LEDC baja velocidad, canal 0 | 50 Hz |
| SERVO_HOMBRO | 19 | LEDC baja velocidad, canal 1 | 50 Hz |
| SERVO_CODO | 21 | LEDC baja velocidad, canal 2 | 50 Hz |
| SERVO_PINZA | 22 | LEDC baja velocidad, canal 3 | 50 Hz |
| SERVO_PAN | 23 | LEDC baja velocidad, canal 4 | Pan de la cámara, en la misma placa que el brazo |
| TRIG_F | 5 | Salida | Strapping: un pulso al arrancar solo dispara una medición |
| TRIG_L | 14 | Salida | JTAG |
| TRIG_R | 15 | Salida | Strapping: si queda en bajo al reset solo se silencia el log de arranque |
| ECHO_F | 13 | Captura MCPWM0, canal 0 | Divisor 2,2 k / 3,3 k |
| ECHO_L | 4 | Captura MCPWM0, canal 1 | Divisor 2,2 k / 3,3 k |
| ECHO_R | 17 | Captura MCPWM0, canal 2 | Divisor 2,2 k / 3,3 k |
| ENC_FL | 34 | PCNT, unidad 0 | Delantera izquierda |
| ENC_RL | 35 | PCNT, unidad 1 | Trasera izquierda |
| ENC_FR | 36 (VP) | PCNT, unidad 2 | Delantera derecha |
| ENC_RR | 39 (VN) | PCNT, unidad 3 | Trasera derecha |
| VBAT_SENSE | 32 | ADC1 canal 4 | Opcional: tensión de celda, divisor 100 k / 100 k |
| V5_SENSE | 33 | ADC1 canal 5 | Opcional: riel de 5 V, divisor 100 k (arriba) / 68 k (abajo) |
| LED_ESTADO | 2 | Salida | LED de la placa, si tiene |
| BOTON_INICIO | 0 | Entrada | Botón BOOT de la placa; leerlo solo después del arranque |
| (libre) | 12 | — | Sin conectar: tiene que estar en bajo al reset |
| (reservados) | 1, 3 | UART0 | Programación y log |

Uso: 23 de 26 GPIO (13 salidas y 10 entradas, contando el botón). Quedan libres el 12 (no tocar) y la UART0.

Cableado de los motores:

- **Módulo de 2 canales:** la bornera Motor A va a la rueda delantera izquierda (FL) y a la trasera izquierda (RL) en paralelo; Motor B, a FR y RR. Conexiones: A-IA = GPIO25, A-IB = GPIO26, B-IA = GPIO27, B-IB = GPIO16.
- **Placa de 4 canales, por lado:** MOTORA = FL, MOTORB = RL, MOTORC = FR, MOTORD = RR. GPIO25 → A1 y B1; GPIO26 → A2 y B2; GPIO27 → C1 y D1; GPIO16 → C2 y D2. Cada pin de la ESP32 maneja dos entradas.
- Si una rueda gira al revés, dar vuelta sus dos cables en la bornera. No se corrige por software, porque las ruedas de un lado comparten las señales.

**Por qué los encoders van a 34–39 y los ecos no.** El PCNT tiene un filtro de glitches que absorbe el pulso de 80 ns de GPIO36 y 39. La captura MCPWM no tiene filtro, así que un eco en esos pines podría registrar un flanco falso y dar una distancia errónea. Los ecos llevan divisor de todos modos, y su resistencia de 3,3 k a GND fija el nivel en cualquier pin. Encoders y ultrasonidos se miden por hardware (PCNT y MCPWM), sin interrupciones de GPIO.

### 5.3 Mapa B-rueda: 4 canales, cada rueda por separado (8 pines)

Solo si se quiere un lazo de velocidad por rueda. Cambios respecto del mapa A:

| Señal | GPIO | Notas |
|---|---|---|
| M_FL_IA / M_FL_IB | 25 / 26 | LEDC alta velocidad, canales 0 y 1 |
| M_RL_IA / M_RL_IB | 27 / 16 | Canales 2 y 3 |
| M_FR_IA / M_FR_IB | 17 / 33 | Canales 4 y 5 |
| M_RR_IA / M_RR_IB | 23 / 2 | Canales 6 y 7. GPIO2: si las entradas del driver tienen pull-up, puede impedir programar por USB (M). El LED de la placa parpadea con el PWM |
| ECHO_R | 32 | Pasa del 17 al 32, con divisor |
| SERVO_PAN | ESP32-CAM, GPIO13 | Solo la señal. El servo se alimenta del bus de servos, con GND común |
| VBAT_SENSE, V5_SENSE, LED_ESTADO | — | No quedan pines |

Los servos de base, hombro, codo y pinza, los triggers, ECHO_F (13), ECHO_L (4), los encoders y el botón quedan igual que en el mapa A. Uso: 23 de 26 GPIO (15 salidas y 8 entradas); libres el 12 y la UART0. Este mapa no deja margen para ninguna sorpresa. Por eso se recomienda el mapa A.

### 5.4 Periféricos

**LEDC**

| Placa | Uso | Modo y timer (ESP-IDF) | Canales en Arduino 2.x | Frecuencia / resolución |
|---|---|---|---|---|
| ESP32 de movimiento | Motores (mapa A) | Alta velocidad, timer 0 | 0–3 (timers 0 y 1) | 5 kHz (probar 20 kHz) / 10 bits |
| ESP32 de movimiento | Motores (B-rueda) | Alta velocidad, timer 0 | 0–7 (timers 0 a 3) | Igual |
| ESP32 de movimiento | Servos | Baja velocidad, timer 0 | 8–12 (mapa A) o 8–11 (B-rueda) | 50 Hz / 14 bits: 1,22 µs por cuenta; 500–2400 µs = 410–1966 cuentas |
| ESP32-CAM | XCLK de la cámara | Baja velocidad, timer 0, canal 0 | — | 20 MHz. No tocar |
| ESP32-CAM | Pan (solo B-rueda) | Alta velocidad, timer 1 | Canal 2 | 50 Hz / 14 bits |

- En el core 2.x, llamar a `ledcSetup` en **todos** los canales usados, porque los pares comparten timer (canales 0 y 1 → timer 0, canales 2 y 3 → timer 1, etc.). En el core 3.x, usar `ledcAttachChannel(pin, frec, bits, canal)` con números de canal fijos.
- En la ESP32-CAM, el pan va en el grupo de alta velocidad (canales 0–7). El core 3.x no sabe que la cámara ocupa el timer 0 de baja velocidad y podría reconfigurarlo, lo que corta el reloj de la cámara.

**PCNT.** Unidades 0–3, una por encoder, contando los dos flancos: 40 por vuelta con un disco de 20 ranuras, es decir 5,105 mm por flanco con rueda de 65 mm (C: π × 65 / 40). Filtro de 5–10 µs (el máximo es 12,79 µs). A la velocidad máxima estimada hay unos 115 flancos por segundo, o sea unos 8,7 ms entre flancos, así que el filtro no se come flancos reales. Acumulación activada.

**MCPWM0.** El timer de captura con sus 3 canales mide los ecos (flanco de subida y de bajada). Disparo de 10 µs, un sensor por vez: F → L → R cada 25 ms.

**ADC1** (atenuación de 11 dB, rango de 150 a 3100 mV; calibrar contra un tester):

| Señal | Divisor | Tensión en el pin (C) | Consumo |
|---|---|---|---|
| VBAT_SENSE | 100 k / 100 k | 4,2 V → 2,10 V; 3,0 V → 1,50 V | 21 µA |
| V5_SENSE | 100 k / 68 k | 5,5 V → 2,23 V; 5,0 V → 2,02 V | 30 µA |

VBAT_SENSE necesita llegar al borne + de las celdas. Hay que ver si el módulo lo expone (M). Poner 100 nF en cada pin del ADC.

**Ecos.** 2,2 k en serie y 3,3 k a GND dan 3,00 V con 5,00 V y 3,15 V con 5,25 V. La ESP32 necesita al menos 0,75 × 3,3 = 2,475 V para leer un alto y admite como máximo 3,6 V. Si los sensores fueran HC-SR04P (funcionan a 3,3 V), alimentarlos a 3,3 V y sacar el divisor.

**FC-03.** Alimentarlos desde el 3V3 de la DevKit, así la salida es de 3,3 V. Los pines 34–39 no tienen pull interno: si el módulo no trae pull-up en DO (M), poner 10 k a 3V3.

## 6. Alimentación

### 6.1 Distribución

```
[2 x XTAR 18650 en paralelo] -> [modulo: proteccion + elevador a 5 V]
                                      | 5 V (bornera, cables cortos)
         +----------------------------+------------------------------+
    rama MOTORES                 rama SERVOS                   rama LOGICA
    VCC del L9110S               5 x SG90                      pin 5V de la ESP32 (USB desconectado)
    470 uF + 100 nF              1000 uF                       5V de la ESP32-CAM: 470 uF + 100 nF en sus pines
    100 nF en cada motor                                       3 x HC-SR04 a 5 V
                                                               4 x FC-03 desde el 3V3 de la ESP32
GND en estrella en la bornera del modulo: la corriente de motores y servos no vuelve por los cables de la logica.
```

- Cables: 5 V principal y rama de motores de 0,5 mm² (20 AWG) o más; servos, 22 AWG; lógica, 22–24 AWG. Lo más cortos posible.
- Los capacitores van solo si la organización los permite. Verificar que el módulo arranque con unos 2000 µF conectados: algunos elevadores se cortan por la corriente de carga inicial (M).

### 6.2 Consumo por carga (riel de 5 V)

| Carga | Típico | Pico | Estado |
|---|---|---|---|
| ESP32 DevKit, Wi-Fi sin ahorro de energía | 0,12 A | 0,25 A (transmitiendo) | P |
| ESP32-CAM, video, flash apagado | 0,18 A | 0,30 A (estimado) | P / M |
| 3 × HC-SR04 | 0,045 A | 0,045 A | P |
| 4 × FC-03 (por el regulador de la DevKit) | 0,06 A | 0,06 A | P |
| 5 × SG90 en reposo | 0,05 A | — | P |
| 1 × SG90 en movimiento | 0,25 A (estimado) | 0,36–0,75 A trabado | P / M |
| 1 × motor TT en crucero | 0,15–0,30 A (en vacío, 0,15 A) | 0,74–1,23 A al arrancar o trabado | V / C / M |

Lógica: 0,405 A típicos y 0,655 A de pico (C: 0,12 + 0,18 + 0,045 + 0,06 y 0,25 + 0,30 + 0,045 + 0,06).

### 6.3 Escenarios frente al límite del módulo

| Escenario | Corriente en 5 V (C) | Fórmula | ¿Hasta 2,2 A? | ¿Hasta 3 A? |
|---|---|---|---|---|
| S1 Quieto, Wi-Fi y video | 0,46 A (pico 0,71 A) | 0,405 + 5 × 0,01 | Sí | Sí |
| S2 Crucero, 4 motores a 0,25 A | 1,46 A (pico 1,71 A) | S1 + 4 × 0,25 | Sí | Sí |
| S3 Arranque sin rampa | 3,4–5,6 A | 4 × (0,74…1,23) + 0,455…0,705 | **No** | **No: se reinicia** |
| S3b Arranque con rampa (0,4–0,6 A por motor) | 2,1–2,9 A | 4 × (0,4…0,6) + 0,455 | Al límite | Sí |
| S4 Brazo: 1 servo en movimiento, robot quieto | 0,86 A (pico 1,11 A) | 0,405 + 0,25 + 4 × 0,05 | Sí | Sí |
| S4b Los 5 servos a la vez | 1,66 A | 0,405 + 5 × 0,25 | Sí | Sí, pero nunca con los motores andando |
| S5 Crucero con el cubo en la pinza (0,10–0,36 A) | 1,56–1,82 A | S2 + 0,10…0,36 | Sí | Sí |
| S6 Giro en el lugar (0,5–0,8 A por motor, estimado) | 2,5–3,7 A | 4 × (0,5…0,8) + 0,455 | **No** | **Puede que no** |
| S7 Todo trabado | ~9,3 A | 4 × 1,23 + 5 × 0,75 + 0,655 | No | No: corta el módulo |

### 6.4 Corriente de las celdas y autonomía

Corriente de las celdas = P en 5 V / (η × V de celda), con η entre 0,85 y 0,80. Es un supuesto: el "95 %" del vendedor no está verificado.

| Salida de 5 V | Total de las celdas (η 0,85–0,80; 3,6–3,3 V) | Por celda |
|---|---|---|
| 1,46 A (crucero) | 2,4–2,8 A | 1,2–1,4 A |
| 2,2 A | 3,6–4,2 A | 1,8–2,1 A |
| 3,0 A | 4,9–5,7 A | 2,5–2,8 A |

Las celdas de 20 A tienen margen de sobra. Incluso unas protegidas de 5 A alcanzarían, aunque justas con 3 A de salida y las celdas bajas.

Autonomía = 2 × C × 3,6 V × 0,8 / (I_5V × 5 V / η):

| Celdas | 1,4 A promedio, η 0,85 | 1,8 A promedio, η 0,80 |
|---|---|---|
| 2 × 2,0 Ah | 84 min | 61 min |
| 2 × 2,5 Ah | 105 min | 77 min |
| 2 × 2,6 Ah | 109 min | 80 min |

Una ronda dura minutos y la recarga entre rondas está garantizada, así que lo que limita es el tiempo de desarrollo. Cerca del final de la descarga el elevador entrega menos corriente y los reinicios aparecen antes. Regla inicial: cambiar las celdas cuando la tensión en reposo baje de 3,7 V, y ajustar ese valor con la experiencia (M).

### 6.5 Mitigaciones en el firmware (sin hardware extra)

1. **Rampa de PWM.** Desde parado, saltar como máximo a duty 0,45, que con el motor quieto da unos 0,37 A (C: 0,45 × 3,1 V / 3,75 Ω). Después, subir como mucho 0,03 por ciclo de 10 ms: de 0 a 100 % en unos 330 ms. Además, limitar la aceleración desde la PC.
2. **Corte por motor trabado.** Si |duty| > 0,5 y los dos encoders de un lado cuentan 0 flancos en 300 ms, poner ese lado en 0 durante 1 s y publicar la falla.
3. **Tope de duty en los giros en el lugar.** Empezar en 0,6 hasta medir la corriente real.
4. **Servos de a uno**, con interpolación de 60–90 °/s. El brazo solo se mueve con el robot quieto; eso lo garantiza la máquina de estados de la misión. Calibrar el cierre de la pinza al cubo de 20 mm para que no quede trabada apretando.
5. **Arranque ordenado:** primero los pines de motor en 0 (la primera instrucción del programa); después el Wi-Fi; por último, los servos a su posición inicial de a uno, cada 300 ms.
6. **Bajar la potencia de transmisión de la ESP32 de movimiento.** Su punto de acceso, la ESP32-CAM, está en el mismo robot. `esp_wifi_set_max_tx_power()` acepta de 2 a 20 dBm; el ahorro de corriente hay que medirlo. La ESP32-CAM conserva su potencia porque tiene que llegar a la PC.
7. **Detector de brownout activado.** Al arrancar, publicar la versión de firmware y el motivo del último reinicio (`esp_reset_reason()`, que distingue un brownout). Así se sabe si un reinicio fue eléctrico.
8. **Watchdog de `/cmd_vel`:** si dejan de llegar órdenes, frenar.
9. El flash de la ESP32-CAM (GPIO4) siempre apagado.

### 6.6 Si la organización lo permite

1. **Un segundo módulo de baterías igual**, con 2 celdas más: uno alimenta el L9110S y los servos, y el otro la ESP32, la ESP32-CAM y los HC-SR04, con GND común. Es la mejor protección contra los reinicios.
2. Un driver de 4 canales (o dos módulos de 2 canales), si el estándar es el de 2 canales.
3. Los capacitores de 6.1, pull-downs de 10 k en las entradas del L9110S (si el módulo no define el nivel mientras la ESP32 está en reset) y los divisores de los ecos.

## 7. Riesgos

| Riesgo | Efecto | Mitigación |
|---|---|---|
| Caída del 5 V en arranques, giros, motores trabados o movimientos de servos | Se reinician juntas la ESP32, la ESP32-CAM y los servos en plena corrida | 6.5 completo; medir (sección 8, puntos 5 y 6); segunda fuente si la permiten |
| L9110S de 2 canales con 2 motores por canal | Por encima de 0,8 A continuos en giros y cerca del pico en arranques; sin protección térmica documentada | Rampa, tope en los giros, corte por bloqueo, control de temperatura; 4 canales si lo permiten |
| Hardware no aprobado (pasivos, segunda fuente, otro driver) | Descalificación definitiva | Consulta escrita antes de soldar |
| Módulo de baterías con datos solo de vendedor | Límite real de 2,2 A; corte al enchufar el cargador; celdas desparejas en paralelo | Medir el límite; no cargar con el robot andando; colocar las celdas apareadas |
| Entradas del L9110S con nivel desconocido en el reset | Las ruedas pueden dar un tirón al arrancar o al programar | Mapa A sin pines de strapping en los motores; medir; pull-downs si los permiten |
| GPIO2 como entrada de motor (solo B-rueda) | No se puede programar por USB si el driver tiene pull-up | Usar el mapa A |
| Discrepancia freno / rueda libre | Marcha atrás distinta con el esquema PWM + dirección | Esquema de 2 PWM |
| 20 kHz sin dato de conmutación | Calentamiento del chip | Empezar en 5 kHz y probar |
| Ecos de 5 V sin divisor | Pines de la ESP32 por encima de 3,6 V | Divisor 2,2 k / 3,3 k, o sensores HC-SR04P a 3,3 V |
| DO del FC-03 sin pull-up, o con rebotes | Cuentas falsas en 34–39 (sin pull interno) | Medir; pull-up de 10 k; filtro del PCNT |
| Pinza apretando el cubo trabada | Consumo continuo y servo caliente | Calibrar el ángulo de cierre; medir la corriente |
| Pan en la ESP32-CAM sobre el timer de la cámara | Se pierde el video | Pan en el grupo de alta velocidad, con canal fijo |
| USB y batería conectados a la vez a la DevKit | Daño a la placa o al puerto de la PC | Conector 5V_ESP separado |
| Velocidad máxima de ~0,37–0,41 m/s en carga (estimado) | Tiempos más largos | Suficiente para el laberinto; medir |
| Capacidad de las celdas desconocida | Autonomía de 61–109 min, con reinicios más probables con la batería baja | Leer la etiqueta; segundo par de celdas; cambiar por debajo de 3,7 V |

## 8. Decisiones a confirmar y qué medir

| # | Qué confirmar | Cómo medirlo | Qué decide |
|---|---|---|---|
| 1 | Canales del driver | Contar los chips SOP-8 y los pines de entrada: 6 pines (B-IA, B-IB, GND, VCC, A-IA, A-IB) son 2 canales; 10 (A1…D2, +, −) son 4. Leer la marca del chip | Mapa A con 2 motores por canal, mapa A por lado o B-rueda |
| 2 | Nivel de las entradas del driver sin señal | Driver con VCC = 5 V y sin ESP32: medir la tensión en cada entrada. Sin alimentación: medir la resistencia de cada entrada a VCC y a GND | Si hay pull-up a VCC, no usar 0, 2 ni 12 para motores (el mapa A ya lo cumple). Si flotan, pedir permiso para pull-downs |
| 3 | ¿0/0 y 1/1 frenan igual? | Con la rueda girando, aplicar 0/0 y después 1/1, y comparar cuánto tarda en parar | Confirma que el esquema de 2 PWM es simétrico |
| 4 | Frecuencia de PWM | A 1, 5 y 20 kHz: duty mínima para arrancar, temperatura del chip después de 2 min al 50 % (al tacto o con termómetro IR) y linealidad | `MOTOR_PWM_HZ` |
| 5 | Corrientes de motor | Tester en la escala de 10 A, en serie con el 5 V del driver, robot en el piso: crucero, giro en el lugar, arranque y rueda trabada (máximo 2 s) | Tope de duty en los giros; si alcanza con 2 canales |
| 6 | Caída del 5 V | En los pines de la ESP32-CAM, con osciloscopio o con V5_SENSE muestreado rápido: arranque de 4 motores con y sin rampa, y movimiento de servos. Objetivo: que no se reinicie y no baje de ~4,5 V (M) | Parámetros de la rampa; si hace falta una segunda fuente |
| 7 | Módulo de baterías | Leer los chips (elevador, cargador, protección); confirmar el paralelo (continuidad entre los + de los portapilas); ver si expone B+ y si tiene NORMAL/HOLD; cargarlo de a poco y ver hasta qué corriente mantiene 5 V | Límite real (2,2 o 3 A); si se puede tener VBAT_SENSE |
| 8 | Celdas | Leer la capacidad y si son protegidas (sin protección miden ~65 mm, con protección ~69 mm); medir la tensión de las dos antes de colocarlas | Autonomía; si entran firmes en el portapilas |
| 9 | Kit TT | Contar las ranuras del disco; confirmar que el motor es de doble eje; medir el avance en 10 vueltas de rueda | mm por flanco (5,105 mm con 20 ranuras y rueda de 65 mm) |
| 10 | FC-03 | A 3,3 V: polaridad de DO (ranura tapada o abierta), pull-up en la placa, flancos por vuelta a baja velocidad | Pull-up externo; valor del filtro del PCNT |
| 11 | HC-SR04 | Modelo (HC-SR04 o HC-SR04P); tensión en Trig sin conectar (¿tiene pull-up?); duración del eco sin obstáculo | Divisor sí o no; tiempo máximo de espera del eco |
| 12 | Servos | Corriente con la pinza cerrada sobre el cubo y con el brazo sosteniendo una pose | Ángulo de cierre de la pinza |
| 13 | Placa ESP32 | Que la serigrafía coincida con la DevKitC V4; módulo WROOM-32; si hay LED en GPIO2; si hay diodo entre el USB y el 5V | Uso de los pines 16 y 17 y del LED de estado |
| 14 | Organización (por escrito) | ¿Se permiten divisores, capacitores, pull-downs y una bornera? ¿Una segunda fuente o módulo de baterías? ¿Cuál es el driver estándar (2 o 4 canales; el Art. 8 dice "motores DC independientes")? ¿El servo de pan puede ir en la ESP32-CAM? | **Bloquea la soldadura** |

## Fuentes

1. Datasheet L9110 (ASIC), copias en GitHub: https://raw.githubusercontent.com/rydepier/L9110-Motor-Driver-and-Fan-motor-for-Arduino./master/Datasheet-L9110.pdf y https://github.com/ZdenekBrichacek/PCB-H_Bridge-L9110/blob/master/L9110.pdf
2. Datasheets L9110S de HGSEMI y otros fabricantes (solo extractos de búsqueda): https://atta.szlcsc.com/upload/public/pdf/source/20200820/C725348_2E2A3469F0E09406AB557FA4F5094EF0.pdf , https://wmsc.lcsc.com/wmsc/upload/file/pdf/v2/lcsc/2401021513_HGSEMI-L9110SM-TR_C725348.pdf , https://wmsc.lcsc.com/wmsc/upload/file/pdf/v2/lcsc/2109081930_HTCSEMI-L9110S_C2895059.pdf
3. SunFounder, módulo L9110: https://github.com/sunfounder/3in1-kit/blob/docs-r3-v2-en/docs/source/components/component_l9110_module.rst y código de velocidad: https://github.com/sunfounder/3in1-kit/blob/main/car_project/3.speed_up/3.speed_up.ino
4. RpiMotorLib: https://github.com/gavinlyonsrepo/RpiMotorLib/blob/master/extra/Documentation/L9110S_DC.md ; RobTillaart L9110: https://github.com/RobTillaart/L9110
5. Banana Robotics HG7881 (copias en gist): https://gist.github.com/sukinull/3cc498cf468a8453e8e6 y https://gist.github.com/rsrini7/9e27e09a51225a3af13bf8da3348a985
6. HG7881 de 4 canales: https://www.makerlab-electronics.com/products/hg7881-4-channel-dc-motor-driver-controller-board y https://robu.in/product/hg7881-4-channel-dc-2-4-10v-motor-driver-broad/
7. SunFounder, motor TT: https://github.com/sunfounder/3in1-kit/blob/docs-r3-v2-en/docs/source/components/component_tt_motor.rst
8. Investigación previa del equipo (12/10/2026) sobre Adafruit 3777 y consumos de ESP32, ESP32-CAM, SG90, HC-SR04 y FC-03 (fuentes en el borrador del plan anterior)
9. Kits 2WD: https://www.robotshop.com/es/products/kit-chasis-dagu-2wd-madera , https://sigmanortec.ro/en/smart-car-2wd-chassis-kit , https://www.fruugo.us/2wd-smart-robot-car-chassis-kit-with-speed-encoder-for-arduino-diy-stem-educational-projects/p-471540064
10. Battery Shield V8: https://shillehtek.com/blogs/shillehtek-product-manuals/2x18650-battery-shield-v8-5v-3a-esp32-arduino-manual , https://zbotic.in/?p=453818 , https://store.mectronica.it/en/various/2223-shield-power-bank-for-18650-lithium-batteries.html , https://aws.robu.in/wp-content/uploads/2021/07/NA694-2.docx
11. Publicación del equipo (no se pudo leer): https://www.mercadolibre.com.ar/modulo-baterias-bank-18650-para-3v1a-5v3a-2-power-micro-us/p/MLA2050868757
12. XTAR: https://imrbatteries.com/collections/xtar-18650-batteries , https://www.nkon.nl/en/xtar-18650-2600mah-protected.html , https://www.tme.eu/Document/2e42a914763d073309a83fd4f3c88f56/XTAR18650-2600PCM.pdf
13. ESP32-DevKitC V4: https://raw.githubusercontent.com/espressif/esp-idf/release/v4.4/docs/en/hw-reference/esp32/get-started-devkitc.rst
14. GPIO de la ESP32: https://raw.githubusercontent.com/espressif/esp-idf/release/v5.3/docs/en/api-reference/peripherals/gpio/esp32.inc
15. MTDI / GPIO12: https://raw.githubusercontent.com/espressif/esp-idf/release/v5.3/docs/en/api-reference/peripherals/sd_pullup_requirements.rst
16. ESP-FAQ, GPIO36/39: https://github.com/espressif/esp-faq/blob/master/docs/en/software-framework/peripherals/gpio.rst
17. soc_caps.h, pcnt_ll.h y ledc_types.h (ESP-IDF v5.3): https://raw.githubusercontent.com/espressif/esp-idf/release/v5.3/components/soc/esp32/include/soc/soc_caps.h , https://raw.githubusercontent.com/espressif/esp-idf/release/v5.3/components/hal/esp32/include/hal/pcnt_ll.h , https://raw.githubusercontent.com/espressif/esp-idf/release/v5.3/components/hal/include/hal/ledc_types.h
18. Captura MCPWM: https://raw.githubusercontent.com/espressif/esp-idf/release/v5.3/components/esp_driver_mcpwm/include/driver/mcpwm_cap.h
19. ADC en Arduino-ESP32: https://raw.githubusercontent.com/espressif/arduino-esp32/master/docs/en/api/adc.rst
20. LEDC en Arduino-ESP32: https://raw.githubusercontent.com/espressif/arduino-esp32/release/v2.x/cores/esp32/esp32-hal-ledc.c y https://raw.githubusercontent.com/espressif/arduino-esp32/master/cores/esp32/esp32-hal-ledc.c
21. esp32-camera: https://raw.githubusercontent.com/espressif/esp32-camera/master/target/xclk.c y https://raw.githubusercontent.com/espressif/arduino-esp32/master/libraries/ESP32/examples/Camera/CameraWebServer/CameraWebServer.ino
22. esp_wifi.h: https://raw.githubusercontent.com/espressif/esp-idf/release/v5.3/components/esp_wifi/include/esp_wifi.h
23. Guía de diseño de Espressif: https://raw.githubusercontent.com/espressif/esp-hardware-design-guidelines/master/docs/en/schematic-checklist.rst
24. BOM de la organización: hardware/bom/Lista_de_partes.ods (repo)
25. Reglamento: docs/reglamento/CAID_2026_Reglamento.pdf (repo)
