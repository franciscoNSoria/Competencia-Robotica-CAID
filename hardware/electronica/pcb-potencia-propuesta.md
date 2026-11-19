# PCB de potencia: propuesta de distribución (100 × 80 mm)

Issue: HW-01 (#95). Esquemas del rediseño: [hoja 1, potencia](img/esquema_potencia.png) (pack 2S2P → BMS → fusible → interruptor → 2 × XL4015 → cargas)
y [hoja 2, señales](img/esquema_senales.png).

Es un **dibujo de distribución**, no un archivo de fabricación (no hay KiCad ni Gerber todavía).
Los dos XL4015 miden 61 × 37 mm; para que entren con todo lo demás van **de pie** (37 de ancho, 61 de alto) y lado a lado.

## Lista de lo que lleva
| Ref. | Qué | Notas |
|---|---|---|
| J1 | Bornera enchufable 5,08 mm de 2 (batería: pads (+) y (−) del BMS) | |
| J2 | Bornera 2 contactos, **F1** | Un cable puente por ahora; después el portafusible (estándar, 7,5 A) |
| J3 | Bornera 2 contactos, interruptor | Interruptor de 10 A en CC |
| J4, J5 | Borneras 2 contactos, 5V_MOT hacia cada L9110S | |
| J6, J7 | Borneras 2 contactos, 5V_LOG hacia ESP32-S3 y ESP32-CAM | |
| 2 × hembras 2,54 | Para enchufar los XL4015 y poder sacarlos | Hay que medir dónde están sus pads (ver abajo) |
| C1 | 1000 µF 16 V, entrada VBAT | |
| C2, C3 | 1000 µF 16 V, reserva en 5V_MOT y 5V_LOG | Los capacitores que **protegen a cada ESP32** van pegados a su pin 5V, no acá |
| 4 × cabezal 3 pines 2,54 | Servos (señal, +5 V, GND) | La señal llega por SIG1–4 desde GPIO 1, 2, 41, 42 |
| 1 × cabezal 1×4 | SIG1–4 | Por cables dupont desde la ESP32 |
| 1 × cabezal 2×3 | Sonares: fila de 5 V y fila de GND | |
| TP1–TP4 | Puntos de prueba VBAT, 5V_MOT, 5V_LOG, GND | Para medir con el tester sin abrir nada |
| 4 agujeros M3 | Sujeción | |

## Pistas y masa (estimación mía)
- Fórmula IPC-2221 (cobre de 1 oz, 10 °C de subida): 4 A → ~2 mm de ancho; 8 A → ~5 mm. Usar 2,5–3 mm o rellenos.
- Plano de GND completo en la cara inferior. Los GND de todas las cargas se unen en **un solo punto**, cerca del negativo de J1.

## Lo que hay que definir antes de pedir la placa
1. **Pads de tus XL4015:** distancia entre ellos y diámetro de los agujeros. Medilo con calibre. Yo supuse entrada abajo y salida arriba en las esquinas, y eso hay que verificarlo.
2. **Alto de los capacitores** y del conjunto (si va dentro de una caja).
3. **Orientación del tablero:** 100 × 80 o 80 × 100.
4. **¿Ponemos la ESP32-S3 sobre hembras en esta misma placa?** Ahorra muchos cables de señal, pero la placa deja de ser solo de potencia.
5. **El fusible:** hoy es un puente; la bornera J2 ya prevé el portafusible.
