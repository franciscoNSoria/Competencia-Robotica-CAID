# Cableado de las 4 ruedas (motores + encoders)

> Borrador para `docs/guias/cableado-4-ruedas.md`. Se apoya en `hardware/electronica/propuesta-v0.2.md` (mapa B-rueda, sección 5.3).
> Estados: **V** verificado en documentación del repo, **P** propuesta mía sin probar, **M** hay que medirlo.
> Supuesto: dos módulos L9110S de 2 canales (uno por lado). Es lo que se usó en el banco ("canal B del L9110S derecho").

## 1. Decisión de diseño

Cada rueda tiene sus **dos entradas propias** (IA e IB) y su **encoder propio**. Eso permite un PI por rueda, corregir la
polaridad por software y escalonar los empujones. Cuesta 8 pines de motor en vez de 4 (mapa A del repo).

Cambio respecto de la propuesta v0.2 (P): `RR_IB` pasa del GPIO2 al **GPIO32**, porque el GPIO2 es de arranque (tiene que estar
en bajo para programar por USB [V, propuesta §5.1]) y el módulo podría tener pull-up en sus entradas. `ECHO_R` queda **sin
asignar** hasta que se cableen los sonares.

## 2. Tabla de pines (ESP32 DevKit de 38 pines)

| Rueda | Señal | GPIO | Va a |
|---|---|---|---|
| FL (delantera izq.) | IA / IB | 25 / 26 | Módulo IZQ, A-IA / A-IB (motor A) |
| RL (trasera izq.) | IA / IB | 27 / 16 | Módulo IZQ, B-IA / B-IB (motor B) |
| FR (delantera der.) | IA / IB | 17 / 33 | Módulo DER, B-IA / B-IB (motor B) |
| RR (trasera der.) | IA / IB | 23 / 32 | Módulo DER, A-IA / A-IB (motor A) |
| FL | encoder D0 | 34 | D0 del FC-03 de FL |
| RL | encoder D0 | 35 | D0 del FC-03 de RL |
| FR | encoder D0 | 36 (VP) | D0 del FC-03 de FR |
| RR | encoder D0 | 39 (VN) | D0 del FC-03 de RR |

No usar: GPIO 6–11 (flash), 1 y 3 (UART del USB), 0 y 12 (arranque), 2 (arranque/LED), 15 y 5 (arranque, reservados a sonares).
Algunas placas rotulan el 16 como RX2 y el 17 como TX2: confirmar con la serigrafía de la placa.

## 3. Conexiones

**Alimentación y masa**
- Riel de 5 V del shield de baterías → `VCC` de los dos módulos (cables cortos y gruesos). Riel de GND del shield → `GND` de los dos módulos.
- **Un cable de masa común** entre el GND del shield y un GND de la ESP32. Soldado o con bornera de tornillo, no con dupont suelto
  (el contacto flojo en ese cable hizo girar las ruedas solas). Se conecta primero y se desconecta último.
- Durante las pruebas, la ESP32 va por **USB**. No conectar el 5 V del shield al pin 5V de la ESP32 mientras esté el USB [V, propuesta v0.2].

**Motores**: cada motor a su borne (A o B) del módulo de su lado, como indica la tabla. La polaridad no importa ahora: se
corrige por software (por eso `prueba` pide anotar el sentido real).

**Encoders (FC-03)**: `VCC` al pin **3V3** de la ESP32 (así D0 sale a 3,3 V), `GND` al GND de la ESP32, `D0` al GPIO de la tabla. `A0` sin conectar.

**Opcional, si los tenés** [propuesta §6]: 100 nF cerámico soldado en los bornes de cada motor; 470 µF + 100 nF en `VCC`/`GND` de cada módulo.

## 4. Antes de energizar (todo apagado, USB desconectado)

1. **Continuidad de masa** (tester en modo continuidad): GND de la ESP32 ↔ GND de cada módulo ↔ GND del shield. Tiene que sonar siempre; mové los cables mientras medís.
2. **Pull-downs** (tester en ohmios, antes de conectar las señales): resistencia entre cada entrada (A-IA, A-IB, B-IA, B-IB) y GND del módulo.
   Si da un valor estable, el módulo ya las fija a 0. Si da "OL" (abierto), poner 10 kΩ de cada entrada a GND (8 resistencias). Esto evita tirones al reiniciar. **M**
3. **Cortocircuitos**: entre VCC y GND de cada módulo y del 3V3 y GND de la ESP32 no debe sonar la continuidad.
4. Etiquetar con cinta cada cable (FL_IA, FL_IB, FL_ENC…).

## 5. Pruebas

1. Cablear todo y cargar `main_prueba_cableado.c` (sin micro-ROS).
2. **Fase 1 (20 s, motores sin alimentar)**: girar cada rueda a mano; en el monitor sube el contador de **su** encoder. 40 cuentas por vuelta.
3. Antes de la fase 2 hay 10 s: encender la alimentación de los motores.
4. **Fase 2**: mueve **una rueda por vez** (1 s, duty 1,0), adelante y atrás, ruedas en el aire. Compara los 4 encoders y dice OK, MAL o DUDOSO.
5. Anotar, por rueda, **si "adelante" giró en el sentido en que avanzaría el robot**. Eso define la polaridad en el firmware final.

## 6. Pendientes

- ECHO_R (sonar derecho) y el resto del mapa: se asignan al cablear los sonares.
- Medir la corriente del riel de motores con las 4 ruedas (tester en serie, rango 10 A), en el aire y luego en el piso.
- Decidir si hace falta un segundo módulo de baterías para la lógica [propuesta v0.2, mejoras propuestas].
