# Bill of Materials - ARMADILLO (Todas las Placas)

Esta lista contiene componentes que aun no fueron verificados

---

## 1. Placa Control Brazo

| Cant. | Designador | Componente | Descripción | Footprint |
|:-----:|:----------:|:----------:|:------------|:---------:|
| 1 | DS1 | LED 3MM | LED 3MM | LED-3MM |
| 1 | J1 | Brazo | Header, 10-Pin (doble fila) | HDR2X5 |
| 4 | P1, P2, P3, P4 | Header 3 | Header, 3-Pin | HDR1X3 |
| 1 | R1 | Resistor | 1K | C1206 |


---

## 2. Placa Control Robot (Placa Principal)

| Cant. | Designador | Componente | Descripción | Footprint |
|:-----:|:----------:|:----------:|:------------|:---------:|
| 1 | C1 | Capacitor electrolítico aluminio, radial. | 330µF/25V | CAPPRD500W60D1000H1350 |
| 1 | C3 | Capacitor | .1uF | C1206 |
| 1 | J1 | entrada5V | Bornera 2 pines, paso 5 mm | BORNERA_T_VERDE |
| 1 | J3 | ESP32CAM | Header, 10-Pin (conector hacia placa ESP32-CAM) | HDR2X5 |
| 1 | J4 | Brazo | Header, 10-Pin (conector hacia placa Control Brazo) | HDR2X5 |
| 1 | P1 | Header 7X2 | Header, 7-Pin, doble fila | HDR2X7 |
| 1 | P2 | Header 5X2 | Header, 5-Pin, doble fila | HDR2X5 |
| 1 | Q1 | BC817 | Transistor bipolar NPN | SOT-23_L |
| 4 | R1, R3, R5, R6 | Resistor | 10K |  C1206 |
| 2 | R2, R4 | Resistor | ??? | C1206 |
| 4 | R7, R8, R10, R11 | Resistor | 330 | C1206 |
| 1 | R9 | Resistor | 15K | C1206 |
| 1 | U1 | ESP32_NODEMCU | NodeMCU — ESP32 SoC con Wi-Fi y Bluetooth dual | MODULE_ESP32_NODEMCU |
| 1 | U2 | MC54HC14AJ | Hex Schmitt Trigger Inverter | DIP-14 |
| 2 | U3, U4 | Optoisolator2 | Optoacoplador con fototransistor, 4-Pin | DIP-4 |

Los valores de R2 y R4 no figuran ni en el PDF ni en el Excel.

---

## 3. Placa ESP32-CAM

| Cant. | Designador | Componente | Descripción | Footprint |
|:-----:|:----------:|:----------:|:------------|:---------:|
| 1 | C1 | Capacitor polarizado (radial) | ??? | — |
| 1 | J1 | ESP32-CAM | ESP32 Transceiver 802.11 a/b/g/n (Wi-Fi), Bluetooth® Smart 4.x Low Energy (BLE) — Placa de evaluación | — |
| 1 | J2 | ESP32CAM | Header, 10-Pin | — |
| 1 | P1 | Header 3 | Header, 3-Pin | — |
| 1 | Q1 | NPN | Transistor bipolar NPN | — |
| 1 | R1 | Resistor | 10K | — |
| 1 | R2 | Resistor | 4K7 | — |

Los valores de C1 no figuran ni en el PDF ni en el Excel.

---

## 4. Placa Motor

| Cant. | Designador | Componente | Descripción | Footprint |
|:-----:|:----------:|:----------:|:------------|:---------:|
| 1 | C1 | Capacitor polarizado (radial) | 47uF/25V | ELECTROLITICO-220/16V |
| 2 | P1, P2 | Header 6 | Header, 6-Pin | HDR1X6 |
| 1 | P3 | Header 5X2H | Header, 5-Pin, doble fila | HDR2X5H |

---

## 5. Placa Concentrador Delantero

| Cant. | Designador | Componente | Descripción | Footprint |
|:-----:|:----------:|:----------:|:------------|:---------:|
| 2 | U1, U2 | TCST2103 | Sensor óptico transmissive | DIP-4 |
| 1 | P2 | Header 7X2 | Header, 7-Pin, doble fila | HDR2X7 |
| 1 | P5 | Header 7 | Header, 7-Pin | HDR1X7 |
| 2 | R2, R4 | Resistor | 150 | C1206 |
| 2 | R1, R3 | Resistor | 10K | C1206 |

---
