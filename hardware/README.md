# Hardware

Todo lo físico del robot: piezas impresas, lista de partes y electrónica.

| Carpeta | Qué va |
|---|---|
| `stl/` | STL **originales** que entregó la organización, sin modificar |
| `cad/` | Archivos fuente de CAD (SolidWorks / Fusion 360), si se versionan |
| `bom/` | Lista de partes (`Lista_de_partes.ods`, provista por la organización) |
| `electronica/` | Conexionado, pinout y alimentación. Hoy: [`propuesta-v0.2.md`](electronica/propuesta-v0.2.md), a confirmar con mediciones y con la organización |

Las mallas que usa la descripción del robot (URDF) **no** van acá: se exportan desde el CAD con el
origen en cada articulación y van en `ros2_ws/src/caid_description/meshes/`.

## Componentes confirmados

| Componente | Detalle |
|---|---|
| Controlador de movimiento | ESP32 NodeMCU 38 pines (ESP32-WROOM-32) con micro-ROS |
| Cámara | ESP32-CAM (AI-Thinker, OV2640) |
| Tracción | 4 motorreductores amarillos "TT" con ruedas, driver L9110S |
| Encoders | 4 sensores ópticos de ranura FC-03 (uno por rueda), permitidos en competencia |
| Brazo | 3 GDL + pinza, servos SG90 |
| Cámara con giro | 1 servo SG90 (pan) |
| Distancia | 3 HC-SR04: frontal, lateral izquierdo y lateral derecho |
| Energía | 2 celdas 18650 XTAR 20 A en un módulo de baterías con salidas de 5 V / 3 A y 3 V / 1 A |

Pendiente de confirmar: cuántos canales tiene el módulo L9110S (2 o 4) y cuántas ranuras tienen los
discos de los encoders.
