# Roadmap CAID 2026: laberinto inteligente

Las 68 tareas viven en el tablero de GitHub Projects (`CAID 2026 Laberinto`). Este documento es el mapa:
cómo se organizan, cuándo se integran y qué se demuestra en cada fecha.

## Cómo se organiza el trabajo

- **Columna:** una cadena de tareas que se hacen en orden (la tarea 2 necesita la 1).
- **Troncal:** una columna que arranca hoy y no depende de otra.
- **Desbloqueada:** una columna que se abre cuando se cierran tareas de otras (se ve el ícono "Blocked").
- **Unión:** el punto donde se juntan columnas y se habilita algo nuevo.
- **Hito:** fecha de integración. Se prueba una demo, `dev` pasa a `main` y se pone un tag.
- Nadie está asignado de antemano: cada persona elige una columna y la sigue hasta terminarla,
  documentando mientras trabaja. Una sola tarea "En curso" por persona.

## Columnas

| Columna | Tipo | Tareas | Qué resuelve |
|---|---|---|---|
| INF Infraestructura | troncal | 7 | Repo, contrato de interfaces, tablero, CI, memoria y maqueta |
| DESC Modelo 3D | troncal | 4 | Robot en xacro/URDF y mundos de Gazebo |
| FW Firmware | troncal | 5 | Firmware de la ESP32 de movimiento (banco) |
| CAM Cámara y red | troncal | 5 | ESP32-CAM, video y red |
| PER Percepción | troncal | 7 | Flechas, cubos y depósitos (OpenCV clásico) |
| NAV Navegación | troncal | 7 | Paredes, mapa en línea, exploración y política de misión |
| MAN Brazo | troncal | 5 | Cinemática, driver y agarre del brazo |
| SIM Simulación | desbloqueada | 6 | Navegación completa en Gazebo |
| VIS Ver y agarrar | desbloqueada | 3 | Ver, acercarse, agarrar y depositar en simulación |
| MIS Misión | desbloqueada | 5 | Misión completa en simulación |
| REAL Hardware real | desbloqueada | 8 | Banco y robot físico |
| CIE Cierre | desbloqueada | 6 | Memoria, presentación, inspección y ensayo general |

## Uniones principales

- `INF-02` (contrato de interfaces) habilita `DESC-04`, `FW-04`, `PER-04`, `NAV-03` y `MAN-03`.
- `SIM-01` espera a `DESC-03` e `INF-02`.
- `VIS-01` espera a `SIM-02`, `PER-04`, `MAN-03` y `DESC-04`.
- `MIS-01` espera a `SIM-06`, `VIS-03` y `NAV-07`.
- `REAL-01` espera a `FW-03`, `CAM-02` y `NAV-03`. `REAL-02` además necesita el robot ensamblado.
- `CIE-04` espera a `MIS-05` y `REAL-06`. `CIE-05` espera a `REAL-08`.
- Ruta crítica: **DESC → SIM → VIS → MIS**. Siempre tiene que haber alguien sobre ella.

## Hitos (fechas propuestas)

| Hito | Fecha | Tag | Qué se demuestra |
|---|---|---|---|
| H0 Base y contrato | 17/10 | v0.1.0-h0 | Repo listo y contrato de interfaces congelado |
| H1 Teleop de punta a punta | 20/10 | v0.2.0-h1 | Manejo del robot desde la PC, en simulación y en banco |
| H2 Ver y acercarse | 24/10 | v0.3.0-h2 | Pasillos, giros y mapa abstracto; ver y acercarse a un cubo |
| H3 Agarrar al ver | 28/10 | v0.4.0-h3 | Agarrar al ver un cubo y explorar con islas |
| H4 Misión en simulación | 30/10 | v0.5.0-h4 | Misión completa en Gazebo (3 cubos entregados y regreso) |
| H5 Versión 1.0 | 31/10 | v1.0.0 | Ensayo general y versión de competencia |

## Comportamiento de la misión

El robot construye un mapa topológico mientras recorre, vacío al empezar cada ronda. Si ve un cubo con
la pinza libre, lo agarra en el momento. Si conoce el depósito de su color, va por el camino más corto;
si no, sigue explorando con el cubo. Con los 3 entregados vuelve al inicio por el camino más corto.
Funciona con islas porque marca los pasillos recorridos, no las celdas. Las flechas aceleran la
exploración, nunca la reemplazan.

## Reglas de merge

- Una rama por tarea: `feature/<ID>-<descripcion>`. Squash and merge a `dev`, con `Closes #n` en el PR.
- `dev` pasa a `main` solo en cada hito, con merge commit y tag.
- Una tarea está Hecha cuando se cumple su "Terminada cuando", está documentada y otra persona aprobó el PR.
