# Divergencias entre el roadmap y lo que se construyó

Revisión del 2026-11-19 (issue INF-09, #92). Compara las 68 tareas del [`ROADMAP.md`](ROADMAP.md) con el código de `dev` y de las ramas abiertas.
Cada fila dice **qué se hizo distinto**, **si el camino nuevo conviene** y **qué hacer**. "Se retoma" significa volver a lo planeado.

Criterio: un desvío es **positivo** si funcionó en el robot o en banco y reduce trabajo o riesgo; es **negativo** si dejó algo sin cubrir
(seguridad, pruebas, integración) que el plan sí cubría.

## Desvíos que conviene mantener

| # | Plan | Lo que se hizo | Veredicto | Acción |
|---|---|---|---|---|
| 1 | FW-01 a FW-05: PlatformIO + Arduino + `micro_ros_platformio`, `firmware/esp32_motion/` | ESP-IDF 5.5 + `micro_ros_espidf_component`, `firmware/banco/robot_4ruedas/` ([ADR 0003](../decisiones/0003-firmware-esp-idf.md)) | **Positivo.** Se probó en banco y anduvo (PCNT, LEDC, MCPWM sin capas intermedias) | Mantener. Reescribir los textos de #13 a #17 para ESP-IDF o cerrarlos como "reemplazados"; los tests nativos de la lógica pura quedan para el CI |
| 2 | NAV-04 y NAV-05: simulador abstracto de grilla, mapa topológico **en línea** y Trémaux por aristas | Mapa de **celdas** con Trémaux (`mapa.py`, `explorador.py`) y planificador propio ([ADR 0004](../decisiones/0004-navegacion-por-celdas-sin-nav2.md)); el grafo topológico se agrega después como capa 3 (#99) | **Positivo.** Más simple; el Trémaux por celdas corre en simulación (PR #87) y el de `tools/tremaux/` se usó en el robot | Mantener. Redefinir #33 y #34 contra esa arquitectura o cerrarlos si #99 los absorbe. Aceptar el ADR 0004 |
| 3 | Un paquete por columna: `caid_perception`, `caid_hw_bridge`, `caid_mission` | `caid_vision` (detector, validador, cámara y calibración) y `caid_navigation` (mapa, explorador y misión simulada) | **Positivo** (menos paquetes y menos launch) con una **pérdida**: no hay `core/` puro sin ROS ni tests en CI | Mantener los paquetes. Retomar la regla de "núcleo sin rclpy" para el código nuevo (#97 a #100 ya la cumplen) |
| 4 | INF-02: contrato con mensajes propios (`caid_mcu_msgs`, <500 B, sin strings) | El firmware real usa `std_msgs`: `/wheel_ticks` (Int32MultiArray), `/wheel_state` (34 Float32, índices fijos) y `/robot_cmd` (ids 1 a 9) | **Mixto.** Anduvo y evitó compilar mensajes en micro-ROS, pero los 34 índices son un contrato implícito y frágil | Mantener hoy. Documentar el contrato real (tópicos, índices, frecuencias) en `docs/arquitectura/contrato.md` y recién después decidir si vale migrar a mensajes tipados |
| 5 | Robot nuevo: ESP32 clásica | La ESP32 se quemó; port a ESP32-S3 sin compilar (#94) y rediseño de potencia con 2 × XL4015 y PCB (#95) | **Necesario.** No estaba en el roadmap y reemplaza parte de REAL-02 y REAL-03 | Issues creados. #58 (REAL-03) queda cubierto por #95 |
| 6 | SIM-01: emulador de firmware sobre `/sim/*` | No existe. La sim usa `DiffDrive` ideal y odometría casi perfecta; hay `odom_ruedas.py` en una rama | **Negativo, pero ya planificado**: sin el emulador, afinar la navegación en simulación engaña | Se retoma, con la guía de calibración ([`calibracion-sim2real.md`](../guias/calibracion-sim2real.md)) como diseño. Issues #42, #59 y #96 |

## Desvíos que conviene revertir (retomar lo planeado)

| # | Plan | Lo que se hizo | Por qué es un problema | Acción |
|---|---|---|---|---|
| 7 | NAV-03: `cmd_vel_gate` (prioridad teleop > manip > nav, timeout, freno frontal, salida única) | `tools/ruedas`, `tools/tremaux`, `tools/torneo` hablan **directo** con la ESP32 y mandan `/cmd_vel` y `/robot_cmd`. El propio código avisa: dos programas a la vez se pisan | Sin compuerta, la seguridad depende de que el operador cierre el otro programa. Además hay lógica duplicada (Trémaux en `tools/tremaux/` y en `caid_navigation`) y `mapa.py` copiado para correr desde Descargas | **Se retoma** la compuerta de #32. Mientras tanto, `tools/` queda como herramienta de banco y de competencia, no como arquitectura. La navegación nueva (#97 a #99) va en `caid_navigation` |
| 8 | INF-04: CI y rulesets; INF-01: ramas protegidas | No hay `.github/workflows/`, ni ISSUE_TEMPLATE, ni rulesets. El código roto puede entrar a `dev` sin aviso | Sin CI, `caid_vision/test_detector` se saltea (depende de `caid_navigation.simulacion`, que no está en `dev`) | **Se retoma** #6 y la parte de rulesets de #3. Prioridad alta y barata |
| 9 | Una rama por tarea, desde `dev`, con ID (`feature/<ID>-…`) | Ramas sin ID (`integracion`, `esp-vision-5-previo-al-sanguche`, `claude/*`), 4 ramas con 16 a 45 commits sin integrar | El conocimiento de lo que anda en el robot está fuera de `dev` | **Se retoma**: #93 (INF-10) integra por partes, y se podan las ramas mergeadas (13 hoy) |
| 10 | Documentar mientras se trabaja (memoria sección por sección) | Solo existe `09-simulacion.md` de 10 secciones | La memoria vale 20 puntos y es la base de la presentación | **Se retoma** #8 y #64. Las secciones salen de las guías que ya existen en `docs/guias/` |
| 11 | Tablero al día | 33 de los issues abiertos tenían trabajo hecho y ninguna marca; el repo y el tablero habían divergido | No se sabía qué faltaba | Ya corregido con la etiqueta `en-curso` y las subtareas tildadas. Mantenerlo es parte de cada PR (`Closes #n`) |

## Trabajo nuevo que no estaba en el roadmap

| Issue | Qué |
|---|---|
| #93 INF-10 | Integrar a `dev` el stack del robot real |
| #94 FW-07 | Port a ESP32-S3 |
| #95 HW-01 | Rediseño de potencia y PCB |
| #96 DESC-05 | Mundo y modelo calibrados con el robot |
| #97 a #99 NAV-08 a NAV-10 | Navegación por capas (control por paredes, flechas, grafo) |
| #100 CAM-06 | Geometría y calibración extrínseca de la cámara |

## Riesgos abiertos

- **Los resultados de las capas 1 a 3 salen de un simulador 2D con parámetros en parte supuestos.** No hay datos en el robot ([informe](../pruebas/navegacion-capas.md)). Hasta la campaña de calibración (#59), no hay que usarlos para decidir.
- **La semántica de las flechas** (corta/media/larga = 1/2/3 segmentos, sentido relativo) es un supuesto sin confirmar con la pista. Además `docs/arquitectura/laberintos.md` todavía describe las flechas del anexo anterior (punta 70×50 mm) y el código usa las del reglamento nuevo (base 29 mm; cuerpo 30, 60 y 95 mm).
- **Falta saber qué rama corrió en la competencia** y cómo le fue. Es el dato que decide qué congelar en `candidatos/` y qué se descarta.
