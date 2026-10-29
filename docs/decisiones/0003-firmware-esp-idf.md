# 0003. Firmware de la ESP32 de movimiento con ESP-IDF y micro_ros_espidf_component

- **Estado:** Propuesta
- **Fecha:** 2026-10-24

## Contexto

Los textos de FW-01 a FW-04 y de `docs/guias/entorno.md` planean el firmware con PlatformIO, el framework Arduino y
`micro_ros_platformio`. En el banco (2026-10-24) se probó otro camino con ESP-IDF 5.5:

- El periférico PCNT cuenta los flancos de un encoder por hardware y la lectura de un HC-SR04 por polling con `esp_timer` mide con un desvío de décimas de milímetro.
- El componente `micro_ros_espidf_component` (rama `jazzy`, versión 24.x) compila, se conecta por Wi-Fi a un agente micro-ROS en la PC y publica un tópico que ROS 2 recibe (ver `docs/guias/micro-ros.md`).

Restricciones: el robot es el estándar de la organización (Art. 8 y 10 del reglamento), el pin map A usa PCNT, captura MCPWM y LEDC,
el tiempo es corto y todo tiene que funcionar sin internet en la competencia.

## Decisión

El firmware de la **ESP32 de movimiento** se escribe en **C/C++ con ESP-IDF 5.5**, se compila con `idf.py` y usa el componente
`micro_ros_espidf_component` (rama `jazzy`, versión `^24`). La ESP32-CAM queda fuera de este ADR: su firmware lo define CAM-01.

## Alternativas consideradas

- **PlatformIO con Arduino y `micro_ros_platformio` (el plan original).** No se probó en el banco. No se descarta, pero obligaría a validar de nuevo micro-ROS, PCNT y MCPWM sobre otra capa, y ya existe un camino verificado.
- **PlatformIO como envoltorio de ESP-IDF.** Facilitaría `pio test -e native`, pero agrega una herramienta más sin que hoy haga falta. Se puede revisar si los tests de la lógica pura resultan difíciles con `idf.py`.

## Consecuencias

- Hay un camino ya probado: compilar, flashear y hablar con ROS 2 por Wi-Fi.
- Menos capas entre el código y los periféricos (PCNT, LEDC, MCPWM).
- La compilación de micro-ROS exige una **terminal sin ROS cargado** y la primera vez tarda; queda documentado en `docs/guias/micro-ros.md`.
- Los **tests nativos de la lógica pura** (FW-02, `pio test -e native`) y el **CI** (INF-04) necesitan otra solución: por ejemplo, la lógica como componente sin dependencias del hardware, probada en la PC con un CMake propio, o el target `linux` de ESP-IDF (a verificar).
- Hay que **actualizar** los textos de FW-01, FW-02, FW-03 y FW-04 (`platformio.ini`, `pio run`, `pio test`, `include/pins.h`, `lib/motion`, `src/*.cpp`), `firmware/README.md` y `docs/guias/entorno.md`.
- El pin map A no cambia.
