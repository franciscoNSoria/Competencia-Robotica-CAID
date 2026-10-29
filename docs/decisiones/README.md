# Decisiones de arquitectura (ADR)

Un ADR ("Architecture Decision Record") es una nota corta que registra **una decisión técnica
importante**: qué decidimos, por qué, qué alternativas descartamos y qué consecuencias tiene.
Sirve para que cualquiera del equipo (o el jurado, en la memoria técnica) entienda por qué el
proyecto es como es, sin depender de que alguien se acuerde de una charla.

## Cuándo escribir uno

Cuando una decisión afecta a más de una columna del roadmap o sería cara de revertir.
Ejemplos: topología de red, formato de los mensajes entre el robot y la PC, cómo se agarra el cubo.

## Cómo

1. Copiar `plantilla.md` como `NNNN-titulo-corto.md` (número siguiente, 4 dígitos).
2. Completarlo y abrir un PR. La discusión ocurre en el PR.
3. Al mergear, el estado pasa a **Aceptada**. Si más adelante cambia, no se borra: se marca
   **Reemplazada por NNNN** y se escribe uno nuevo.

## Índice

| N.º | Decisión | Estado |
|---|---|---|
| [0001](0001-topicos-simples-sin-ros2-control.md) | Robot expuesto por tópicos simples de micro-ROS, sin ros2_control | Propuesta |
| [0002](0002-topologia-de-red.md) | Topología de red: router en desarrollo, punto de acceso de la ESP32-CAM en competencia | Propuesta (falta confirmar con la organización) |
| [0003](0003-firmware-esp-idf.md) | Firmware de la ESP32 de movimiento con ESP-IDF y micro_ros_espidf_component | Propuesta |
| [0004](0004-navegacion-por-celdas-sin-nav2.md) | Navegación por celdas con planificador propio, sin Nav2 ni SLAM | Propuesta |
