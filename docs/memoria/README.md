# Memoria técnica

Entregable obligatorio del reglamento: **PDF de 20 páginas como máximo**. Vale 20 puntos, y además
es la base de la presentación oral (15 puntos) y del puntaje de innovación (15 puntos).

## Cómo se construye

La memoria **no se escribe al final**: cada tarea del roadmap indica en qué sección documentar lo
que hizo (campo "Documentar en"). Cada sección es un archivo Markdown en `secciones/` y el PDF se
genera a partir de esos archivos (la tarea de documentación define cómo: pandoc o similar).

## Índice previsto

| Archivo (en `secciones/`) | Contenido |
|---|---|
| `01-resumen.md` | Qué hace el robot y cuál es la idea central de la solución |
| `02-arquitectura.md` | Arquitectura distribuida: ESP32, ESP32-CAM y PC con ROS 2; diagrama de nodos y tópicos |
| `03-hardware.md` | Plataforma, electrónica, alimentación y conexionado |
| `04-firmware-y-comunicacion.md` | micro-ROS, red, video, seguridad (watchdog) |
| `05-percepcion.md` | Detección de flechas, colores, cubos y depósitos |
| `06-navegacion-y-mapeo.md` | Recorrido del laberinto, mapa en línea y uso de las flechas |
| `07-manipulacion.md` | Brazo, aproximación, agarre y verificación |
| `08-mision.md` | Estrategia de la misión y máquina de estados |
| `09-simulacion.md` | Modelo del robot y simulación en Gazebo |
| `10-pruebas-y-resultados.md` | Métricas, ensayos y lecciones aprendidas |

Reglas: figuras y tablas en lugar de párrafos largos, cada afirmación técnica respaldada por una
medición o una prueba, y referencias a las decisiones de `docs/decisiones/`.
