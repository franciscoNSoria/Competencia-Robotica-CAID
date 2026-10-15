# Competencia Robótica CAID 2026: "Laberinto inteligente"

Robot autónomo para la **Competencia de Arquitecturas Inteligentes Distribuidas (CAID 2026)**,
UTN-FRC, Categoría A (autonomía completa), subcategoría con GPU.

El robot recorre un laberinto desconocido, interpreta flechas de colores en las paredes, encuentra
tres cubos de 20 mm (rojo, verde y azul), los agarra con su brazo, los deja en su depósito y
vuelve a la zona de inicio. Todo sin intervención humana.

## Arquitectura en una imagen

```
                     red Wi-Fi (router en desarrollo / AP de la ESP32-CAM en competencia)
 ┌──────────────────────────────┐        ┌──────────────────────────────────────────────┐
 │ ESP32 (micro-ROS)            │──UDP──►│ PC con ROS 2 Jazzy (el "cerebro")            │
 │ motores, encoders,           │◄───────│  agente micro-ROS · percepción · navegación  │
 │ ultrasonidos, servos         │        │  y mapa · manipulación · misión              │
 └──────────────────────────────┘        │                                              │
 ┌──────────────────────────────┐──HTTP─►│  puente de video (MJPEG → tópico de imagen)  │
 │ ESP32-CAM: video MJPEG       │        └──────────────────────────────────────────────┘
 └──────────────────────────────┘
```

Las razones de cada decisión están en [`docs/decisiones/`](docs/decisiones/).

## Estructura del repositorio

| Carpeta | Contenido |
|---|---|
| [`docs/`](docs/) | Reglamento, roadmap, guías, decisiones de arquitectura y memoria técnica |
| [`hardware/`](hardware/) | STL, CAD, lista de partes y electrónica |
| [`firmware/`](firmware/) | Proyectos PlatformIO de la ESP32 y la ESP32-CAM |
| [`ros2_ws/`](ros2_ws/) | Workspace de ROS 2 con todos los paquetes |
| [`tools/`](tools/) | Scripts auxiliares (verificación del entorno, etc.) |

## Primeros pasos

```bash
git clone https://github.com/franciscoNSoria/Competencia-Robotica-CAID.git
cd Competencia-Robotica-CAID
source /opt/ros/jazzy/setup.bash
bash tools/setup/check_env.sh      # verifica que tengas todo instalado
```

Después:

1. Si es la primera vez: [`docs/guias/primeros-pasos.md`](docs/guias/primeros-pasos.md).
2. Leer el desafío: [`docs/reglamento/desafio-laberinto.md`](docs/reglamento/desafio-laberinto.md).
3. Ver el plan: [`docs/roadmap/ROADMAP.md`](docs/roadmap/ROADMAP.md).
4. Aprender el flujo de trabajo: [`CONTRIBUTING.md`](CONTRIBUTING.md) y
   [`docs/guias/github-projects.md`](docs/guias/github-projects.md).
5. Elegir una tarea en el tablero de GitHub Projects y empezar.

## Stack

Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic · micro-ROS (`micro_ros_platformio`, distro Jazzy) ·
PlatformIO · Python 3 / OpenCV
