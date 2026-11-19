# Firmware

Código que corre en los microcontroladores. Todo se compila con **ESP-IDF 5.5** (ADR 0003); el plan original con PlatformIO se descartó.

| Carpeta | Placa | Qué hace |
|---|---|---|
| `banco/robot_4ruedas/` | ESP32 NodeMCU 38 pines (la S3 está en la rama `feature/FW-07-port-esp32-s3`) | micro-ROS: PI por rueda, encoders, sonares, servos; es el firmware que se probó en el robot |
| `banco/` (resto) | ESP32 | Programas de prueba de banco por componente |
| `esp32_cam/` | ESP32-CAM AI-Thinker | Video MJPEG por HTTP, ajustes de cámara y, según la red, punto de acceso Wi-Fi |

## Reglas

- micro-ROS se compila con `micro_ros_espidf_component` (rama `jazzy`) en una terminal **sin ROS cargado**;
  ver `docs/guias/micro-ros.md`. Otra distribución no se entiende con nuestro ROS 2 Jazzy.
- Claves Wi-Fi e IPs **nunca** en el código: van en `secrets.h` (ignorado por git), y se versiona
  `secrets.example.h` como modelo.
- Cada versión de firmware que va al robot se etiqueta en git (`fw-esp32-vX.Y.Z`, `fw-cam-vX.Y.Z`)
  y el firmware informa su versión al arrancar. La inspección técnica verifica la versión.
