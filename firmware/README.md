# Firmware

Código que corre en los microcontroladores. Cada placa es un proyecto de PlatformIO independiente;
cada uno se crea en su tarea del roadmap.

| Carpeta (se crea en su tarea) | Placa | Qué hace |
|---|---|---|
| `esp32_motion/` | ESP32 NodeMCU 38 pines | micro-ROS: motores (L9110S), encoders, ultrasonidos, servos del brazo, watchdog de seguridad |
| `esp32_cam/` | ESP32-CAM AI-Thinker | Video MJPEG por HTTP, ajustes de cámara y, según la red, punto de acceso Wi-Fi |

## Reglas

- micro-ROS se compila con `micro_ros_platformio` y **`board_microros_distro = jazzy`**. El valor por
  defecto es otra distribución y no se entiende con nuestro ROS 2 Jazzy.
- Claves Wi-Fi e IPs **nunca** en el código: van en `secrets.h` (ignorado por git), y se versiona
  `secrets.example.h` como modelo.
- Cada versión de firmware que va al robot se etiqueta en git (`fw-esp32-vX.Y.Z`, `fw-cam-vX.Y.Z`)
  y el firmware informa su versión al arrancar. La inspección técnica verifica la versión.
