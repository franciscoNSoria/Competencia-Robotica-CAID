# Firmware

Código que corre en los microcontroladores. Cada placa es un proyecto de PlatformIO independiente;
cada uno se crea en su tarea del roadmap.

| Carpeta (se crea en su tarea) | Placa | Qué hace |
|---|---|---|
| `esp32_motion/` | ESP32 NodeMCU 38 pines | micro-ROS: motores (L9110S), encoders, ultrasonidos, servos del brazo, watchdog de seguridad |
| `esp32_cam/` | ESP32-CAM AI-Thinker | Video MJPEG por HTTP, ajustes de cámara y, según la red, punto de acceso Wi-Fi |

### Arquitectura del Firmware (esp32_motion)

```
firmware/esp32_motion/
├── include/
│   ├── shared_data.h    <-- (Variables globales, Mutex y Queues)
│   ├── core_zero.h      <-- (Cabecera del Core 0)
│   └── core_one.h       <-- (Cabecera del Core 1)
├── src/
│   ├── main.cpp         <-- (Solo inicializa hardware y lanza las tareas)
│   ├── core_zero.cpp    <-- (Lógica del PID y sensores)
│   └── core_one.cpp     <-- (Lógica de micro-ROS y telemetría)
```

El firmware de la placa principal (ESP32) aprovecha su procesador de doble núcleo utilizando **FreeRTOS**. Las responsabilidades se dividen estrictamente para evitar que el tráfico de red interfiera con la física del robot:

*   **Core 0 (Control Reactivo y Tiempo Real):** 
    Ejecuta un bucle de alta prioridad. Su única función es leer los *ticks* de los encoders, medir la distancia de los 3 sensores ultrasónicos y ejecuta el control PID para mantener el robot centrado y a la velocidad deseada (`target_v`, `target_w`). También incorpora sistemas de seguridad de bajo nivel, como frenado automático ante obstáculos inminentes y un *watchdog* que detiene los motores si se pierde comunicación con la PC.

*   **Core 1 (Comunicaciones y micro-ROS):** 
    Maneja el cliente XRCE-DDS sobre Wi-Fi/UDP. Este núcleo atiende las suscripciones enviadas por la PC (como `/cmd_vel` para velocidad, y comandos para los actuadores del brazo y servo de la cámara). A su vez, recolecta la información del Core 0 y publica los tópicos de telemetría (`/odom` y `/ultrasonicos/rango`) para que la PC pueda construir la memoria de ruta y tomar decisiones.

#### Sincronización entre nucleos
Dado que ambos núcleos comparten memoria, se comunican de forma segura mediante primitivas de FreeRTOS:

*   **Mutex:** Se utiliza para proteger las variables de consigna de velocidad (`target_v` y `target_w`). Cuando el Core 1 recibe un mensaje nuevo por micro-ROS, toma el Mutex para actualizar la meta, garantizando que el Core 0 jamás lea velocidades a medio sobrescribir durante su cálculo del PID.

*   **Queues (Colas de mensajes):** Los datos crudos de odometría y ultrasonidos se empaquetan en el Core 0 y se envían al Core 1 mediante una *Queue* no bloqueante, permitiendo que el Core 1 los procese y publique por Wi-Fi a su propio ritmo sin demorar el control de los motores.


## Reglas

- micro-ROS se compila con `micro_ros_platformio` y **`board_microros_distro = jazzy`**. El valor por
  defecto es otra distribución y no se entiende con nuestro ROS 2 Jazzy.
- Claves Wi-Fi e IPs **nunca** en el código: van en `secrets.h` (ignorado por git), y se versiona
  `secrets.example.h` como modelo.
- Cada versión de firmware que va al robot se etiqueta en git (`fw-esp32-vX.Y.Z`, `fw-cam-vX.Y.Z`)
  y el firmware informa su versión al arrancar. La inspección técnica verifica la versión.
