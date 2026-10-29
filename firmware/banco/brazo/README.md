# Firmware de banco del brazo (ESP-IDF 5.5)

Programas de prueba de los servos del brazo, sin micro-ROS ni Wi-Fi. Se compilan como `main/main.c` de un proyecto de ESP-IDF,
igual que los de `firmware/banco/` (ver su README). Pines (`hardware/electronica/propuesta-v0.2.md`): base GPIO18, hombro GPIO19,
codo GPIO21, pinza GPIO22. Los servos se alimentan del riel de 5 V con GND común con la ESP32.

| Carpeta | Para qué |
|---|---|
| `pwm_auto/` | Sin teclado ni monitor: manda PWM de 50 Hz a hombro y codo apenas arranca y hace parpadear el LED (GPIO2). Cada 10 s mueve el codo y el hombro 200 µs (unos 23°) de golpe. Sirve para saber si la placa corre el programa y si los servos responden |
| `calibracion_servo/` | Un servo a la vez (`SERVO_ELEGIDO`): pulso contra ángulo por el monitor, con velocidad limitada y marcas. Es el de `docs/guias/calibracion-servo-base.md` |
| `hombro_codo/` | Hombro y codo coordinados para levantar el brazo (`l`, `L`, `b`, `B`), con adelanto del pulso (`>`/`<`) para aumentar el torque entregado |

Estado de las pruebas: `docs/guias/brazo-medidas.md`, sección "Pruebas de servos".
