# Firmware de movimiento (ESP32): proyecto autonomo

Ruedas (PI por rueda), encoders, 3 sonares con freno local, brazo, micro-ROS por Wi-Fi. `main/` tiene el `main.c` y los `.h` locales
que usa; nada se copia a otro proyecto.

```bash
./configurar_wifi.sh            # una vez: toma red, clave e IP del agente de tu ejemplo que ya anda (no imprime la clave)
./compilar.sh                   # compila (el primer build tarda: reconstruye micro-ROS)
./compilar.sh grabar            # compila y graba en /dev/ttyUSB0
```

- Terminal LIMPIA de ESP-IDF (sin ROS), ESP32 fuera del shield al grabar, sin `monitor`.
- La placa se elige con `CAID_TARGET` (por defecto `esp32`; para la ESP32-S3 el firmware hay que portarlo: ver `ESTADO.md`).
- Si no compila, el camino de siempre sigue funcionando: copiar `main/*` sobre `main/` del ejemplo `int32_publisher`.
