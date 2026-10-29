# Firmware de banco (ESP-IDF 5.5)

Programas de prueba de la ESP32 de movimiento, escritos en ESP-IDF. Cada carpeta tiene un `main.c`. **No son todavía el firmware
definitivo** (`firmware/esp32_motion/`, ver ADR 0003): son lo que se usó en el banco.

## Cómo se compilan

Se copian como `main/main.c` dentro de un proyecto de ESP-IDF ya configurado. Para micro-ROS se usa el ejemplo `int32_publisher`
del componente `micro_ros_espidf_component` (rama `jazzy`), con la terminal limpia de ROS descrita en `docs/guias/micro-ros.md`:

```bash
cd ~/caid/pruebas-banco/micro_ros_espidf_component/examples/int32_publisher
cp main/main.c main/main.c.anterior
cp ~/caid/Competencia-Robotica-CAID/firmware/banco/robot_4ruedas/main.c main/main.c
idf.py build
idf.py -p /dev/ttyUSB0 -b 115200 flash monitor
```

Para flashear, **desconectar el cable del 5 V del shield al pin 5V de la ESP32** (nunca USB y 5 V a la vez) y mantener apretado BOOT
cuando aparezca `Connecting......`. Salir del monitor con `Ctrl+]`.

El Wi-Fi y la IP del agente van en `idf.py menuconfig` (`micro-ROS Settings`), no en el código.

## Programas

| Carpeta | Para qué | Usa micro-ROS |
|---|---|---|
| `motor_pi_1rueda/` | PI de **una** rueda con prealimentación, empujón y bloqueo (versión v8d del banco). Recibe `/cmd_vel` | Sí |
| `prueba_cableado/` | Prueba de las 4 ruedas con el monitor: giro a mano (encoders) y giro por motor | No |
| `prueba_cableado_autonoma/` | La misma prueba, sin PC: el resultado sale por el LED de la placa y se pasa de etapa con BOOT | No |
| `prueba_4ruedas_pi/` | PI por rueda en el aire y en el piso, con 1 m calculado por los encoders; resultados por LED | No |
| `robot_4ruedas/` | **El firmware de comunicación y PID de las 4 ruedas**: `/cmd_vel`, `/robot_cmd`, `/wheel_ticks`, `/wheel_state`. Se maneja con `tools/banco/robot_teclado.py` | Sí |

Documentación: `docs/guias/robot-4-ruedas-pid-y-comunicacion.md`, `docs/guias/cableado-4-ruedas.md` y `docs/guias/caracterizacion-motores.md`.
Los programas del brazo están en `firmware/banco/brazo/`.
