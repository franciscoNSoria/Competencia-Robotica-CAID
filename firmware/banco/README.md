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

`robot_4ruedas/main.c` necesita, además, los archivos que no van en este repo junto a él (van en `main/` cuando se compila):
`sonar.h` y `guardia.h` de `firmware/pruebas/comun/`, y `servos_brazo.h` de `firmware/brazo_sliders/` (ver más abajo, es
**un solo firmware** para ruedas, sonares, guardia y brazo: no hace falta reflashear para cambiar de uno al otro).

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
| `robot_4ruedas/` | **El firmware de comunicación y PID de las 4 ruedas, más el brazo**: `/cmd_vel`, `/robot_cmd`, `/wheel_ticks`, `/wheel_state`, `/brazo/cmd`, `/brazo/estado`, sonares y guardia. Ruedas con `tools/banco/robot_teclado.py`, brazo con `tools/brazo/brazo_sliders.py` | Sí |

Documentación: `docs/guias/robot-4-ruedas-pid-y-comunicacion.md`, `docs/guias/cableado-4-ruedas.md` y `docs/guias/caracterizacion-motores.md`.
Los programas sueltos del brazo (otras versiones, no fusionadas) están en `firmware/banco/brazo/` y `firmware/brazo_sliders/`.

## `robot_4ruedas`: por qué incluye el brazo

Es **una sola ESP32** para la prueba de banco "sonar + brazo a comando sin cámara" (`tools/pruebas/prueba_autonomia_sonar_brazo_comando_sin_camara/`):
el robot tiene que explorar con los sonares, pararse, y ahí mismo mover el brazo con los sliders sin sacar la placa ni
reflashear. `robot_4ruedas/main.c` es la fusión de ese firmware (ruedas, sonares, guardia) con `firmware/brazo_sliders/`
(los 4 servos del brazo): mismo protocolo en `/brazo/cmd` y `/brazo/estado`, misma lógica de reposo seguro antes de girar
la base. No compiten por hardware: las ruedas usan los canales LEDC de "high speed" (0 a 7) y los servos los de "low
speed" (0 a 3), dos periféricos aparte en la ESP32 clásica, y los pines de los servos (18, 19, 21, 22) no se usan en
ningún otro lado del firmware.

Para flashear esta versión fusionada hacen falta, todos juntos en `main/`: `robot_4ruedas/main.c` (este archivo),
`firmware/pruebas/comun/sonar.h`, `firmware/pruebas/comun/guardia.h` y `firmware/brazo_sliders/servos_brazo.h`. **No
hace falta `enlace_uros.h` ni el `main.c` de `brazo_sliders/`**: la conexión con el agente y el envío de `/brazo/cmd`
y `/brazo/estado` ya están en `robot_4ruedas/main.c`, integrados con el mismo executor que usan `/cmd_vel` y `/robot_cmd`.

```bash
cd ~/caid/pruebas-banco/micro_ros_espidf_component/examples/int32_publisher
cp main/main.c main/main.c.anterior
cp ~/caid/Competencia-Robotica-CAID/firmware/banco/robot_4ruedas/main.c main/main.c
cp ~/caid/Competencia-Robotica-CAID/firmware/pruebas/comun/sonar.h main/
cp ~/caid/Competencia-Robotica-CAID/firmware/pruebas/comun/guardia.h main/
cp ~/caid/Competencia-Robotica-CAID/firmware/brazo_sliders/servos_brazo.h main/
idf.py build
idf.py -p /dev/ttyUSB0 -b 115200 flash monitor
```

Después de flashear, con el agente corriendo y la ESP32 conectada, los dos tópicos del brazo ya están disponibles junto
con los de las ruedas: podés correr `tools/brazo/brazo_sliders.py` y `tools/pruebas/prueba_autonomia_sonar_brazo_comando_sin_camara/explorar_sin_camara.py`
(o `caid_bringup`) al mismo tiempo, sin tocar la placa entre uno y otro.

**Verificar antes de confiar en el combinado**: probar primero que las ruedas, los sonares y la guardia siguen
funcionando igual que antes de la fusión (lo que ya estaba andando y flasheado), y recién después activar el brazo con
los sliders. Si algo de las ruedas se comporta distinto después de flashear esta versión, es señal de que el brazo está
compitiendo por algo (tiempo de CPU, memoria) y hay que avisar antes de seguir.
