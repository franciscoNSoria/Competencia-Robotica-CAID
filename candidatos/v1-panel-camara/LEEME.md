# v1-panel-camara (congelada)

Carpeta autonoma: tiene todo lo necesario para compilar los firmwares y correr el robot real, sin depender de la rama en la que
estes parado ni de ~/Descargas. **No se edita**: si algo mejora, se congela otra version y se anota en `../VERSIONES.md`.

- Que anda y que no: [ESTADO.md](ESTADO.md)
- Como correrla: [COMO_CORRER.md](COMO_CORRER.md)
- De donde salio cada archivo: [ORIGEN.txt](ORIGEN.txt)
- Que no se cambio desde que se congelo: `sha256sum -c SHA256SUMS` (desde esta carpeta)

```
pc/ruedas/        panel, laberinto por celdas con sonares, seguidor de flechas (Python)
launch/           robot.launch.xml: camara + vision (+ visor)
ros2_ws/src/      caid_vision (camara, vision, calibracion), caid_navigation/mapa.py (el Tremaux que usa el laberinto)
firmware/         movimiento/ (ESP32) y camara/ (ESP32-CAM), cada uno un proyecto ESP-IDF autonomo
config/           lanzamiento.env, mi_maquina/ (calibracion.json y env_ros.sh de quien congelo)
scripts/          un script por terminal
```
