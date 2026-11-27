# Como se corre (robot real, laberinto con cubo)

Cada comando en su propia terminal, siempre desde esta carpeta. Las terminales de ESP-IDF y las de ROS NO se mezclan.

## Una sola vez

1. `scripts/0_construir.sh`: compila los paquetes de ROS 2 de esta carpeta (terminal sin ESP-IDF).
2. Firmware: `firmware/movimiento/` (`./configurar_wifi.sh`, `./compilar.sh grabar`) y `firmware/camara/` (`./compilar.sh grabar`).
   Terminal limpia de ESP-IDF, placa fuera del shield, sin `monitor`.
3. Revisar `config/lanzamiento.env` (IP de la camara, distancias de las flechas).

## El dia

| Terminal | Comando | Que es |
|---|---|---|
| 1 | `scripts/1_agente.sh` | Agente micro-ROS. Antes de encender el robot |
| 2 | `scripts/2_camara_vision.sh` | Camara + vision + visor |
| 3 | `scripts/3_laberinto.sh` (o `--cuenta-giros`) | El recorrido. Teclas: **g** arranca, **Y** = OK, ESPACIO o p frena, q sale |

Robot en el centro de la celda de partida, derecho, mirando hacia adentro. Cerrar el panel de ruedas antes de la 3.
Si algo se desconecta: esperar ~10 s con el LED parpadeando (la ESP32 reconecta sola); no relanzar todo.

## Mas detalle

- `pc/ruedas/LABERINTO_CUBO.md`: que hace cada modo, ajustes y pruebas.
- `pc/ruedas/FLECHAS_Y_CUBO.md` (si existe en esta version): el modo que decide por flechas.
- `pc/ruedas/README.md`: el panel de ruedas y la calibracion.
- `scripts/pruebas.sh`: las pruebas de la logica (sin robot).
