# Estado de v1-panel-camara: que anda, que no, y con que evidencia

> Completar a mano, y SOLO con lo que se probo. Estados: **M** probado en el robot, **S** solo en simulador/tests, **?** sin probar.

## Resultado del dia de la competencia

- Rama/commit que se corrio: ______________________
- Que paso (llego al cubo, volvio, choco, se desconecto): ______________________

## Que probo esta version (panel de camara, 2026-11-27)

Alcance: esta version agrega el panel web de la camara y la deteccion de ruta con rumbo. La parte de ruedas, laberinto
y firmware de movimiento NO se probo aca (siguen en `?` mas abajo).

| Pieza | Estado | Evidencia |
|---|---|---|
| Firmware ESP32-CAM (repo) en placa de 8 MB, red `Personal-443` | M | 15,0 fps a 640x480 (3 corridas de 10 s), 0 cuadros corruptos, pausa maxima 117 ms. Sin tope: 24,5 fps en VGA |
| Panel web: video, fps, ms por cuadro, senal | M | 15 fps recibidos / 15 procesados, 15 ms por cuadro, con la camara real |
| Panel: espejar y voltear en vivo | M | `vflip` 0 -> 1 -> 0 comprobado en `/status` de la camara |
| Panel: guardar configuracion en `colores.yaml` | M | El usuario lo uso desde el navegador y se escribio la seccion `camara:` |
| Panel: guardar foto y rotar imagen | S | Probado sin camara con un cuadro inyectado (3 archivos); con camara falta |
| Deteccion de ruta y rumbo (`ruta.py`) | S | 5 tests con escenas dibujadas (recto, curva der/izq, pared, imagen negra) |
| Ruta en el laberinto real | ? | Sin probar: el unico cuadro real era un cuarto con la camara inclinada y la zona "piso" se derramo a una pared del mismo color |
| Deteccion de cubos y flechas con el panel | ? | El detector es el del repo; no habia cubos ni flechas delante durante las pruebas |
| `/vision/ruta` en ROS y uso en el explorador | ? | No existe todavia: el rumbo solo se ve en el panel |

Para que la red no meta pausas: apagar el ahorro de energia del Wi-Fi de la notebook
(`nmcli connection modify "<red>" 802-11-wireless.powersave 2`). Con eso el ping a la camara bajo de 41 a 7 ms.
La camara entrega el video a un solo cliente a la vez.

## Que hace cada pieza

| Pieza | Estado | Evidencia |
|---|---|---|
| Firmware de movimiento (PI por rueda, sonares, freno local, brazo) | ? | |
| Panel de ruedas y calibracion | ? | |
| Laberinto por celdas con sonares (Tremaux) | ? | |
| Enderezado en linea recta y giro de 90 grados | ? | |
| Vision: flechas y cubos | ? | |
| Seguidor de flechas | ? | |
| Vuelta (mapa / cuenta-giros) | ? | |

## Commits de la rama maze que dicen haber validado algo (pistas, no pruebas)

- `a823df4` 2026-11-19: Merge pull request #88 from franciscoNSoria/feature/CAM-vision-final
- `49d5d2f` 2026-11-05: feat(vision): flecha fija a 30 cm, /vision/proximo_giro, servo en GPIO13 y lector de sonares
- `7e20b0e` 2026-11-05: feat(vision): flechas del reglamento nuevo, validación y calibración de cámara

## Fallas conocidas (de las pruebas del 3 y 4/11, no de esta version)

- El giro por encoders no repite en la baldosa lustrada (trocha efectiva ~291 mm +-7 %, 4 de 10 giros salieron muy cortos).
- Desvio lateral de ~5 cm por metro sin signo fijo: sin enderezado con paredes no se mantiene el rumbo.
- El sonar lateral perdio 32 % de las lecturas a 5,5 cm; el frontal no vio una tela en diagonal (sillon).
- Tras reiniciar el agente rapido, la ESP32 tarda ~6-7 s en reconectar (esperar con el LED parpadeando).
- La ESP32 arranca con la calibracion de fabrica si nadie se la manda (ya corregido en los firmwares nuevos).

## ESP32-S3 (placa nueva)

Este firmware es para el ESP32 clasico de 38 pines. En la ESP32-S3 hay que portarlo: el LEDC solo tiene modo low speed
(el codigo usa HIGH_SPEED), los GPIO 35-37 los usa la PSRAM octal, y los pines cambian. No flashear tal cual.
