#!/usr/bin/env bash
# Graba una prueba de firmware/pruebas en la ESP32 (o en la ESP32-CAM), con micro-ROS por Wi-Fi.
#
# Uso (en la terminal LIMPIA de ESP-IDF, la de docs/guias/micro-ros.md, sin ROS cargado):
#   ./compilar.sh <prueba> [puerto] [monitor]
#   ./compilar.sh 10_sonar_frontal
#   ./compilar.sh 34_ruedas_4 /dev/ttyUSB0
#   ./compilar.sh 50_servo_pan_cam /dev/ttyUSB0 monitor
#
# Copia <prueba>.c como main/main.c (y los .h de comun/) en el ejemplo int32_publisher del componente micro-ROS, compila y
# graba. La red Wi-Fi y la IP del agente son las que ya estan en idf.py menuconfig de ese ejemplo (micro-ROS Settings).
# La primera vez guarda el main.c original del ejemplo como main/main.c.original.
#
# Ojo: el monitor muestra la clave del Wi-Fi al conectar. No pegar ese log en ningun lado.
set -euo pipefail

AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EJEMPLO="${MICROROS_EJEMPLO:-$HOME/caid/pruebas-banco/micro_ros_espidf_component/examples/int32_publisher}"
PRUEBA="${1:-}"
PUERTO="${2:-/dev/ttyUSB0}"
MONITOR="${3:-}"

if [[ -z "$PRUEBA" ]]; then
	echo "Uso: $0 <prueba> [puerto] [monitor]"
	echo "Pruebas:"
	ls "$AQUI"/*.c | xargs -n1 basename | sed 's/\.c$//; s/^/  /'
	exit 1
fi
PRUEBA="${PRUEBA%.c}"
ARCHIVO="$AQUI/$PRUEBA.c"
if [[ ! -f "$ARCHIVO" ]]; then
	echo "No existe $ARCHIVO"
	exit 1
fi
if [[ ! -d "$EJEMPLO/main" ]]; then
	echo "No encuentro el ejemplo de micro-ROS en $EJEMPLO"
	echo "Si esta en otro lado: export MICROROS_EJEMPLO=/ruta/a/micro_ros_espidf_component/examples/int32_publisher"
	exit 1
fi
if [[ -n "${AMENT_PREFIX_PATH:-}" ]]; then
	echo "Esta terminal tiene ROS cargado (AMENT_PREFIX_PATH no esta vacio). Usar la terminal limpia de ESP-IDF."
	exit 1
fi
if ! command -v idf.py >/dev/null; then
	echo "No esta idf.py. Antes: . ~/esp/esp-idf-v5.5/export.sh"
	exit 1
fi

if [[ ! -f "$EJEMPLO/main/main.c.original" ]]; then
	cp "$EJEMPLO/main/main.c" "$EJEMPLO/main/main.c.original"
fi
cp "$ARCHIVO" "$EJEMPLO/main/main.c"
cp "$AQUI"/comun/*.h "$EJEMPLO/main/"
echo ">>> $PRUEBA copiada a $EJEMPLO/main/main.c"

cd "$EJEMPLO"
idf.py build
if [[ "$MONITOR" == "monitor" ]]; then
	idf.py -p "$PUERTO" -b 115200 flash monitor
else
	idf.py -p "$PUERTO" -b 115200 flash
	echo ">>> Grabada. En la PC (terminal con ROS): python3 tools/pruebas/prueba.py ${PRUEBA}"
	echo ">>> Si vas a conectar el 5 V del shield al pin 5V de la ESP32, primero desconecta el USB (nunca los dos a la vez)."
fi
