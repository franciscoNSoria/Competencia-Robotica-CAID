#!/usr/bin/env bash
# Compila (y graba) el firmware de movimiento. En una terminal LIMPIA de ESP-IDF, sin ROS cargado.
#   ./compilar.sh                      solo compila
#   ./compilar.sh grabar [puerto]      compila y graba (por defecto /dev/ttyUSB0), sin monitor
# Ojo: la ESP32 FUERA del shield al grabar (nunca USB y 5 V del shield a la vez). El monitor imprime la clave del Wi-Fi: no se usa.
# El primer build de un proyecto nuevo reconstruye la libreria de micro-ROS: tarda varios minutos.
set -euo pipefail
AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -n "${AMENT_PREFIX_PATH:-}" ]; then
	echo "Esta terminal tiene ROS cargado. Usa una terminal limpia de ESP-IDF:"
	echo "  env -i HOME=\"\$HOME\" USER=\"\$USER\" TERM=\"\$TERM\" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin bash --noprofile --norc"
	exit 1
fi
command -v idf.py >/dev/null || { echo "No esta idf.py. Antes:  . ~/esp/esp-idf-v5.5/export.sh"; exit 1; }
[ -f "$AQUI/sdkconfig.secreto" ] || { echo "Falta sdkconfig.secreto: corre primero ./configurar_wifi.sh"; exit 1; }
cd "$AQUI"
if [ ! -f sdkconfig ]; then
	idf.py set-target "${CAID_TARGET:-esp32}"      # solo en un proyecto nuevo: set-target borra el sdkconfig
fi
idf.py build
if [ "${1:-}" = "grabar" ]; then
	idf.py -p "${2:-/dev/ttyUSB0}" -b 115200 flash
	echo ">>> Grabada. LED parpadeando = buscando al agente; apagado = conectada."
fi
