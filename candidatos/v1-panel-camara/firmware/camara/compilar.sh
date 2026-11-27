#!/usr/bin/env bash
# Compila (y graba) la ESP32-CAM. Terminal LIMPIA de ESP-IDF. Antes: copiar main/secrets.example.h como main/secrets.h y completarlo
# (red y clave; ese archivo NO va a git).   ./compilar.sh [grabar [puerto]]
set -euo pipefail
AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ -z "${AMENT_PREFIX_PATH:-}" ] || { echo "Esta terminal tiene ROS cargado: usa una terminal limpia de ESP-IDF."; exit 1; }
command -v idf.py >/dev/null || { echo "No esta idf.py. Antes:  . ~/esp/esp-idf-v5.5/export.sh"; exit 1; }
[ -f "$AQUI/main/secrets.h" ] || { echo "Falta main/secrets.h (copia secrets.example.h y completalo; no se sube a git)."; exit 1; }
cd "$AQUI"
idf.py build
if [ "${1:-}" = "grabar" ]; then
	echo ">>> IO0 a GND solo para grabar. Despues sacalo y apreta RESET. Sin monitor (imprime la clave)."
	idf.py -p "${2:-/dev/ttyUSB0}" -b 115200 flash
fi
