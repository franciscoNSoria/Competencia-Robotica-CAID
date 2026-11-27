#!/usr/bin/env bash
# El panel de las 4 ruedas (calibracion, giros, odometria). Cerrar antes cualquier otro programa que mande /cmd_vel.
set -euo pipefail
AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$AQUI/entorno.sh"
cd "$CAID_RAIZ/pc/ruedas"
python3 ruedas_panel.py "$@"
