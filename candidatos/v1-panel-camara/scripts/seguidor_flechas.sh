#!/usr/bin/env bash
# Seguidor de flechas reactivo (opcional --volver). Necesita 1_agente y 2_camara_vision corriendo.
set -euo pipefail
AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$AQUI/entorno.sh"
cd "$CAID_RAIZ/pc/ruedas"
python3 seguidor_real.py "$@"
