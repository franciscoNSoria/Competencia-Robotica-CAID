#!/usr/bin/env bash
# Terminal 3: el laberinto con cubo (CERRAR el panel de ruedas: no pueden estar los dos).
#   ./3_laberinto.sh                  primer cubo, vuelta por el mapa (Tremaux)
#   ./3_laberinto.sh --cuenta-giros   primer cubo, vuelta a ciegas repitiendo los giros al reves
set -euo pipefail
AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$AQUI/entorno.sh"
cd "$CAID_RAIZ/pc/ruedas"
python3 laberinto_real.py "$@"
