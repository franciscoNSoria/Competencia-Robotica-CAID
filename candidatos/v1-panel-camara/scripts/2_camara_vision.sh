#!/usr/bin/env bash
# Terminal 2: camara + vision (+ visor). Los valores se cambian en config/lanzamiento.env, no aca.
set -euo pipefail
AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$AQUI/entorno.sh"
source "$CAID_RAIZ/config/lanzamiento.env"
ros2 launch "$CAID_RAIZ/launch/robot.launch.xml" fuente:="http://${IP_CAMARA}:81/stream" esp32_falsa:=false visor:="${VISOR}" \
	calibracion:="${CALIBRACION}" distancia_flecha:="${DIST_FLECHA}" distancia_fija:="${DIST_FIJA}"
