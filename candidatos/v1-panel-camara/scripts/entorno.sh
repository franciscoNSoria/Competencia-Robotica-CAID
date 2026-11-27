#!/usr/bin/env bash
# Se usa con:  source scripts/entorno.sh     (ROS 2 + dominio de este robot + el workspace de esta carpeta)
_AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export CAID_RAIZ="$(cd "$_AQUI/.." && pwd)"
export CAID_DOMINIO="${CAID_DOMINIO:-71}"
if [ -f "$CAID_RAIZ/config/mi_maquina/env_ros.sh" ]; then
	source "$CAID_RAIZ/config/mi_maquina/env_ros.sh"
elif [ -f "$HOME/caid/env_ros.sh" ]; then
	source "$HOME/caid/env_ros.sh"
else
	source /opt/ros/jazzy/setup.bash
	export ROS_DOMAIN_ID="$CAID_DOMINIO"
fi
[ -f "$CAID_RAIZ/ros2_ws/install/setup.bash" ] && source "$CAID_RAIZ/ros2_ws/install/setup.bash"
true
