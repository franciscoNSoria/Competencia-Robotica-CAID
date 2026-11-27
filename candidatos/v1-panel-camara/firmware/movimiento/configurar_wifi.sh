#!/usr/bin/env bash
# Copia la configuracion de red (Wi-Fi, clave, IP del agente, puerto) y los ajustes de micro-ROS de tu ejemplo que YA anda
# a sdkconfig.secreto, que NO va a git. No imprime la clave. Se corre una vez (o cuando cambie la red).
set -euo pipefail
AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EJEMPLO="${MICROROS_EJEMPLO:-$HOME/caid/pruebas-banco/micro_ros_espidf_component/examples/int32_publisher}"
if [ ! -f "$EJEMPLO/sdkconfig" ]; then
	echo "No encuentro $EJEMPLO/sdkconfig. Pasale la ruta:  MICROROS_EJEMPLO=/ruta/al/int32_publisher $0"
	echo "o corre 'idf.py menuconfig' aca (micro-ROS Settings: WLAN, red, clave, IP del agente, puerto 8888)."
	exit 1
fi
grep -E '^CONFIG_(ESP_WIFI_|MICRO_ROS_|FREERTOS_HZ|ESP_DEFAULT_CPU_FREQ|COMPILER_OPTIMIZATION)' "$EJEMPLO/sdkconfig" >"$AQUI/sdkconfig.secreto"
chmod 600 "$AQUI/sdkconfig.secreto"
rm -f "$AQUI/sdkconfig"
echo "OK: $(grep -c . "$AQUI/sdkconfig.secreto") ajustes copiados a sdkconfig.secreto (ignorado por git). Se borro sdkconfig para que se regenere."
echo "Red configurada: $(grep -E '^CONFIG_ESP_WIFI_SSID=' "$AQUI/sdkconfig.secreto" | cut -d= -f1)  IP del agente: $(grep -E '^CONFIG_MICRO_ROS_AGENT_IP=' "$AQUI/sdkconfig.secreto" | cut -d= -f2)"
