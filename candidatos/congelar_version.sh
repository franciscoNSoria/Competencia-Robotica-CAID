#!/usr/bin/env bash
# congelar_version.sh: congela una VERSION CANDIDATA en una carpeta autonoma (candidatos/NOMBRE/), copiando solo lo necesario
# desde commits/ramas de git. La carpeta no depende de la rama en la que estes parado ni de ~/Descargas.
#
# Uso (desde cualquier lugar del repo):
#   bash candidatos/congelar_version.sh v1
#   bash candidatos/congelar_version.sh v2 maze=origin/feature/laberinto-cuenta-giros vision=origin/integracion
#
# De donde sale cada cosa (cada una puede ser una rama, una etiqueta o un commit):
#   maze=   tools/ruedas (panel, laberinto por celdas con sonares, seguidor de flechas) y herramientas de PC
#   fw=     firmware de la ESP32 de movimiento (por defecto = maze)
#   vision= paquetes de ROS 2 (camara, vision, launch)
#   cam=    firmware de la ESP32-CAM (por defecto = vision)
#   ref=    pone las cuatro al mismo tiempo (una rama que ya tenga todo junto)
# Opciones: --sin-fetch (no hace git fetch), --reemplazar (pisa una carpeta existente; normalmente NO se pisa nada),
#   --completa (deja tambien pruebas, simulador, laberintos de simulacion y herramientas sueltas; por defecto la carpeta queda MINIMA:
#   solo lo que se necesita para compilar los firmwares y correr el laberinto con camara)
#
# Una version congelada no se edita: si algo mejora, se congela otra (v2, v3...) y se anota en candidatos/VERSIONES.md cual
# anda mejor en que.
set -euo pipefail

MAZE="origin/feature/laberinto-cuenta-giros"
FW=""
VISION="origin/integracion"
CAM=""
FETCH=1
REEMPLAZAR=0
COMPLETA=0
NOMBRE=""

aviso() { echo "  AVISO: $*" >&2; }
fallar() { echo "ERROR: $*" >&2; exit 1; }

for arg in "$@"; do
	case "$arg" in
	maze=*) MAZE="${arg#maze=}" ;;
	fw=*) FW="${arg#fw=}" ;;
	vision=*) VISION="${arg#vision=}" ;;
	cam=*) CAM="${arg#cam=}" ;;
	ref=*) MAZE="${arg#ref=}"; FW="$MAZE"; VISION="$MAZE"; CAM="$MAZE" ;;
	--sin-fetch) FETCH=0 ;;
	--reemplazar) REEMPLAZAR=1 ;;
	--completa) COMPLETA=1 ;;
	-h | --help) sed -n '2,26p' "$0"; exit 0 ;;
	-*) fallar "opcion desconocida: $arg" ;;
	*) [ -z "$NOMBRE" ] && NOMBRE="$arg" || fallar "sobra el argumento: $arg" ;;
	esac
done
[ -n "$NOMBRE" ] || fallar "falta el nombre de la version. Ejemplo: bash candidatos/congelar_version.sh v1"
[[ "$NOMBRE" =~ ^[A-Za-z0-9._-]+$ ]] || fallar "el nombre solo puede tener letras, numeros, punto, guion y guion bajo"
FW="${FW:-$MAZE}"
CAM="${CAM:-$VISION}"

RAIZ="$(git rev-parse --show-toplevel)" || fallar "esto no es un repositorio git"
cd "$RAIZ"
if [ "$FETCH" = 1 ]; then
	git fetch origin --quiet 2>/dev/null || aviso "no pude hacer git fetch (sin red?): uso lo que ya hay"
fi
for r in "$MAZE" "$FW" "$VISION" "$CAM"; do
	git rev-parse --verify --quiet "$r^{commit}" >/dev/null || fallar "no existe la rama/commit: $r"
done

DEST="candidatos/$NOMBRE"
if [ -e "$DEST" ]; then
	[ "$REEMPLAZAR" = 1 ] || fallar "$DEST ya existe. Una version congelada no se pisa: usa otro nombre (o --reemplazar si es un error tuyo)."
	rm -rf "$DEST"
fi
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
mkdir -p "$DEST"

sha() { git rev-parse --short=9 "$1^{commit}"; }

# extraer REF RUTA DESTINO: copia un archivo o carpeta versionados en REF
extraer() {
	local ref="$1" ruta="$2" dest="$3"
	if ! git cat-file -e "$ref:$ruta" 2>/dev/null; then
		aviso "no esta $ruta en $ref"
		return 1
	fi
	rm -rf "$TMP/x"
	mkdir -p "$TMP/x" "$(dirname "$dest")"
	git archive "$ref" -- "$ruta" | tar -x -C "$TMP/x"
	rm -rf "$dest"
	mv "$TMP/x/$ruta" "$dest"
}

echo "== Congelando $NOMBRE"
echo "   maze   = $MAZE ($(sha "$MAZE"))"
echo "   fw     = $FW ($(sha "$FW"))"
echo "   vision = $VISION ($(sha "$VISION"))"
echo "   cam    = $CAM ($(sha "$CAM"))"

# ---------------------------------------------------------------------------------------------------------------------
# 1. PC: panel de ruedas, laberinto y herramientas
# ---------------------------------------------------------------------------------------------------------------------
echo "-- PC"
extraer "$MAZE" tools/ruedas "$DEST/pc/ruedas" || true
# Hermanos que tools/ruedas importa por ruta relativa (../tremaux, ../sim, ../../ros2_ws/src/caid_*): se respeta la misma distribucion
extraer "$MAZE" tools/tremaux "$DEST/pc/tremaux" || true
rm -f "$DEST/pc/tremaux/test_tremaux.py"   # necesita el simulador de tools/sim, que el robot real no usa
extraer "$MAZE" tools/sim/laberintos "$DEST/pc/sim/laberintos" || true
extraer "$MAZE" ros2_ws/src/caid_gazebo/mazes "$DEST/ros2_ws/src/caid_gazebo/mazes" || true
extraer "$MAZE" ros2_ws/src/caid_navigation/caid_navigation "$DEST/ros2_ws/src/caid_navigation/caid_navigation" || true
for f in tools/vigilar_esp32.py tools/brazo/brazo_sliders.py tools/pruebas/prueba.py; do
	extraer "$MAZE" "$f" "$DEST/pc/herramientas/$(basename "$f")" || true
done

# ---------------------------------------------------------------------------------------------------------------------
# 2. ROS 2: solo lo que corre en el robot real (camara + vision + launch)
# ---------------------------------------------------------------------------------------------------------------------
echo "-- ROS 2"
for p in caid_vision caid_bringup caid_interfaces; do
	extraer "$VISION" "ros2_ws/src/$p" "$DEST/ros2_ws/src/$p" || true
done
extraer "$VISION" ros2_ws/src/caid_bringup/launch/robot.launch.xml "$DEST/launch/robot.launch.xml" || true
extraer "$VISION" ros2_ws/src/caid_vision/config "$DEST/config/vision" || true

# ---------------------------------------------------------------------------------------------------------------------
# 3. Firmware de movimiento: proyecto ESP-IDF autonomo (main.c + los .h locales que use, recursivamente)
# ---------------------------------------------------------------------------------------------------------------------
echo "-- Firmware de movimiento"
FWDIR="$DEST/firmware/movimiento"
mkdir -p "$FWDIR/main"
extraer "$FW" firmware/banco/robot_4ruedas/main.c "$FWDIR/main/main.c" || fallar "no hay firmware/banco/robot_4ruedas/main.c en $FW"
BUSCAR_EN=(firmware/pruebas/comun firmware/brazo_sliders firmware/banco/robot_4ruedas)
pendientes=("main/main.c")
vistos=" "
COPIADOS=()
while [ "${#pendientes[@]}" -gt 0 ]; do
	archivo="${pendientes[0]}"
	pendientes=("${pendientes[@]:1}")
	[ -f "$FWDIR/$archivo" ] || continue
	for h in $(grep -oE '^#include "[^"/]+\.h"' "$FWDIR/$archivo" | sed -E 's/#include "(.*)"/\1/' | sort -u); do
		case "$vistos" in *" $h "*) continue ;; esac
		vistos="$vistos$h "
		for d in "${BUSCAR_EN[@]}"; do
			if git cat-file -e "$FW:$d/$h" 2>/dev/null; then
				extraer "$FW" "$d/$h" "$FWDIR/main/$h"
				COPIADOS+=("$h")
				pendientes+=("main/$h")
				break
			fi
		done
	done
done
echo "   headers locales copiados junto a main.c: ${COPIADOS[*]:-ninguno}"

cat >"$FWDIR/CMakeLists.txt" <<'EOF'
# Proyecto ESP-IDF AUTONOMO del firmware de movimiento (ruedas, sonares, freno local y brazo en una sola ESP32).
# Se compila desde esta carpeta: no hace falta copiar nada al ejemplo int32_publisher.
cmake_minimum_required(VERSION 3.16)

# El componente micro-ROS no vive en el repo: es el que ya clonaste en ~/caid/pruebas-banco (rama jazzy).
# Si lo tenes en otro lado:  export MICROROS_COMPONENT=/ruta/a/micro_ros_espidf_component
if(DEFINED ENV{MICROROS_COMPONENT})
  set(MICROROS_DIR "$ENV{MICROROS_COMPONENT}")
else()
  set(MICROROS_DIR "$ENV{HOME}/caid/pruebas-banco/micro_ros_espidf_component")
endif()
if(NOT EXISTS "${MICROROS_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "No encuentro el componente micro-ROS en ${MICROROS_DIR}. Exporta MICROROS_COMPONENT con su ruta.")
endif()
set(EXTRA_COMPONENT_DIRS "${MICROROS_DIR}")

# sdkconfig.defaults va en el repo; sdkconfig.secreto (red Wi-Fi, clave, IP del agente) NO: lo genera configurar_wifi.sh
set(SDKCONFIG_DEFAULTS "sdkconfig.defaults")
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.secreto")
  list(APPEND SDKCONFIG_DEFAULTS "sdkconfig.secreto")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(robot_movimiento)
EOF

cat >"$FWDIR/main/CMakeLists.txt" <<'EOF'
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "")
EOF

cat >"$FWDIR/sdkconfig.defaults" <<'EOF'
# Valores base (los mismos del ejemplo int32_publisher de micro-ROS). Sin secretos.
# Si cambias algo aca: borra sdkconfig y vuelve a compilar para que se aplique.
CONFIG_ESP_MAIN_TASK_STACK_SIZE=3000
CONFIG_ESP_TASK_WDT=n
EOF

cat >"$FWDIR/.gitignore" <<'EOF'
build/
sdkconfig
sdkconfig.old
sdkconfig.secreto
managed_components/
EOF

cat >"$FWDIR/configurar_wifi.sh" <<'EOF'
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
EOF

cat >"$FWDIR/compilar.sh" <<'EOF'
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
EOF
chmod +x "$FWDIR/compilar.sh" "$FWDIR/configurar_wifi.sh"

cat >"$FWDIR/LEEME.md" <<'EOF'
# Firmware de movimiento (ESP32): proyecto autonomo

Ruedas (PI por rueda), encoders, 3 sonares con freno local, brazo, micro-ROS por Wi-Fi. `main/` tiene el `main.c` y los `.h` locales
que usa; nada se copia a otro proyecto.

```bash
./configurar_wifi.sh            # una vez: toma red, clave e IP del agente de tu ejemplo que ya anda (no imprime la clave)
./compilar.sh                   # compila (el primer build tarda: reconstruye micro-ROS)
./compilar.sh grabar            # compila y graba en /dev/ttyUSB0
```

- Terminal LIMPIA de ESP-IDF (sin ROS), ESP32 fuera del shield al grabar, sin `monitor`.
- La placa se elige con `CAID_TARGET` (por defecto `esp32`; para la ESP32-S3 el firmware hay que portarlo: ver `ESTADO.md`).
- Si no compila, el camino de siempre sigue funcionando: copiar `main/*` sobre `main/` del ejemplo `int32_publisher`.
EOF

# ---------------------------------------------------------------------------------------------------------------------
# 4. Firmware de la camara (ya es un proyecto autonomo)
# ---------------------------------------------------------------------------------------------------------------------
echo "-- Firmware de la camara"
extraer "$CAM" firmware/esp32_cam "$DEST/firmware/camara" || true
if [ -d "$DEST/firmware/camara" ]; then
	cat >"$DEST/firmware/camara/compilar.sh" <<'EOF'
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
EOF
	chmod +x "$DEST/firmware/camara/compilar.sh"
fi

# ---------------------------------------------------------------------------------------------------------------------
# 5. Lo de tu maquina que hace falta el dia de la competencia (no esta en git)
# ---------------------------------------------------------------------------------------------------------------------
echo "-- Config de esta maquina"
mkdir -p "$DEST/config/mi_maquina"
for f in "$HOME/caid/config/calibracion.json" "$HOME/caid/env_ros.sh"; do
	if [ -f "$f" ]; then
		cp "$f" "$DEST/config/mi_maquina/"
		echo "   copiado $f"
	else
		aviso "no existe $f (si lo usas, copialo a $DEST/config/mi_maquina/)"
	fi
done

# ---------------------------------------------------------------------------------------------------------------------
# 6. Scripts de la PC: un comando por terminal, siempre los mismos
# ---------------------------------------------------------------------------------------------------------------------
mkdir -p "$DEST/scripts"
cat >"$DEST/scripts/entorno.sh" <<'EOF'
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
EOF
cat >"$DEST/scripts/1_agente.sh" <<'EOF'
#!/usr/bin/env bash
# Terminal 1: agente micro-ROS (antes de encender el robot). Mata uno viejo si quedo vivo.
pkill -f micro-ros-agent 2>/dev/null || true
docker run -it --rm --net=host docker.io/microros/micro-ros-agent:jazzy udp4 --port 8888
EOF
cat >"$DEST/scripts/0_construir.sh" <<'EOF'
#!/usr/bin/env bash
# Una vez (y despues de cambiar codigo de ros2_ws): compila los paquetes de ROS 2 de esta carpeta. Sin ESP-IDF cargado.
set -euo pipefail
[ -z "${IDF_PATH:-}" ] || { echo "Esta terminal tiene ESP-IDF cargado: usa otra."; exit 1; }
AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
cd "$AQUI/../ros2_ws"
colcon build --symlink-install --packages-select caid_vision
EOF
cat >"$DEST/scripts/2_camara_vision.sh" <<'EOF'
#!/usr/bin/env bash
# Terminal 2: camara + vision (+ visor). Los valores se cambian en config/lanzamiento.env, no aca.
set -euo pipefail
AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$AQUI/entorno.sh"
source "$CAID_RAIZ/config/lanzamiento.env"
ros2 launch "$CAID_RAIZ/launch/robot.launch.xml" fuente:="http://${IP_CAMARA}:81/stream" esp32_falsa:=false visor:="${VISOR}" \
	calibracion:="${CALIBRACION}" distancia_flecha:="${DIST_FLECHA}" distancia_fija:="${DIST_FIJA}"
EOF
cat >"$DEST/config/lanzamiento.env" <<'EOF'
# Valores del lanzamiento de camara + vision. Son los de tools/ruedas/LABERINTO_CUBO.md.
IP_CAMARA=172.16.0.57          # IP de la ESP32-CAM en la red de la competencia (cambiala si es otra)
VISOR=true
CALIBRACION="$CAID_RAIZ/ros2_ws/src/caid_vision/config/calibracion_esp32cam.yaml"
DIST_FLECHA=0.8
DIST_FIJA=0.60
EOF
cat >"$DEST/scripts/3_laberinto.sh" <<'EOF'
#!/usr/bin/env bash
# Terminal 3: el laberinto con cubo (CERRAR el panel de ruedas: no pueden estar los dos).
#   ./3_laberinto.sh                  primer cubo, vuelta por el mapa (Tremaux)
#   ./3_laberinto.sh --cuenta-giros   primer cubo, vuelta a ciegas repitiendo los giros al reves
set -euo pipefail
AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$AQUI/entorno.sh"
cd "$CAID_RAIZ/pc/ruedas"
python3 laberinto_real.py "$@"
EOF
cat >"$DEST/scripts/panel_ruedas.sh" <<'EOF'
#!/usr/bin/env bash
# El panel de las 4 ruedas (calibracion, giros, odometria). Cerrar antes cualquier otro programa que mande /cmd_vel.
set -euo pipefail
AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$AQUI/entorno.sh"
cd "$CAID_RAIZ/pc/ruedas"
python3 ruedas_panel.py "$@"
EOF
cat >"$DEST/scripts/seguidor_flechas.sh" <<'EOF'
#!/usr/bin/env bash
# Seguidor de flechas reactivo (opcional --volver). Necesita 1_agente y 2_camara_vision corriendo.
set -euo pipefail
AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$AQUI/entorno.sh"
cd "$CAID_RAIZ/pc/ruedas"
python3 seguidor_real.py "$@"
EOF
cat >"$DEST/scripts/pruebas.sh" <<'EOF'
#!/usr/bin/env bash
# Pruebas de la logica de PC (sin robot, sin ROS, sin pantalla). Si pasan, la carpeta esta tal cual se congelo.
set -euo pipefail
AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$AQUI/../pc/ruedas"
python3 -m pytest -q --ignore=test_ruedas_panel.py .
EOF
chmod +x "$DEST"/scripts/*.sh

# ---------------------------------------------------------------------------------------------------------------------
# 7. Documentos de la version
# ---------------------------------------------------------------------------------------------------------------------
{
	echo "version: $NOMBRE"
	echo "congelada: $(date '+%Y-%m-%d %H:%M')"
	echo "por: $(git config user.name 2>/dev/null || echo '?')"
	echo
	for k in maze fw vision cam; do
		case "$k" in maze) r="$MAZE" ;; fw) r="$FW" ;; vision) r="$VISION" ;; cam) r="$CAM" ;; esac
		echo "$k = $r"
		echo "     $(git log -1 --format='%H%n     %ad %an: %s' --date=short "$r")"
	done
	echo
	echo "Las tres lineas de abajo son la verdad de esta carpeta: si un dia hay dudas, se compara contra estos commits."
} >"$DEST/ORIGEN.txt"

{
	echo "# Estado de $NOMBRE: que anda, que no, y con que evidencia"
	echo
	echo "> Completar a mano, y SOLO con lo que se probo. Estados: **M** probado en el robot, **S** solo en simulador/tests, **?** sin probar."
	echo
	echo "## Resultado del dia de la competencia"
	echo
	echo "- Rama/commit que se corrio: ______________________"
	echo "- Que paso (llego al cubo, volvio, choco, se desconecto): ______________________"
	echo
	echo "## Que hace cada pieza"
	echo
	echo "| Pieza | Estado | Evidencia |"
	echo "|---|---|---|"
	echo "| Firmware de movimiento (PI por rueda, sonares, freno local, brazo) | ? | |"
	echo "| Panel de ruedas y calibracion | ? | |"
	echo "| Laberinto por celdas con sonares (Tremaux) | ? | |"
	echo "| Enderezado en linea recta y giro de 90 grados | ? | |"
	echo "| Vision: flechas y cubos | ? | |"
	echo "| Seguidor de flechas | ? | |"
	echo "| Vuelta (mapa / cuenta-giros) | ? | |"
	echo
	echo "## Commits de la rama maze que dicen haber validado algo (pistas, no pruebas)"
	echo
	git log "$MAZE" --regexp-ignore-case --grep='valid' --format='- `%h` %ad: %s' --date=short | head -n 15
	echo
	echo "## Fallas conocidas (de las pruebas del 3 y 4/11, no de esta version)"
	echo
	echo "- El giro por encoders no repite en la baldosa lustrada (trocha efectiva ~291 mm +-7 %, 4 de 10 giros salieron muy cortos)."
	echo "- Desvio lateral de ~5 cm por metro sin signo fijo: sin enderezado con paredes no se mantiene el rumbo."
	echo "- El sonar lateral perdio 32 % de las lecturas a 5,5 cm; el frontal no vio una tela en diagonal (sillon)."
	echo "- Tras reiniciar el agente rapido, la ESP32 tarda ~6-7 s en reconectar (esperar con el LED parpadeando)."
	echo "- La ESP32 arranca con la calibracion de fabrica si nadie se la manda (ya corregido en los firmwares nuevos)."
	echo
	echo "## ESP32-S3 (placa nueva)"
	echo
	echo "Este firmware es para el ESP32 clasico de 38 pines. En la ESP32-S3 hay que portarlo: el LEDC solo tiene modo low speed"
	echo "(el codigo usa HIGH_SPEED), los GPIO 35-37 los usa la PSRAM octal, y los pines cambian. No flashear tal cual."
} >"$DEST/ESTADO.md"

cat >"$DEST/COMO_CORRER.md" <<'EOF'
# Como se corre (robot real, laberinto con cubo)

Cada comando en su propia terminal, siempre desde esta carpeta. Las terminales de ESP-IDF y las de ROS NO se mezclan.

## Una sola vez

1. `scripts/0_construir.sh`: compila los paquetes de ROS 2 de esta carpeta (terminal sin ESP-IDF).
2. Firmware: `firmware/movimiento/` (`./configurar_wifi.sh`, `./compilar.sh grabar`) y `firmware/camara/` (`./compilar.sh grabar`).
   Terminal limpia de ESP-IDF, placa fuera del shield, sin `monitor`.
3. Revisar `config/lanzamiento.env` (IP de la camara, distancias de las flechas).

## El dia

| Terminal | Comando | Que es |
|---|---|---|
| 1 | `scripts/1_agente.sh` | Agente micro-ROS. Antes de encender el robot |
| 2 | `scripts/2_camara_vision.sh` | Camara + vision + visor |
| 3 | `scripts/3_laberinto.sh` (o `--cuenta-giros`) | El recorrido. Teclas: **g** arranca, **Y** = OK, ESPACIO o p frena, q sale |

Robot en el centro de la celda de partida, derecho, mirando hacia adentro. Cerrar el panel de ruedas antes de la 3.
Si algo se desconecta: esperar ~10 s con el LED parpadeando (la ESP32 reconecta sola); no relanzar todo.

## Mas detalle

- `pc/ruedas/LABERINTO_CUBO.md`: que hace cada modo, ajustes y pruebas.
- `pc/ruedas/FLECHAS_Y_CUBO.md` (si existe en esta version): el modo que decide por flechas.
- `pc/ruedas/README.md`: el panel de ruedas y la calibracion.
- `scripts/pruebas.sh`: las pruebas de la logica (sin robot).
EOF

cat >"$DEST/LEEME.md" <<EOF
# $NOMBRE (congelada)

Carpeta autonoma: tiene todo lo necesario para compilar los firmwares y correr el robot real, sin depender de la rama en la que
estes parado ni de ~/Descargas. **No se edita**: si algo mejora, se congela otra version y se anota en \`../VERSIONES.md\`.

- Que anda y que no: [ESTADO.md](ESTADO.md)
- Como correrla: [COMO_CORRER.md](COMO_CORRER.md)
- De donde salio cada archivo: [ORIGEN.txt](ORIGEN.txt)
- Que no se cambio desde que se congelo: \`sha256sum -c SHA256SUMS\` (desde esta carpeta)

\`\`\`
pc/ruedas/        panel, laberinto por celdas con sonares, seguidor de flechas (Python)
launch/           robot.launch.xml: camara + vision (+ visor)
ros2_ws/src/      caid_vision (camara, vision, calibracion), caid_navigation/mapa.py (el Tremaux que usa el laberinto)
firmware/         movimiento/ (ESP32) y camara/ (ESP32-CAM), cada uno un proyecto ESP-IDF autonomo
config/           lanzamiento.env, mi_maquina/ (calibracion.json y env_ros.sh de quien congelo)
scripts/          un script por terminal
\`\`\`
EOF

# ---------------------------------------------------------------------------------------------------------------------
# 8. Verificaciones y huellas
# ---------------------------------------------------------------------------------------------------------------------
echo "-- Verificando"
fallos=0
while IFS= read -r -d '' f; do
	python3 -m py_compile "$f" 2>"$TMP/err" || { aviso "no compila $f: $(tail -n 1 "$TMP/err")"; fallos=$((fallos + 1)); }
done < <(find "$DEST" -name '*.py' -print0)
find "$DEST" -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null || true
echo "   Python: $fallos archivos con error de sintaxis"
if command -v python3 >/dev/null && python3 -c 'import pytest' 2>/dev/null && [ -d "$DEST/pc/ruedas" ]; then
	if (cd "$DEST/pc/ruedas" && timeout 120 env DISPLAY= python3 -m pytest -q -x -p no:cacheprovider --ignore=test_ruedas_panel.py . </dev/null >"$TMP/pytest.txt" 2>&1); then
		echo "   Pruebas de pc/ruedas: $(tail -n 1 "$TMP/pytest.txt")"
	else
		aviso "fallan o no terminan (120 s) las pruebas de pc/ruedas; no es grave para congelar. Ultimas lineas: $(tail -n 5 "$TMP/pytest.txt" | tr '\n' ' ')"
	fi
	find "$DEST" -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null || true
	rm -rf "$DEST/pc/ruedas/.pytest_cache"
fi
# Poda: por defecto la carpeta queda minima (las pruebas ya corrieron arriba, sobre la copia completa)
if [ "$COMPLETA" = 0 ]; then
	echo "-- Podando a lo minimo (--completa lo evita)"
	rm -rf "$DEST/pc/tremaux" "$DEST/pc/sim" "$DEST/pc/herramientas" "$DEST/config/vision" \
		"$DEST/ros2_ws/src/caid_gazebo" "$DEST/ros2_ws/src/caid_interfaces" "$DEST/ros2_ws/src/caid_bringup" \
		"$DEST/ros2_ws/src/caid_vision/test" "$DEST/firmware/camara/.vscode" "$DEST/scripts/pruebas.sh"
	rm -f "$DEST"/pc/ruedas/test_*.py "$DEST/pc/ruedas/sim_laberinto.py"
	# caid_navigation: laberinto_ruedas.py solo usa mapa.py (el Tremaux), que no importa nada del resto
	if [ -f "$DEST/ros2_ws/src/caid_navigation/caid_navigation/mapa.py" ]; then
		mv "$DEST/ros2_ws/src/caid_navigation/caid_navigation/mapa.py" "$TMP/mapa_nav.py"
		rm -rf "$DEST/ros2_ws/src/caid_navigation"
		mkdir -p "$DEST/ros2_ws/src/caid_navigation/caid_navigation"
		mv "$TMP/mapa_nav.py" "$DEST/ros2_ws/src/caid_navigation/caid_navigation/mapa.py"
	fi
	find "$DEST" -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null || true
	# comprobar que lo que quedo importa solo lo que quedo
	if (cd "$DEST/pc/ruedas" && python3 -I -c 'import sys; sys.path.insert(0, "."); import laberinto_ruedas, seguidor, ruedas_logica' >"$TMP/imp.txt" 2>&1); then
		echo "   Imports de la version minima: OK"
	else
		aviso "la version minima no importa limpia: $(tail -n 2 "$TMP/imp.txt" | tr '\n' ' ')"
	fi
	find "$DEST" -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null || true
fi
echo "   Archivos en $DEST: $(find "$DEST" -type f | wc -l)"
(cd "$DEST" && find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum >SHA256SUMS)

# ---------------------------------------------------------------------------------------------------------------------
# 9. Indice general: candidatos/VERSIONES.md y README.md (se crean una vez; despues solo se agrega una fila)
# ---------------------------------------------------------------------------------------------------------------------
if [ ! -f candidatos/VERSIONES.md ]; then
	cat >candidatos/VERSIONES.md <<'EOF'
# Versiones candidatas

Cada fila es una carpeta congelada (`candidatos/NOMBRE/`). Se completa a mano la columna "Anda mejor en": que situaciones resuelve
mejor esa version que las otras. Una version que se reemplaza no se borra: se aclara por que.

| Version | Fecha | Origen (maze / vision) | Anda mejor en | Estado | Notas |
|---|---|---|---|---|---|
EOF
fi
printf '| [%s](%s/) | %s | `%s` %s / `%s` %s | (completar) | (ver ESTADO.md) | |\n' "$NOMBRE" "$NOMBRE" "$(date +%Y-%m-%d)" "${MAZE#origin/}" "$(sha "$MAZE")" "${VISION#origin/}" "$(sha "$VISION")" >>candidatos/VERSIONES.md
if [ ! -f candidatos/README.md ]; then
	cat >candidatos/README.md <<'EOF'
# Candidatos: lo que mejor anda, congelado

El repo tiene muchas ramas y el dia de la competencia no se sabia cual era la que andaba. Esta carpeta es la respuesta:
**copias congeladas y autonomas** de lo que mejor funciona, con todo lo necesario para compilar y correr.

- `VERSIONES.md`: indice. Cual es cual, de donde salio y en que anda mejor.
- `NOMBRE/`: una version. Abrir `LEEME.md`, despues `ESTADO.md` y `COMO_CORRER.md`.
- `congelar_version.sh`: crea una version nueva desde ramas/commits (`bash candidatos/congelar_version.sh v2`).

Reglas: (1) una version congelada no se edita; (2) lo que cambia va a una version nueva; (3) el dia de la competencia solo se corre
algo de esta carpeta, con los scripts de su `scripts/`, nunca archivos sueltos de ~/Descargas.
EOF
fi
if [ -f "$0" ] && [ "$(cd "$(dirname "$0")" && pwd)" != "$RAIZ/candidatos" ]; then
	cp "$0" candidatos/congelar_version.sh 2>/dev/null || true
fi

echo
echo "LISTO: $DEST"
echo "  siguiente: completar $DEST/ESTADO.md y la fila de candidatos/VERSIONES.md; despues git add candidatos y commit."
