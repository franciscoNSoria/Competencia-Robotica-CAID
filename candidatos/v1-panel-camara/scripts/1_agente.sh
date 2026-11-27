#!/usr/bin/env bash
# Terminal 1: agente micro-ROS (antes de encender el robot). Mata uno viejo si quedo vivo.
pkill -f micro-ros-agent 2>/dev/null || true
docker run -it --rm --net=host docker.io/microros/micro-ros-agent:jazzy udp4 --port 8888
