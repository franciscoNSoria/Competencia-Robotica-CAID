# Workspace de ROS 2 (Jazzy)

Acá viven los paquetes ROS 2 del proyecto, en `src/`. Se compila con colcon **desde esta carpeta**:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y   # instala dependencias declaradas en package.xml
colcon build --symlink-install                     # --symlink-install: los cambios en Python/launch no requieren recompilar
source install/setup.bash                          # en cada terminal nueva, después de compilar
```

`build/`, `install/` y `log/` los genera colcon y están ignorados por git.

## Paquetes previstos (cada uno se crea en su tarea del roadmap)

| Paquete | Qué contiene |
|---|---|
| `caid_description` | Descripción del robot en xacro/URDF y mallas |
| `caid_gazebo` | Mundos de Gazebo (laberintos), generador de laberintos y emulador de firmware |
| `caid_bringup` | Launch files, configuración del puente con Gazebo y parámetros |
| `caid_mcu_msgs` | Mensajes que intercambia la ESP32 (chicos y de tamaño fijo) |
| `caid_interfaces` | Mensajes, servicios y acciones propios de la PC |
| `caid_hw_bridge` | Puente de video de la ESP32-CAM y monitor del robot |
| `caid_perception` | Detección de flechas, cubos y depósitos |
| `caid_navigation` | Odometría, seguridad, recorrido del laberinto y mapa |
| `caid_manipulation` | Cinemática y control del brazo, agarre |
| `caid_mission` | Máquina de estados de la misión |
