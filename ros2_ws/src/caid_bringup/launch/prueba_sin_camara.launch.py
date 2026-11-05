"""
Bringup para la prueba de banco sin cámara (tools/pruebas/prueba_autonomia_sonar_brazo_comando_
sin_camara/). Levanta SOLO calibracion_robot, odom_ticks y sonar_bridge: ni cámara, ni visión,
ni carga_brazo (el agarre es a mano con tools/brazo/brazo_sliders.py).

  ros2 launch caid_bringup prueba_sin_camara.launch.py

El nodo que explora (explorar_sin_camara.py) se corre aparte, en otra terminal, como
tools/brazo/brazo_sliders.py:

  python3 tools/pruebas/prueba_autonomia_sonar_brazo_comando_sin_camara/explorar_sin_camara.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        # Calibración medida el 3/11 (hardware-estado-y-calibraciones.md, sección 6). Mantener
        # igual en explorar_sin_camara.py si se cambia acá.
        DeclareLaunchArgument('diametro_m', default_value='0.0645'),
        DeclareLaunchArgument('trocha_m', default_value='0.291'),
        # Mas chicos que en mision_real.launch.py a proposito: el explorador de esta prueba se
        # acerca a proposito hasta 3 cm del cubo (distancia_parada) y su propio freno de "pared
        # inesperada" en tick_avanzar() es a 7 cm. Con los 0.20/0.40 de mision_real, la guardia
        # del firmware topaba la velocidad a 0 bastante antes de que esta logica reaccionara, y
        # el robot quedaba quieto (el explorador seguia mandando /cmd_vel sin saber que el
        # firmware lo estaba ignorando). Confirmado en el banco: /wheel_state[29] (estado de la
        # guardia) daba 3 (frenada) con el sonar frontal en 0.18 m, bien antes de cualquiera de
        # los dos umbrales de Python.
        DeclareLaunchArgument('d_stop_m', default_value='0.02'),
        DeclareLaunchArgument('d_lento_m', default_value='0.08'),
    ]
    calibracion = Node(package='caid_bringup', executable='calibracion_robot', output='screen',
                       parameters=[{'diametro_m': LaunchConfiguration('diametro_m'),
                                    'trocha_m': LaunchConfiguration('trocha_m'),
                                    'd_stop_m': LaunchConfiguration('d_stop_m'),
                                    'd_lento_m': LaunchConfiguration('d_lento_m')}])
    odom = Node(package='caid_bringup', executable='odom_ticks', output='screen',
                parameters=[{'separacion': LaunchConfiguration('trocha_m')}])
    sonares = Node(package='caid_bringup', executable='sonar_bridge', output='screen')
    return LaunchDescription(args + [calibracion, odom, sonares])
