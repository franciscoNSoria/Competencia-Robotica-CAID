"""
Misión en el robot REAL (no Gazebo): el mismo explorador, con los tópicos del firmware real.

Levanta cámara + visión, odom_ticks (odometría por flancos de /wheel_ticks), sonar_bridge
(separa los 3 sonares de /wheel_state), calibracion_robot (manda al firmware la calibración
medida y activa la guardia por sonar), carga_brazo contra la ESP32 real del brazo y el
explorador, todo con los nombres reales en vez de los /sim/... de Gazebo.

  ros2 launch caid_navigation mision_real.launch.py
  ros2 launch caid_navigation mision_real.launch.py usar_camara:=false   # sin cámara ni visión

Firmware real: firmware/banco/robot_4ruedas/main.c (sonares, guardia y /robot_cmd ya integrados
al 05/11, docs/guias o el md "hardware-estado-y-calibraciones" que trae el equipo). Lo que sigue
pendiente: el servo de la cámara (pan) no lo escucha ninguna ESP32 todavía, por eso
usar_servo_camara y reconocer_al_inicio quedan en false acá (no bloquea: hay timeout, pero no
escanea flechas laterales).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('colores', default_value='[verde,rojo,azul]'),
        DeclareLaunchArgument('tam_celda', default_value='0.40'),
        DeclareLaunchArgument('umbral_pared', default_value='0.25'),
        DeclareLaunchArgument('usar_camara', default_value='true'),
        DeclareLaunchArgument('fuente_camara', default_value='http://192.168.1.1:81/stream',
                              description="'0' = webcam de la PC | URL de la ESP32-CAM"),
        DeclareLaunchArgument('usar_servo_camara', default_value='false'),
        DeclareLaunchArgument('reconocer_al_inicio', default_value='false'),
        # Calibración medida el 3/11 (hardware-estado-y-calibraciones.md, sección 6). Mantener
        # igual en caid_bringup/odom_ticks.py y caid_bringup/calibracion_robot.py.
        DeclareLaunchArgument('diametro_m', default_value='0.0645'),
        DeclareLaunchArgument('trocha_m', default_value='0.291'),
        DeclareLaunchArgument('d_stop_m', default_value='0.20'),
        DeclareLaunchArgument('d_lento_m', default_value='0.40'),
    ]
    con_camara = IfCondition(LaunchConfiguration('usar_camara'))

    camara = Node(package='caid_vision', executable='camara_node', output='screen',
                  parameters=[{'fuente': LaunchConfiguration('fuente_camara')}],
                  condition=con_camara)
    vision = Node(package='caid_vision', executable='vision_node', output='screen',
                  parameters=[{'imagen_topic': '/camera/image_raw'}], condition=con_camara)

    calibracion = Node(package='caid_bringup', executable='calibracion_robot', output='screen',
                       parameters=[{'diametro_m': LaunchConfiguration('diametro_m'),
                                    'trocha_m': LaunchConfiguration('trocha_m'),
                                    'd_stop_m': LaunchConfiguration('d_stop_m'),
                                    'd_lento_m': LaunchConfiguration('d_lento_m')}])
    odom = Node(package='caid_bringup', executable='odom_ticks', output='screen',
                parameters=[{'separacion': LaunchConfiguration('trocha_m')}])
    sonares = Node(package='caid_bringup', executable='sonar_bridge', output='screen')
    brazo = Node(package='caid_manipulation', executable='carga_brazo', output='screen',
                 parameters=[{'simulacion': False}])
    explorador = Node(
        package='caid_navigation', executable='explorador', output='screen',
        parameters=[{
            'cmd_vel_topic': '/cmd_vel',
            'odom_topic': '/odom',
            'frente_topic': '/distancia/frente',
            'izquierda_topic': '/distancia/izquierda',
            'derecha_topic': '/distancia/derecha',
            'detecciones_topic': '/vision/detecciones',
            'camara_pan_topic': '/camera_pan/cmd',
            'joint_states_topic': '/joint_states',
            'usar_servo_camara': LaunchConfiguration('usar_servo_camara'),
            'reconocer_al_inicio': LaunchConfiguration('reconocer_al_inicio'),
            'canastos_con_camara': True,
            'tam_celda': LaunchConfiguration('tam_celda'),
            'umbral_pared': LaunchConfiguration('umbral_pared'),
            'colores': LaunchConfiguration('colores'),
        }])

    return LaunchDescription(
        args + [camara, vision, calibracion, odom, sonares, brazo, explorador])
