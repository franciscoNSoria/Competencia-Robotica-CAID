"""
Misión de prueba en Gazebo: entrar, buscar el cubo verde, cargarlo y volver a salir.

Levanta la simulación con el robot en la entrada del laberinto, el detector de cubo simulado,
el explorador y RViz con el mapa que va armando.

  ros2 launch caid_navigation mision_sim.launch.py                 # laberinto_chico (rápido)
  ros2 launch caid_navigation mision_sim.launch.py escenario:=e4   # E4 isla, de DESC-04

Escenarios: chico, maze_2 y e1 a e6. e1 a e6 son las copias simples de los de DESC-04
(docs/pruebas/escenarios.md), sin flechas laterales ni zonas prohibidas; se arman con
tools/sim/simplificar_laberintos.py. Ahí el cubo verde ya está en el mundo, a la altura de la
cámara: la visión lo detecta y las flechas verdes guían la exploración. En chico y maze_2 el cubo
se agrega en el piso en cubo_x, cubo_y y lo detecta cubo_simulado.
La carga es la action /cargar, simulada por carga_simulada.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Pose de aparición (mundo de Gazebo), tamaño de celda y umbral de pared de cada escenario.
# limites: celdas [x_min, x_max, y_min, y_max] en el marco de la odometría (inicio = 0, 0,
# x hacia donde mira el robot); hacen falta con áreas abiertas sin paredes.
_E = {'tam': 0.40, 'umbral': 0.25, 'cubo': None}   # mapas simples: celda de 0,40 m
ESCENARIOS = {
    'chico': {'world': 'laberinto_chico.world', 'x': 2.25, 'y': -0.9, 'yaw': 3.14159,
              'tam': 0.45, 'umbral': 0.30, 'cubo': (-1.8, 1.8), 'colores': ['verde']},
    'maze_2': {'world': 'maze_2.world', 'x': 4.5, 'y': -4.05, 'yaw': 3.14159,
               'tam': 0.45, 'umbral': 0.30, 'cubo': (-4.05, 4.05), 'colores': ['verde']},
    'e1': {**_E, 'world': 'e1_serpentina_simple.world', 'x': 1.0, 'y': 2.2, 'yaw': 0.0},
    'e2': {**_E, 'world': 'e2_bifurcaciones_simple.world', 'x': 1.0, 'y': 2.2, 'yaw': 0.0},
    'e3': {**_E, 'world': 'e3_obstaculos_simple.world', 'x': 1.0, 'y': 2.2, 'yaw': 0.0},
    'e4': {**_E, 'world': 'e4_isla_simple.world', 'x': 1.0, 'y': 2.2, 'yaw': 0.0},
    'e5': {**_E, 'world': 'e5_competencia_simple.world', 'x': 1.0, 'y': 2.6, 'yaw': 0.0},
    'e6': {**_E, 'world': 'e6_salida_abierta_simple.world', 'x': 3.0, 'y': 2.2, 'yaw': 0.0},
}

CUBO_SDF = """<sdf version="1.9"><model name="cubo_verde"><static>true</static>
<link name="link"><visual name="visual"><geometry><box><size>0.04 0.04 0.04</size></box>
</geometry><material><ambient>0 0.8 0 1</ambient><diffuse>0 0.8 0 1</diffuse></material>
</visual></link></model></sdf>"""


def armar(context):
    nombre = LaunchConfiguration('escenario').perform(context)
    if nombre not in ESCENARIOS:
        raise RuntimeError(f'escenario desconocido: {nombre} (opciones: {", ".join(ESCENARIOS)})')
    e = ESCENARIOS[nombre]
    mundo = os.path.join(get_package_share_directory('caid_gazebo'), 'worlds', e['world'])
    cubo = e['cubo']
    if cubo is not None:
        cubo = (float(LaunchConfiguration('cubo_x').perform(context) or cubo[0]),
                float(LaunchConfiguration('cubo_y').perform(context) or cubo[1]))

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('caid_bringup'), 'launch', 'sim.launch.py'])),
        launch_arguments={'world': mundo, 'x': str(e['x']), 'y': str(e['y']),
                          'yaw': str(e['yaw']), 'rviz': 'false',
                          'gui': LaunchConfiguration('gui')}.items())

    mision = [
        Node(package='caid_navigation', executable='carga_simulada', output='screen'),
        Node(package='caid_vision', executable='vision_node', output='screen',
             parameters=[{'imagen_topic': '/sim/camera/image_raw', 'use_sim_time': True}]),
        Node(package='caid_navigation', executable='explorador', output='screen',
             parameters=[{'tam_celda': e['tam'], 'umbral_pared': e['umbral'],
                          'limites': e.get('limites', [0, 0, 0, 0]),
                          'colores': e.get('colores', ['verde', 'rojo', 'azul']),
                          'use_sim_time': True}]),
    ]
    if cubo:
        # Cubo en el piso: la cámara no lo ve, lo "detecta" cubo_simulado al entrar a su celda.
        mision += [
            Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-name', 'cubo_verde', '-string', CUBO_SDF,
                            '-x', str(cubo[0]), '-y', str(cubo[1]), '-z', '0.02']),
            Node(package='caid_navigation', executable='cubo_simulado', output='screen',
                 parameters=[{'cubo_x': cubo[0], 'cubo_y': cubo[1],
                              'robot_x': e['x'], 'robot_y': e['y'], 'robot_yaw': e['yaw'],
                              'tam_celda': e['tam'], 'use_sim_time': True}]),
        ]

    # El cubo y la misión arrancan cuando el servidor de Gazebo ya está arriba.
    return [sim, TimerAction(period=5.0, actions=mision)]


def generate_launch_description():
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('caid_navigation'), 'rviz', 'mapa.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('escenario', default_value='chico',
                              description=' | '.join(ESCENARIOS)),
        # Solo para chico y maze_2: dónde poner el cubo (vacío = el del escenario).
        DeclareLaunchArgument('cubo_x', default_value=''),
        DeclareLaunchArgument('cubo_y', default_value=''),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=armar),
        rviz,
    ])
