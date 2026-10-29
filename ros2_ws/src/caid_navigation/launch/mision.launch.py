"""
Misión completa en Gazebo: simulación + visión + navegador + brazo simulado.

    ros2 launch caid_navigation mision.launch.py laberinto:=e2_bifurcaciones

`laberinto` es el nombre de un escenario de caid_gazebo/mazes (e1_serpentina, e2_bifurcaciones,
e3_obstaculos, e4_isla, e5_competencia, e6_salida_abierta). La pose de partida y el significado
de los colores de las flechas salen del mismo YAML con gen_maze.py, así no hay que copiarlos a mano.
El navegador NO lee el mapa: arranca sin saber nada del laberinto.
"""
import importlib.util
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, \
    TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NOMBRES = {'R': 'rojo', 'G': 'verde', 'B': 'azul'}


def _gen_maze(share_gazebo):
    ruta = os.path.join(share_gazebo, 'scripts', 'gen_maze.py')
    spec = importlib.util.spec_from_file_location('gen_maze', ruta)
    modulo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(modulo)
    return modulo


def _armar(context):
    nombre = LaunchConfiguration('laberinto').perform(context)
    share_gazebo = get_package_share_directory('caid_gazebo')
    gm = _gen_maze(share_gazebo)
    lab = gm.cargar(os.path.join(share_gazebo, 'mazes', nombre + '.yaml'))
    (r, c), = lab.buscar('S')
    x, y = lab.centro(r, c)
    yaw = gm.YAW[lab.inicio_dir]
    significado = {color: [NOMBRES[s] for s in cubos]
                   for color, cubos in lab.significado_colores.items()}

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('caid_bringup'), 'launch', 'sim.launch.py')),
        launch_arguments={
            'world': os.path.join(share_gazebo, 'worlds', nombre + '.world'),
            'x': str(x), 'y': str(y), 'yaw': str(yaw), 'rviz': 'false',
            'gui': LaunchConfiguration('gui')}.items())
    vision = Node(
        package='caid_vision', executable='vision_node', output='screen',
        parameters=[{'imagen_topic': '/sim/camera/image_raw', 'use_sim_time': True}])
    navegador = Node(
        package='caid_navigation', executable='navegador', output='screen',
        parameters=[{'significado_colores': json.dumps(significado), 'use_sim_time': True}])
    brazo = Node(
        package='caid_navigation', executable='manipulacion_simulada', output='screen',
        parameters=[{'use_sim_time': True}])
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('caid_navigation'), 'rviz',
                                      'mapa.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))
    # La misión arranca cuando el robot ya está en el mundo (sim.launch lo crea a los 5 s).
    return [sim, rviz, TimerAction(period=8.0, actions=[vision, brazo, navegador])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('laberinto', default_value='e2_bifurcaciones'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_armar),
    ])
