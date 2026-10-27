"""
Misión de prueba en Gazebo: entrar, buscar el cubo verde, cargarlo y volver a salir.

Levanta la simulación con el robot en la entrada del laberinto, agrega el cubo verde, el detector
simulado, el explorador y RViz con el mapa que va armando.

Por defecto usa laberinto_chico.world (11x11 celdas, 2 islas). Para el mundo grande:
  ros2 launch caid_navigation mision_sim.launch.py world:=maze_2.world \
      x:=4.5 y:=-4.05 cubo_x:=-4.05 cubo_y:=4.05
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

CUBO_SDF = """<sdf version="1.9"><model name="cubo_verde"><static>true</static>
<link name="link"><visual name="visual"><geometry><box><size>0.04 0.04 0.04</size></box>
</geometry><material><ambient>0 0.8 0 1</ambient><diffuse>0 0.8 0 1</diffuse></material>
</visual></link></model></sdf>"""


def generate_launch_description():
    arg = {n: LaunchConfiguration(n) for n in ('world', 'x', 'y', 'yaw', 'cubo_x', 'cubo_y')}

    def flotante(n):
        return ParameterValue(arg[n], value_type=float)

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('caid_bringup'), 'launch', 'sim.launch.py'])),
        launch_arguments={
            'world': PathJoinSubstitution(
                [FindPackageShare('caid_gazebo'), 'worlds', arg['world']]),
            'x': arg['x'], 'y': arg['y'], 'yaw': arg['yaw'],
            'rviz': 'false', 'gui': LaunchConfiguration('gui')}.items())

    cubo = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'cubo_verde', '-string', CUBO_SDF,
                   '-x', arg['cubo_x'], '-y', arg['cubo_y'], '-z', '0.02'])

    detector = Node(
        package='caid_navigation', executable='cubo_simulado', output='screen',
        parameters=[{'cubo_x': flotante('cubo_x'), 'cubo_y': flotante('cubo_y'),
                     'robot_x': flotante('x'), 'robot_y': flotante('y'),
                     'robot_yaw': flotante('yaw'), 'use_sim_time': True}])

    explorador = Node(
        package='caid_navigation', executable='explorador', output='screen',
        parameters=[{'use_sim_time': True}])

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('caid_navigation'), 'rviz', 'mapa.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='laberinto_chico.world'),
        # Entrada única de laberinto_chico (borde este), mirando hacia adentro.
        DeclareLaunchArgument('x', default_value='2.25'),
        DeclareLaunchArgument('y', default_value='-0.9'),
        DeclareLaunchArgument('yaw', default_value='3.14159'),
        # Cubo en la esquina de arriba a la izquierda: la otra punta desde la entrada.
        DeclareLaunchArgument('cubo_x', default_value='-1.8'),
        DeclareLaunchArgument('cubo_y', default_value='1.8'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        sim,
        rviz,
        # El cubo y la misión arrancan cuando el servidor de Gazebo ya está arriba.
        TimerAction(period=5.0, actions=[cubo, detector, explorador]),
    ])
