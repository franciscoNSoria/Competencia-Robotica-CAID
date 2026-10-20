from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    RegisterEventHandler, Shutdown, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration('world')
    arm = LaunchConfiguration('arm')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    gui = LaunchConfiguration('gui')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('caid_description'), 'urdf', 'caid_bot.urdf.xacro'])
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' arm:=', arm, ' sim:=true']),
        value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': ['-s -r ', world]}.items())

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': PathJoinSubstitution(
            [FindPackageShare('caid_bringup'), 'config', 'sim', 'bridge.yaml'])}])

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=[
            '-topic', 'robot_description', '-name', 'caid_bot',
            '-x', x, '-y', y, '-z', '0.02', '-Y', yaw])

    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g'], output='screen',
        condition=IfCondition(gui), on_exit=Shutdown())

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution(
            [FindPackageShare('caid_gazebo'), 'worlds', 'maze_2.world'])),
        DeclareLaunchArgument('arm', default_value='fixed'),
        DeclareLaunchArgument('x', default_value='-0.45'),
        DeclareLaunchArgument('y', default_value='2.7'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),
        DeclareLaunchArgument('gui', default_value='true'),
        gz_sim,
        bridge,
        rsp,
        # Esperamos a que el servidor este listo antes de crear el robot
        TimerAction(period=5.0, actions=[spawn]),
        # La GUI arranca después de crear el robot: así lo dibuja completo.
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[gz_gui])),
    ])
