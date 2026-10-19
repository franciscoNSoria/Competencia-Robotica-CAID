from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('caid_description')
    xacro_file = PathJoinSubstitution([share, 'urdf', 'caid_bot.urdf.xacro'])
    rviz_config = PathJoinSubstitution([share, 'rviz', 'caid.rviz'])

    # xacro convierte el .xacro en URDF (texto); robot_state_publisher lo publica y calcula TF.
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        # Ventana con sliders para mover las articulaciones a mano
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config]),
    ])