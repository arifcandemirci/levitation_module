
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('levitation_demo'))
    gazebo_ros_share = Path(get_package_share_directory('gazebo_ros'))

    world = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    pause =LaunchConfiguration('pause')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=str(pkg_share / 'worlds' / 'levitation_demo_marker_test.world'))

    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='-0.510715')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.350583')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='1.90')
    pause_arg = DeclareLaunchArgument('pause', default_value='true')
    gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=[
            str(pkg_share),
            TextSubstitution(text=':'),
            EnvironmentVariable('GAZEBO_RESOURCE_PATH', default_value=''),
        ],
    )

    robot_description = (pkg_share / 'urdf' / 'levitation_robot.urdf').read_text()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_ros_share / 'launch' / 'gazebo.launch.py')),
        launch_arguments={'world': world, 'pause': 'true'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'levitation_robot',
            '-topic', 'robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
        ],
    )

    return LaunchDescription([
        world_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        pause_arg,
        gazebo_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])
