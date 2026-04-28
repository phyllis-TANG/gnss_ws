"""Launch the UBX replay node with parameters loaded from config/replay.yaml."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = Path(get_package_share_directory('ubx_parser'))
    default_config = pkg_share / 'config' / 'replay.yaml'

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=str(default_config),
        description='Path to a YAML parameter file for the replay node.',
    )
    ubx_file_arg = DeclareLaunchArgument(
        'ubx_file',
        default_value='',
        description='Override the ubx_file parameter (path to a .ubx capture).',
    )

    replay_node = Node(
        package='ubx_parser',
        executable='replay_node',
        name='ubx_replay_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            {'ubx_file': LaunchConfiguration('ubx_file')},
        ],
    )

    return LaunchDescription([config_arg, ubx_file_arg, replay_node])
