"""Launch RGB-D red, green and blue cube detection."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Load detector parameters and start the vision node."""
    package_share = Path(get_package_share_directory('ur7e_vision'))
    return LaunchDescription(
        [
            Node(
                package='ur7e_vision',
                executable='color_cube_detector',
                name='color_cube_detector',
                output='screen',
                parameters=[
                    str(package_share / 'config' / 'color_cube_detector.yaml')
                ],
            )
        ]
    )
