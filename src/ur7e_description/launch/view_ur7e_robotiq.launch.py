"""Display the combined UR7e and Robotiq 2F-85 model in RViz."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start robot state publishing, joint sliders, and RViz."""
    model = PathJoinSubstitution(
        [
            FindPackageShare("ur7e_description"),
            "urdf",
            "ur7e_robotiq_2f_85.urdf.xacro",
        ]
    )
    ur_view_launch = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "launch", "view_ur.launch.xml"]
    )

    view_robot = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(ur_view_launch),
        launch_arguments={
            "description_file": model,
            "ur_type": "ur7e",
        }.items(),
    )
    return LaunchDescription([view_robot])
