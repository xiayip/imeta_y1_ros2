from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch_file = PathJoinSubstitution(
        [FindPackageShare('imeta_y1_bringup'), 'launch', 'bringup.launch.py']
    )

    launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={'real_world': 'true'}.items()
    )

    return LaunchDescription([launch])