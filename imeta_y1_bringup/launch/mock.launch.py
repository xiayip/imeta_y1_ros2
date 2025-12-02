from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    imeta_y1_launch_file = PathJoinSubstitution(
        [FindPackageShare('imeta_y1_bringup'), 'launch', 'bringup.launch.py']
    )

    launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imeta_y1_launch_file),
        launch_arguments={'use_mock_hardware': 'true'}.items()
    )

    return LaunchDescription([launch])