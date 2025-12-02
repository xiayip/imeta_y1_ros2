from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    mh3_launch_file = PathJoinSubstitution(
        [FindPackageShare('imeta_y1_bringup'), 'launch', 'bringup.launch.py']
    )

    launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mh3_launch_file),
        launch_arguments={'sim_mujoco':'true'}.items()
    )
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("imeta_y1_description"),
                    "urdf/",
                    "imeta_y1.urdf.xacro",
                ]
            ),
            " ",
            "sim_mujoco:=true",
        ]
    )
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    movebase_controllers = PathJoinSubstitution(
        [FindPackageShare("imeta_y1_bringup"), "config", "controllers.yaml",]
    )
    mujoco_mjcf_path = PathJoinSubstitution( [
            FindPackageShare("imeta_y1_description"),
            "mjcf/",
            "imeta_y1.xml",
        ]
    )
    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            movebase_controllers,
            {'mujoco_model_path':mujoco_mjcf_path}
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ]
    )

    return LaunchDescription([node_mujoco_ros2_control] + [launch])