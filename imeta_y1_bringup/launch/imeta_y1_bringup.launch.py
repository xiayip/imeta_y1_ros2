# Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Source of this file are templates in
# [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
#
# Author: Dr. Denis
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. \
            Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo_classic",
            default_value="false",
            description="Start robot with gazebo classic simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value="false",
            description="Start robot with gazebo simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_isaac_sim",
            default_value="false",
            description="Start robot with isaac sim simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "real_world",
            default_value="false",
            description="Start robot with real world",
        )
    )
    # Initialize Arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    sim_gazebo_classic = LaunchConfiguration("sim_gazebo_classic")
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    sim_isaac_sim = LaunchConfiguration("sim_isaac_sim")
    real_world = LaunchConfiguration("real_world")

    ld = LaunchDescription()
    # Add declared arguments individually
    for arg in declared_arguments:
        ld.add_action(arg)
    # Include the controllers launch file
    controllers_launch_file = PathJoinSubstitution(
        [FindPackageShare('imeta_y1_bringup'), 'launch', 'include', 'controllers.launch.py']
    )
    controllers_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controllers_launch_file),
        launch_arguments={
            'use_mock_hardware': use_mock_hardware,
            'mock_sensor_commands': mock_sensor_commands,
            'sim_gazebo_classic': sim_gazebo_classic,
            'sim_gazebo': sim_gazebo,
            'sim_isaac_sim': sim_isaac_sim,
            'real_world': real_world
        }.items()
    )
    ld.add_action(controllers_include)
    
    return ld
