# Copyright 2022 ICube Laboratory, University of Strasbourg
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

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
import os
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument('use_sim', default_value='true', description='Flag to use simulation'),
        DeclareLaunchArgument(
            'gz_args',
            default_value='-r -v 1 empty.sdf',
            description='Arguments for gz_sim'
        ),
    ]

    # Get URDF
    arm_description_path = get_package_share_directory('arm_description')
    urdf_file = os.path.join(arm_description_path, 'urdf', 'arm.urdf.xacro')
    
    # Get YAML
    arm_gazebo_path = get_package_share_directory('arm_control')
    controller_config_file = os.path.join(arm_gazebo_path, 'config', 'arm_control.yaml')
        
    robot_description_arm = {"robot_description":Command(['xacro ', urdf_file])}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description_arm,
                    {"use_sim_time": True},
            ],
        remappings=[('/robot_description', '/robot_description')]
    )
    
    gazebo_ignition = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic','robot_description', '-entity', 'arm'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_sim')),
    )

    return LaunchDescription(
        
        declared_arguments +
        [
            robot_state_pub_node,
            gazebo_ignition,
            spawn_entity,
        ]
    )
