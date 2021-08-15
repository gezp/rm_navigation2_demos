#  Copyright (c) 2020 robomaster-oss, All rights reserved.
#
#  This program is free software: you can redistribute it and/or modify it
#  under the terms of the MIT License, See the MIT License for more details.
#
#  You should have received a copy of the MIT License along with this program.
#  If not, see <https://opensource.org/licenses/MIT/>.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    pkg_rmua19_ignition_simulator = get_package_share_directory('rmua19_ignition_simulator')
    # Gazebo launch
    world_sdf_path = os.path.join(get_package_share_directory('rm_navigation2_demos'),
        'resource','worlds', 'rmua19_robot_world.sdf')
    ign_config_path = os.path.join(pkg_rmua19_ignition_simulator, 'ign', 'gui.config')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py'),
        ),
        launch_arguments={
            'ign_args': world_sdf_path + ' -v 2 --gui-config ' + ign_config_path,
        }.items()
    )
    ld.add_action(gazebo)
    # robot base for each robot
    robot_name = "standard_robot_red1"
    robot_base =Node(package='rmua19_ignition_simulator', executable='rmua19_robot_base',
        parameters=[
                {"world_name": "default"},
                {"robot_name": robot_name},
        ],output='screen') 
    ld.add_action(robot_base)
    # static tf
    tf1 = Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            name = "static_transform_publisher1",
            arguments=['0', '0', '0.0758', '0.0', '0.0', '0.0', 'standard_robot_red1/footprint', 'standard_robot_red1/chassis'],
        )
    tf2 = Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            name = "static_transform_publisher2",
            arguments=['0.155', '0', '0.1', '0.0', '0.0', '0.0', 'standard_robot_red1/chassis', 'standard_robot_red1/front_rplidar_a2/front_rplidar_a2'],
        )
    ld.add_action(tf1)
    ld.add_action(tf2)
    return ld
