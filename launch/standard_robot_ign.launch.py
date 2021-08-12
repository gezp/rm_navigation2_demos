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
    robot_base =Node(package='rmua19_ignition_simulator', executable='rmua19_robot_base2',
        parameters=[
                {"world_name": "default"},
                {"robot_name": robot_name},
        ],output='screen') 
    robot_ign_bridge = Node(package='ros_ign_bridge',executable='parameter_bridge',
        arguments=["/world/default/model/%s/link/front_rplidar_a2/sensor/front_rplidar_a2/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan"%(robot_name),
            "/%s/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry"%(robot_name)
        ],
        remappings=[
            ("/world/default/model/%s/link/front_rplidar_a2/sensor/front_rplidar_a2/scan"%(robot_name),"/scan"),
            ("/%s/odometry"%(robot_name),"/odom"),
        ],
        output='screen'
    )
    ld.add_action(robot_base)
    ld.add_action(robot_ign_bridge)
    return ld
