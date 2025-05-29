#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription , ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():

    drone_pkg = get_package_share_directory('sjtu_drone_bringup')
    my_pkg = get_package_share_directory('followme_drone')

    # 1. Declare the launch argument for your custom world
    declare_world_path = DeclareLaunchArgument(
        'world_path',
        default_value=os.path.join(
            my_pkg,
            'worlds', 'laboratory.world'
        ),
        description='Path to the custom world file'
    )
    
    # 2. Include the submodule's launch file, passing your world as an argument
    include_sjtu_drone = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                drone_pkg,  # Or 'sjtu_drone' if that's the package
                'launch',
                'sjtu_drone_bringup.launch.py'
            )
        ]),
        launch_arguments={'world': LaunchConfiguration('world_path')}.items()
    )

    takeoff_cmd= ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', 'simple_drone/takeoff', 'std_msgs/msg/Empty', '{}','--once'],
            shell=True,
            output="screen"
        )
    
    # takeoff_cmd= ExecuteProcess(
    #         cmd=['ros2', 'topic', 'pub', '/simple_drone/cmd_vel', 'geometry_msgs/msg/Twist', '{linear: {x: 0.0, y: 0.0, z: 0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
    #         shell=True,
    #         output="screen"
    #     )

    drone_patrol_controller = Node(
            package="followme_drone",
            executable="drone_patrol_controller",
            output="screen"
        )

    return LaunchDescription([
        declare_world_path,
        include_sjtu_drone,
        takeoff_cmd,
        drone_patrol_controller,
    ])