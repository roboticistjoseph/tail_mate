#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription , ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

def generate_launch_description():

    drone_pkg = get_package_share_directory('sjtu_drone_bringup')
    tb3_pkg = get_package_share_directory('turtlebot3_gazebo')
    my_pkg = get_package_share_directory('followme_drone')


    # 1. Declare the launch argument for your custom world
    declare_world_path = DeclareLaunchArgument(
        'world_path',
        default_value=os.path.join(
            my_pkg,  # Replace with your package
            'worlds', 'office_cpr.world'                # Replace with your world file
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

    drone_bringup= IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(drone_pkg, 'launch', 'sjtu_drone_bringup.launch.py')
            ),

        )

    tb3_bringup= IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_pkg, 'launch', 'spawn_turtlebot3.launch.py')
            ),

        )

    takeoff_cmd= ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '/drone/takeoff', 'std_msgs/msg/Empty', '{}','--once'],
            shell=True,
            output="screen"
        )


    follow_tb3_vision= Node(
            package="followme_drone",
            executable="followme_drone_node",
            output="screen"
        )
    drive_tb3= Node(
            package="teleop_twist_keyboard",
            executable="teleop_twist_keyboard",
            output="screen",
            prefix="xterm -e"
        )

    nodes_to_run = [
        declare_world_path,
        include_sjtu_drone,
        # drone_bringup,
        takeoff_cmd,
        follow_tb3_vision,
        tb3_bringup,
        drive_tb3


    ]

    return LaunchDescription(nodes_to_run)