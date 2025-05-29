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
    tb3_pkg = get_package_share_directory('turtlebot3_gazebo')
    my_pkg = get_package_share_directory('followme_drone')

    # Path declarations
    param_file = PathJoinSubstitution([
        FindPackageShare('followme_drone'),
        'nav2', 'tb3_nav2_params.yaml'
    ])
    
    map_file = PathJoinSubstitution([
        FindPackageShare('followme_drone'),
        'maps', 'tb3_map.yaml'
    ])
    
    rviz_config_dir = PathJoinSubstitution([
        FindPackageShare('followme_drone'),
        'rviz', 'navigation.rviz'
    ])
    
    turtlebot3_spawn_coordinates = (0.0,0.0)  # Default spawn coordinates for TurtleBot3

    # 1. Declare the launch argument for your custom world
    declare_world_path = DeclareLaunchArgument(
        'world_path',
        default_value=os.path.join(
            my_pkg,  # Replace with your package
            'worlds', 'laboratory.world'                # Replace with your world file
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
    
    tb3_bringup= IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_pkg, 'launch', 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose' :str(turtlebot3_spawn_coordinates[0]),
                'y_pose' :str(turtlebot3_spawn_coordinates[1]),
                # 'yaw_rot' :'1.57',
            }.items()

        )

    


    follow_tb3_vision= Node(
            package="followme_drone",
            executable="tb3_follower",
            output="screen"
        )
    
    tb3_navigate= Node(
            package="followme_drone",
            executable="tb3_nav_node",
            output="screen"
        )
    
    drive_tb3= Node(
            package="teleop_twist_keyboard",
            executable="teleop_twist_keyboard",
            output="screen",
            prefix="xterm -e"
        )

    # Timer action to delay robot spawning to avoid conflicts
    delayed_spawn = TimerAction(
        period= 15.0,  # 2 second delay between each spawn
        actions=[tb3_bringup]
    )
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_nav_rviz =  DeclareLaunchArgument('use_nav_rviz', default_value='true')


    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_pkg, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    nav2_bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                ])
            ]),
            launch_arguments={
                'map': map_file,
                'params_file': param_file
            }.items()
        )
    
    rviz_launch_cmd = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            arguments=['-d', rviz_config_dir],
            output='screen'
        )
    
    # Example: Delay Nav2 bringup until after TurtleBot3 is spawned
    delayed_nav2 = TimerAction(
        period=25.0,  # Wait 15 seconds after TB3 spawn
        actions=[nav2_bringup_cmd]
    )
    
    # Delay RViz2 until after Nav2 is up
    delayed_rviz = TimerAction(
        period=30.0,  # Wait 20 seconds after Nav2
        actions=[rviz_launch_cmd]
    )

    nodes_to_run = [
        declare_world_path,
        include_sjtu_drone,
        # drone_bringup,
        takeoff_cmd,
        follow_tb3_vision,
        delayed_spawn,
        robot_state_publisher_cmd,
        # tb3_bringup,
        drive_tb3,
        # nav2_bringup_cmd,
        # rviz_launch_cmd,
        delayed_nav2,
        delayed_rviz,
        # tb3_navigate,
    ]

    return LaunchDescription(nodes_to_run)