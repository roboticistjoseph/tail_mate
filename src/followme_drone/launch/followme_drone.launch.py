"""
Author: Joseph Katakam

Launch File: followme_drone.launch.py
Description:
    This launch file is designed to set up a simulation environment for a TurtleBot3 robot and SJTU drone in Gazebo,
    including spawning the robot, configuring navigation parameters, and launching RViz for visualization.
    It integrates with the `sjtu_drone_bringup` package to manage drone operations and the `turtlebot3_gazebo`
    package to handle the TurtleBot3 simulation.

Packages involved:
    gazebo_ros
    turtlebot3_gazebo
    sjtu_drone_bringup
    followme_drone
    nav2_bringup

Arguments:
- `world_path`: Path to the custom world file for the simulation.
- `use_sim_time`: Whether to use simulation time (default: true).
"""

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

def generate_launch_description():
    # Get package directories
    drone_pkg = get_package_share_directory('sjtu_drone_bringup')
    tb3_pkg = get_package_share_directory('turtlebot3_gazebo')
    my_pkg = get_package_share_directory('followme_drone')

    # Default spawn coordinates for TurtleBot3
    turtlebot3_spawn_coordinates = (0.0,0.0)  # Default spawn coordinates for TurtleBot3
    
    # Path declarations
    # Navigation parameters file
    param_file = PathJoinSubstitution([
        FindPackageShare('followme_drone'),
        'nav2', 'tb3_nav2_params.yaml'
    ])
    # Map file for the TurtleBot3 simulation
    map_file = PathJoinSubstitution([
        FindPackageShare('followme_drone'),
        'maps', 'tb3_map.yaml'
    ])
    # RViz configuration file
    rviz_config_dir = PathJoinSubstitution([
        FindPackageShare('followme_drone'),
        'rviz', 'navigation.rviz'
    ])

    # Declare the launch argument for your custom world
    declare_world_path = DeclareLaunchArgument(
        'world_path',
        default_value=os.path.join(
            my_pkg,
            'worlds', 'laboratory.world'
        ),
        description='Path to the custom world file'
    )
    
    # Declare the launch argument for simulation time
    # This is used to synchronize the simulation with real time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Include the submodule's launch file, passing your world as an argument
    include_sjtu_drone = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                drone_pkg,
                'launch',
                'sjtu_drone_bringup.launch.py'
            )
        ]),
        launch_arguments={'world': LaunchConfiguration('world_path')}.items()
    )

    # Command to take off the drone
    # This command publishes an empty message to the takeoff topic to initiate the drone's takeoff
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
    
    # Command to spawn the TurtleBot3 in the Gazebo simulation
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

    # Robot State Publisher command to publish the robot's state
    # This is necessary for the robot's model to be correctly visualized in RViz
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_pkg, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Timer action to delay robot spawning & avoid conflicts
    delayed_tb3_spawn = TimerAction(
        period= 10.0,
        actions=[tb3_bringup]
    )
    
    # Node to follow the TurtleBot3 using vision
    drone_detection = Node(
            package="followme_drone",
            executable="drone_detection_node",
            output="screen"
        )
    
    # Node to navigate the TurtleBot3 using navigation stack
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
    
    # RViz visualization of TurtleBot3's navigation stack
    rviz_launch_cmd = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            arguments=['-d', rviz_config_dir],
            output='screen'
        )
    
    # Delay Nav2 bringup until after TurtleBot3 is spawned
    delayed_tb3_nav2 = TimerAction(
        period=20.0,  # Wait 10 seconds after TB3 spawn
        actions=[nav2_bringup_cmd]
    )
    
    # Delay RViz2 until after Nav2 is up
    delayed_tb3_rviz = TimerAction(
        period=25.0,  # Wait 5 seconds after Nav2
        actions=[rviz_launch_cmd]
    )

    # Node to waypoint navigate the TurtleBot3 using Nav2 stack
    tb3_waypoint_navigate= Node(
            package="followme_drone",
            executable="tb3_nav_node",
            output="screen"
        )
    
    # Delay navigation
    delayed_b3_waypoint_navigate = TimerAction(
        period=40.0,  # Wait 5 seconds after Nav2
        actions=[tb3_waypoint_navigate]
    )
    
    nodes_to_run = [
        declare_world_path,
        include_sjtu_drone,
        takeoff_cmd,
        delayed_tb3_spawn,
        robot_state_publisher_cmd,
        drone_detection,
        delayed_tb3_nav2,
        delayed_tb3_rviz,
        # delayed_b3_waypoint_navigate,
    ]

    return LaunchDescription(nodes_to_run)