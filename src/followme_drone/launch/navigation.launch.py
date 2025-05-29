import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
# To launch another launch file
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Directories
    my_pkg = 'followme_drone'
    tb3_pkg = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    world_path = os.path.join(my_pkg,'worlds','laboratory.world')
    
    # Path declarations
    param_file = PathJoinSubstitution([
        FindPackageShare('followme_drone'),
        'nav2',
        'tb3_nav2_params.yaml'
    ])
    
    map_file = PathJoinSubstitution([
        FindPackageShare('followme_drone'),
        'maps',
        'tb3_map.yaml'
    ])
    
    rviz_config_dir = PathJoinSubstitution([
        FindPackageShare('followme_drone'),
        'rviz',
        'navigation.rviz'
    ])
    
    # Launch configurations
    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Initiating Gazebo Server and Client with specified world
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
        )
    )

    tb3_bringup= IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_pkg, 'launch', 'spawn_turtlebot3.launch.py')
            )
        )
    
    # robot_state_publisher_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_file_dir, 'launch', 'robot_state_publisher.launch.py')
    #     ),
    #     launch_arguments={'use_sim_time': use_sim_time}.items()
    # )
    
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
    
    # Launch description object
    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(tb3_bringup)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(rviz_launch_cmd)
    
    return ld