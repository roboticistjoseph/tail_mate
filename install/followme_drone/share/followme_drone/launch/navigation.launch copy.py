import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
# To launch another launch file
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = 'followme_drone'
    # Get the path to the config in package directory
    config_dir = os.path.join(get_package_share_directory(pkg_name),'config')
    
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
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_gazebo'),
                    'launch',
                    'turtlebot3_world.launch.py'
                ])
            ])
        ),

        IncludeLaunchDescription(
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
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            arguments=['-d', rviz_config_dir],
            output='screen'
        )
    ])