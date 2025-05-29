from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # === CONFIGURATION ===
    # Path to your custom world file
    world_file = '/home/roboticist/0_Projects/tail_mate/src/followme_drone/worlds/laboratory.world'

    # List of drone spawn positions (add as many as you want!)
    drone_positions = [
        {'name': 'drone_0', 'x': 0.0, 'y': 0.0, 'z': 0.1},
        {'name': 'drone_1', 'x': 2.0, 'y': 0.0, 'z': 0.1},
        {'name': 'drone_2', 'x': 0.0, 'y': 2.0, 'z': 0.1},
        # Add more drones here!
    ]

    # === LAUNCH GAZEBO ===
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # === SPAWN DRONES ===
    spawn_nodes = []
    for drone in drone_positions:
        spawn_nodes.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', drone['name'],
                    '-x', str(drone['x']),
                    '-y', str(drone['y']),
                    '-z', str(drone['z']),
                    '-file', '/home/roboticist/0_Projects/tail_mate/src/sjtu_drone/sjtu_drone_description/urdf/sjtu_drone.urdf.xacro',
                    '-robot_namespace', f"/{drone['name']}"
                ],
                output='screen'
            )
        )

    # === LAUNCH CONTROLLERS ===
    controller_nodes = []
    for drone in drone_positions:
        controller_nodes.append(
            Node(
                package='followme_drone',
                executable='drone_patrol_controller',
                namespace=drone['name'],
                output='screen',
                parameters=[
                    {'drone_namespace': f"/{drone['name']}"},
                    {'patrol_height': 1.0},  # Example parameter
                    # Add more patrol params as needed
                ]
            )
        )

    return LaunchDescription([gazebo] + spawn_nodes + controller_nodes)