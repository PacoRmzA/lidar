import os, time
from ament_index_python import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.events.process import ProcessStarted
from launch.actions import RegisterEventHandler
from launch.event_handlers.on_process_start import OnProcessStart

def generate_launch_description():

    # one of 'keyboard', 'path_planner' or 'avoid_obstacles', defaults to 'keyboard'
    control_param = 'path_planner'
    planner_map_size = 300
    
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('te3003b_rover'),
                'config',
                'lidar.rviz')]
        )
    key_teleop = Node(
            package='key_teleop',
            executable='key_teleop',
            name='key_teleop',
            output='screen',
            prefix='xterm -e'
        )
    lidar = Node(
            package='te3003b_rover',  # Replace with the name of your package
            executable='lidar_publisher_slam_map',  # Name of the static transform broadcaster script
            name='lidar_publisher_slam_map',
            output='screen',
            parameters=[{
                'control': control_param,
                'planner_map_size': planner_map_size
            }],
        )
    obj_selector = Node(
            package='te3003b_rover',  # Replace with the name of your package
            executable='objective_selector',  # Name of the static transform broadcaster script
            name='objective_selector',
            output='screen',
            parameters=[{
                'goals': [188,127,  158,123,  172,146],
                'max_distance': 200.0,
                'planner_map_size': planner_map_size
            }],
        )
    cartographer_node = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }],
            arguments=[
                '-configuration_directory', os.path.join(get_package_share_directory('te3003b_rover'), 'config'),
                '-configuration_basename', 'cartographer.lua'  # Ensure this file exists in your config folder
            ],
            remappings=[('/scan', '/scan'),('/imu', '/imu'),('/odom', '/odom')]
        )
    occupancy_grid_node = Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'resolution': 0.05
            }],
        )
    
    def bringup_cartographer(event: ProcessStarted, context: LaunchContext):
        time.sleep(5)
        return cartographer_node, occupancy_grid_node, obj_selector

    cart_handler = RegisterEventHandler(event_handler=OnProcessStart(target_action=lidar,
                                                                     on_start=bringup_cartographer))

    l_d = LaunchDescription([rviz, key_teleop, lidar, cart_handler])

    return l_d
