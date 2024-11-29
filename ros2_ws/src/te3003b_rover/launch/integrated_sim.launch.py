import os, time
from ament_index_python import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.events.process import ProcessStarted
from launch.actions import RegisterEventHandler
from launch.event_handlers.on_process_start import OnProcessStart

def generate_launch_description():

    control_param = 'path_planner'
    planner_map_size = 500
    
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
            	get_package_share_directory('te3003b_rover'),
                'config',
                'lidar_new.rviz')]
        )

    motors = Node(
            package='te3003b_rover',  # Replace with the name of your package
            executable='odometry_publisher',  # Name of the static transform broadcaster script
            name='odometry_publisher',
            output='screen'
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
    
    cartographer_node = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer',
            output='log',
            parameters=[{
                'use_sim_time': True
            }],
            arguments=[
                '-configuration_directory', os.path.join(get_package_share_directory('te3003b_rover'), 'config'),
                '-configuration_basename', 'cartographer.lua',  # Ensure this file exists in your config folder
                '--minloglevel', '2'
            ],
            remappings=[('/scan', '/scan')]
        )
    
    occupancy_grid_node = Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid',
            output='log',
            parameters=[{
                'use_sim_time': True,
                'resolution': 0.05
            }],
        )
    
    obj_selector = Node(
            package='te3003b_rover',  # Replace with the name of your package
            executable='objective_selector',  # Name of the static transform broadcaster script
            name='objective_selector',
            output='screen',
            parameters=[{
                'goals': [305,270,  333,230,  292,147],
                'max_distance': 500.0,
                'planner_map_size': planner_map_size
            }],
        )

    def bringup_cartographer(event: ProcessStarted, context: LaunchContext):
        time.sleep(5)
        return cartographer_node, occupancy_grid_node, obj_selector

    cart_handler = RegisterEventHandler(event_handler=OnProcessStart(target_action=lidar,
                                                                    on_start=bringup_cartographer))

    l_d = LaunchDescription([rviz, motors, lidar, cart_handler])

    return l_d