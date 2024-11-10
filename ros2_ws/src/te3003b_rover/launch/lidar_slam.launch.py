import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

	
    
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
	lidar = Node(
            package='te3003b_rover',  # Replace with the name of your package
            executable='lidar_publisher_slam',  # Name of the static transform broadcaster script
            name='lidar_publisher_slam',
            output='screen'
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
            remappings=[('/scan', '/scan')]
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
	
	
	l_d = LaunchDescription([rviz, lidar,cartographer_node, occupancy_grid_node])
	
	return l_d
