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
            executable='lidar_publisher',  # Name of the static transform broadcaster script
            name='lidar_publisher',
            output='screen'
        )
	
	
	
	l_d = LaunchDescription([rviz, lidar])
	
	return l_d
