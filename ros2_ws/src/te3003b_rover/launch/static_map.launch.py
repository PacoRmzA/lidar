import os, time
from ament_index_python import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.events.process import ProcessStarted
from launch.actions import RegisterEventHandler
from launch.event_handlers.on_process_start import OnProcessStart

def generate_launch_description():

    use_keyboard = False

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
    nav_key = Node(
            package='te3003b_rover',  # Replace with the name of your package
            executable='keyboard_movement',  # Name of the static transform broadcaster script
            name='keyboard_movement',
            output='screen'
        )
    nav = Node(
            package='te3003b_rover',  # Replace with the name of your package
            executable='path_planner',  # Name of the static transform broadcaster script
            name='path_planner',
            output='screen'
        )
    img_map = Node(
            package='nav2_map_server',  # Replace with the name of your package
            executable='map_server',  # Name of the static transform broadcaster script
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': '/home/thecubicjedi/lidar/map_converted.yaml'}]
        )
    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}])
    
    def bringup_map(event: ProcessStarted, context: LaunchContext):
        time.sleep(5)
        return start_lifecycle_manager_cmd
    
    def bringup_nav(event: ProcessStarted, context: LaunchContext):
        time.sleep(5)
        return nav

    if use_keyboard:
        l_d = LaunchDescription([
            RegisterEventHandler(event_handler=OnProcessStart(target_action=nav_key,
                                                            on_start=bringup_map)),
            nav_key, key_teleop, img_map, rviz
        ])
    else:
        l_d = LaunchDescription([
            RegisterEventHandler(event_handler=OnProcessStart(target_action=nav,
                                                            on_start=bringup_map)),
            RegisterEventHandler(event_handler=OnProcessStart(target_action=rviz,
                                                            on_start=bringup_nav)),
            img_map, rviz
        ])

    return l_d
