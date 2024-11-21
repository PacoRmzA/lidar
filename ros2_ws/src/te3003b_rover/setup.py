from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'te3003b_rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name), glob('launch/*launch.[pxy][yam]*')),
        (os.path.join('share',package_name), glob('launch/*.[pxy][yam]*')),
        (os.path.join('share',package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emitaker',
    maintainer_email='emitaker@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'static = te3003b_rover.static:main',
        'dynamic = te3003b_rover.dynamic:main',
        'lidar_publisher = te3003b_rover.lidar_publisher:main',
        'lidar_publisher_movement = te3003b_rover.lidar_publisher_movement:main',
        'lidar_publisher_slam = te3003b_rover.lidar_publisher_slam:main',
        'lidar_publisher_slam_map = te3003b_rover.lidar_publisher_slam_map:main',
        'keyboard_movement = te3003b_rover.keyboard_movement:main',
        'path_planner = te3003b_rover.path_planner:main',
        'objective_selector = te3003b_rover.objective_selector:main',
        'rpc_image_processor = te3003b_rover.rpc_image_processor:main'
        ],
    },
)
