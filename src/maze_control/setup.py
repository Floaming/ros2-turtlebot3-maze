from setuptools import find_packages, setup

from glob import glob
import os

package_name = 'maze_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')
        ),
        (
            os.path.join('share', 'maze_control', 'rviz'),
            glob('rviz/*.rviz')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Floaming',
    maintainer_email='lcxznb@qq.com',
    description='ROS 2 TurtleBot3 maze control, sensing, and wall-following nodes.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "tb3_point2point = maze_control.tb3_point2point:main",
            "tb3_waypoints = maze_control.tb3_waypoints:main",
            "tb3_pure_pursuit = maze_control.tb3_pure_pursuit:main",
            "scan_region = maze_control.scan_region_node:main",
            "tb3_avoid = maze_control.tb3_avoid:main",
            "tb3_wall_follower = maze_control.tb3_wall_follower:main",
            "tb3_wall_follower_optimized = maze_control.tb3_wall_follower_optimized:main",
        ],
    },
)
