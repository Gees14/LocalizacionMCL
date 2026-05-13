from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'puzzlebot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'puzzlebot_control'),
            glob('puzzlebot_control/maze_map.png') +
            glob('puzzlebot_control/maze_map.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jorge Reyes',
    maintainer_email='rpz.dar14@gmail.com',
    description='Localization and odometry nodes for the Puzzlebot (MCL)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'wheel_odometry    = puzzlebot_control.wheel_odometry:main',
            'localization_node = puzzlebot_control.localization_node:main',
            'maze_builder      = puzzlebot_control.maze_builder:main',
        ],
    },
)
