# src/kaya_robot_control/setup.py

import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'kaya_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=[
        'setuptools',
        'rclpy',
        'dynamixel_sdk',
        'pygame',
    ],
    zip_safe=True,
    maintainer='tmkristi',
    maintainer_email='tmkristi@todo.todo',
    description='Robot control nodes for the KAYA platform',
    license='Apache-2.0',
    data_files=[
        # so ROS can discover your package
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        # install every Python file under launch/
        (f'share/{package_name}/launch', glob('launch/*.py')),
        # install the package manifest
        (f'share/{package_name}', ['package.xml']),
    ],
        entry_points={
        'console_scripts': [
            'set_motion       = kaya_robot_control.create_motion_controller:main',
            'path_follower    = kaya_robot_control.path_follower:main',
            'keyboard_teleop  = kaya_robot_control.teleop_qewasd:main',  # or teleop_wasd:main
        ],
    },

)
