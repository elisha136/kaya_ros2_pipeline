import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'kaya_motion_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools','numpy','rclpy','matplotlib'],
    zip_safe=True,
    maintainer='tmkristi',
    maintainer_email='tmkristi@todo.todo',
    description='Motion Planning for KAYA',
    license='Apache-2.0',
    data_files=[
        # ament index
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        # install the launch scripts
        (f'share/{package_name}/launch', glob('launch/*.py')),
        # install package.xml
        (f'share/{package_name}', ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'input_simulator       = kaya_motion_planning.input_simulator:main',
            'motion_planner        = kaya_motion_planning.motion_planner:main',
           'trajectory_visualizer = kaya_motion_planning.trajectory_visualizer:main',
      ],
    },
)
