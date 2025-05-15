from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kaya_robot_control',
            executable='path_follower',
            name='path_follower',
            output='screen',
        ),
        Node(
            package='kaya_robot_control',
            executable='set_motion',
            name='set_motion',
            output='screen',
        ),
        Node(
            package='kaya_robot_control',
            executable='keyboard_teleop',
            name='keyboard_teleop',
            output='screen',
        ),
    ])
