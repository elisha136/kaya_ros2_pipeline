from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # 1) Start the simulator (robot_position & goal_position)
        Node(
            package='kaya_motion_planning',
            executable='input_simulator',
            name='input_simulator',
            output='screen',
        ),

        # 2) Run the planner (listens for goal_position)
        Node(
            package='kaya_motion_planning',
            executable='motion_planner',
            name='motion_planner',
            output='screen',
        ),

        # 3) Visualize the Path
        Node(
            package='kaya_motion_planning',
            executable='trajectory_visualizer',
            name='trajectory_visualizer',
            output='screen',
        ),

    ])
