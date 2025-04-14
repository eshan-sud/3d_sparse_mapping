from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='monocular_depth_estimator',
            executable='midas_depth_node.py',
            name='midas_depth_node',
            output='screen'
        )
    ])
