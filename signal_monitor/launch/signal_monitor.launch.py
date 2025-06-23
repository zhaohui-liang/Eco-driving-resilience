from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='signal_monitor',
            executable='signal_monitor_node',
            name='signal_monitor_node',
            output='screen',
            parameters=[{
            'red_duration': 34.8,
            'yellow_duration': 3.2,
            'green_duration': 55.1,
            'signal_offset': 0.0,  # shift phase forward by 20 seconds
            'distrubance': 0.0
            }]
        )
    ])
