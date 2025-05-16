from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    sender_node = Node(
        package='vehicle_sender',
        executable='vehicle_sender_node',
        name='vehicle_sender'
    )

    controller_node = Node(
        package='vehicle_controller',
        executable='vehicle_controller_node',
        name='vehicle_controller',
        parameters=[{
        'expected_speed': 12.0,
        'red_duration': 30.0,
        'yellow_duration': 4.0,
        'green_duration': 20.0,
        }]
    )

    delayed_controller = TimerAction(
        period=2.0,
        actions=[controller_node]
    )

    return LaunchDescription([
        sender_node,
        delayed_controller
    ])
