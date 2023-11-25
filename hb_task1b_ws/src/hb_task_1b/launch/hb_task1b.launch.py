from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hb_task_1b',
            executable='controller.py'
        ),
        Node(
            package='hb_task_1b',
            executable='service_node.py'
        )
    ])