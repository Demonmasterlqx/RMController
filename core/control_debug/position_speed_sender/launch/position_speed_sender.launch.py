from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='position_speed_sender',
            executable='position_speed_sender_node',
            name='position_speed_sender',
            output='screen',
            parameters=[{
                'amplitude': 3.1415926,
                'frequency': 0.5,
                'offset': 3.1415926,
                'phase': 0.0,
                'fixed_speed': 1.0,        # 固定速度值
                'publish_frequency': 1000.0,
                'topic_name': 'position_speed_command'
            }]
        )
    ])