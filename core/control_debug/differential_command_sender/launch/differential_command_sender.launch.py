from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 声明launch参数
        DeclareLaunchArgument('work_mode', default_value='position_sine_velocity_fixed'),
        DeclareLaunchArgument('amplitude', default_value='3.1415926'),
        DeclareLaunchArgument('frequency', default_value='0.5'),
        DeclareLaunchArgument('offset', default_value='0.0'),
        DeclareLaunchArgument('phase', default_value='0.0'),
        DeclareLaunchArgument('fixed_velocity', default_value='1.0'),
        DeclareLaunchArgument('fixed_position', default_value='0.0'),
        DeclareLaunchArgument('publish_frequency', default_value='1000.0'),
        DeclareLaunchArgument('topic_name', default_value='end_differential_command'),
        
        Node(
            package='differential_command_sender',
            executable='differential_command_sender_node',
            name='differential_command_sender',
            output='screen',
            parameters=[{
                'work_mode': LaunchConfiguration('work_mode'),
                'amplitude': LaunchConfiguration('amplitude'),
                'frequency': LaunchConfiguration('frequency'),
                'offset': LaunchConfiguration('offset'),
                'phase': LaunchConfiguration('phase'),
                'fixed_velocity': LaunchConfiguration('fixed_velocity'),
                'fixed_position': LaunchConfiguration('fixed_position'),
                'publish_frequency': LaunchConfiguration('publish_frequency'),
                'topic_name': LaunchConfiguration('topic_name'),
            }]
        )
    ])