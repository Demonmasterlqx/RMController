import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    robot_description_pkg_dir = get_package_share_directory('position_speed_controller')
    xacro_file = os.path.join(robot_description_pkg_dir, "test", 'config', 'robot.urdf.xacro')
    robot_description_content = Command(['xacro', ' ', xacro_file])

    controller_config_pkg_dir = get_package_share_directory('position_speed_controller')
    controller_config_file = os.path.join(controller_config_pkg_dir, 'test', 'config', 'ros2_controller.yaml')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='screen'
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config_file],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output='screen'
    )

    spawner = Node(
        package='controller_manager',
        executable='spawner',
        name="test_position_speed_controller",
        arguments=[
            "test_position_speed_controller",
            '--param-file',
            controller_config_file
            ],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        spawner
    ])