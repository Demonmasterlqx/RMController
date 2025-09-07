import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    robot_description_pkg_dir = get_package_share_directory('speed_effor_controller')
    xacro_file = os.path.join(robot_description_pkg_dir, "test", 'config', "mcknum", 'test_robot.urdf.xacro')
    robot_description_content = Command(['xacro', ' ', xacro_file])

    controller_config_pkg_dir = get_package_share_directory('speed_effor_controller')
    controller_config_file = os.path.join(controller_config_pkg_dir, 'test', 'config', 'mcknum', 'ros2_controller.yaml')

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

    ld = LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
    ])

    # 得到 controller_manager/ros__parameters 下的所有 controller 名称
    import yaml
    with open(controller_config_file, 'r') as f:
        config = yaml.safe_load(f)
    controllers = [name for name in config.keys() if name != 'controller_manager' and name != "mcknum_controller"]
    for controller in controllers:
        spawner = Node(
            package='controller_manager',
            executable='spawner',
            name=controller,
            arguments=[
                controller,
                '--param-file',
                controller_config_file,
                '--activate-as-group',
                ],
        )
        ld.add_action(spawner)

    return ld

# /opt/ros/humble/lib/controller_manager/spawner mcknum_controller --param-file /home/pnx/code/RMController/install/speed_effor_controller/share/speed_effor_controller/test/config/mcknum/ros2_controller.yaml --activate-as-group --ros-args -r __node:=spawner