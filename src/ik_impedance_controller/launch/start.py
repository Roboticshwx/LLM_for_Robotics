from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    robot_ip_parameter_name = 'robot_ip'
    arm_id_parameter_name = 'arm_id'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    arm_id = LaunchConfiguration(arm_id_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(
        fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            robot_ip_parameter_name,
            default_value='192.168.8.2'
        ),
        DeclareLaunchArgument(
            arm_id_parameter_name,
            default_value='fr3'
        ),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='false'
        ),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false'
        ),
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='false'
        ),
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='true'
        ),
        # 包含外部的launch文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ik_impedance_controller'),
                    'launch',
                    'franka.launch.py'
                ])
            ]),
            launch_arguments={
                robot_ip_parameter_name: robot_ip,
                arm_id_parameter_name: arm_id,
                load_gripper_parameter_name: load_gripper,
                use_fake_hardware_parameter_name: use_fake_hardware,
                fake_sensor_commands_parameter_name: fake_sensor_commands,
                use_rviz_parameter_name: use_rviz,
        }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('franka_fr3_moveit_config'),
                    'launch',
                    'move_group.launch.py'
                ])
            ]),
            launch_arguments={
                robot_ip: robot_ip,
                load_gripper_parameter_name: load_gripper,
                use_fake_hardware_parameter_name: 'true',
                fake_sensor_commands_parameter_name: fake_sensor_commands,
                use_rviz_parameter_name: use_rviz,
            }.items(),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['ik_impedance_controller'], # yaml文件中的控制器标识名
            output='screen',
        ),
    ])