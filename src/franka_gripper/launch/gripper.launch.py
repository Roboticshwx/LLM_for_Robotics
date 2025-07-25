import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip_parameter_name = 'robot_ip'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    arm_parameter_name = 'arm_id'
    joint_names_parameter_name = 'joint_names'
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    arm_id = LaunchConfiguration(arm_parameter_name)
    joint_names = LaunchConfiguration(joint_names_parameter_name)

    gripper_config = os.path.join(
        get_package_share_directory('franka_gripper'), 'config', 'franka_gripper_node.yaml'
    )

    default_joint_name_postfix = '_finger_joint'
    arm_default_argument = [
        '[',
        arm_id,
        default_joint_name_postfix,
        '1',
        ',',
        arm_id,
        default_joint_name_postfix,
        '2',
        ']',
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                robot_ip_parameter_name, 
                default_value='192.168.8.2',
                description=('Hostname or IP address of the robot.'),
            ),
            DeclareLaunchArgument(
                use_fake_hardware_parameter_name,
                default_value='false',
                description=(
                    'Publish fake gripper joint states without connecting to a real gripper'
                ),
            ),
            DeclareLaunchArgument(
                arm_parameter_name,
                default_value='fr3',
                description=(
                    'Name of the arm in the URDF file. This is used to generate the joint '
                    'names of the gripper.'
                ),
            ),
            DeclareLaunchArgument(
                joint_names_parameter_name,
                default_value=arm_default_argument,
                description='Names of the gripper joints in the URDF',
            ),
            Node(
                package='franka_gripper',
                executable='franka_gripper_node',
                name=[arm_id, '_gripper'],
                parameters=[{'robot_ip': robot_ip, 'joint_names': joint_names}, gripper_config],
                condition=UnlessCondition(use_fake_hardware),
            ),
            Node(
                package='franka_gripper',
                executable='fake_gripper_state_publisher.py',
                name=[arm_id, '_gripper'],
                parameters=[{'robot_ip': robot_ip, 'joint_names': joint_names}, gripper_config],
                condition=IfCondition(use_fake_hardware),
            ),
        ]
    )