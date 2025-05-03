from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    pkg_path = get_package_share_directory('ggul_bot_v7_config')

    urdf_path = os.path.join(pkg_path, 'config', 'ggul_bot_v7.urdf')
    srdf_path = os.path.join(pkg_path, 'config', 'ggul_bot_v7.srdf')  # ✅ SRDF 추가
    kin_path = os.path.join(pkg_path, 'config', 'kinematics.yaml')

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    with open(srdf_path, 'r') as srdf_file:
        robot_description_semantic = srdf_file.read()

    with open(kin_path, 'r') as kin_file:
        robot_description_kinematics = yaml.safe_load(kin_file)

    return LaunchDescription([
        Node(
            package='ik_solver_cpp',
            executable='ik_subscriber_node',
            name='ik_subscriber_node',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'robot_description_semantic': robot_description_semantic},  # ✅ 반드시 필요
                {'robot_description_kinematics': robot_description_kinematics}
            ]
        )
    ])

