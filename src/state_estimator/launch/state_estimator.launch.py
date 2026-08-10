import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('state_estimator')
    params_path = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='state_estimator',
            executable='state_estimator_node',
            name='state_estimator',
            output='screen',
            parameters=[params_path],
            emulate_tty=True,
        ),
    ])
