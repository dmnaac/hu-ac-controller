import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('uwb_wma_filter')
    params_path = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='uwb_wma_filter',
            executable='uwb_wma_filter_node',
            name='uwb_wma_filter',
            output='screen',
            parameters=[params_path],
            emulate_tty=True,
        ),
    ])
