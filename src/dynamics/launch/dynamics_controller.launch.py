import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    dynamics_config = os.path.join(
        get_package_share_directory('dynamics'),
        'config',
        'params.yaml'
        )
    controller_config = os.path.join(
        get_package_share_directory('controller'),
        'config',
        'params.yaml'
        )
    dynamics=Node(
        package = 'dynamics',
        name = 'dynamics',
        executable = 'dynamics',
        parameters = [dynamics_config]
    )
    controller=Node(
        package = 'controller',
        name = 'pid',
        executable = 'pid',
        parameters = [controller_config]
    )

    ld.add_action(dynamics)
    ld.add_action(controller)

    return ld