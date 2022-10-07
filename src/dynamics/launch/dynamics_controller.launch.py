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

    # free_dynamics=Node(
    #     package = 'dynamics',
    #     name = 'free_dynamics',
    #     executable = 'dynamics',
    #     parameters = [{'m': 1.0, 'k': 2.5, 'c': 0.3, 'x': -1.0, 'v': 0.0, 'a': 0.0}],
    #     remappings=[
    #         ('dynamics/position', '/free_dynamics/position'),
    #         ('dynamics/velocity', '/free_dynamics/velocity'),
    #         ('controller/command', '/free_controller/command'),
    #     ]
    # )
    # free_controller=Node(
    #     package = 'controller',
    #     name = 'free_pid',
    #     executable = 'pid',
    #     parameters = [{'kp': 0.0, 'ki': 0.0, 'kd': 0.0}],
    #     remappings=[
    #         ('dynamics/position', '/free_dynamics/position'),
    #         ('dynamics/velocity', '/free_dynamics/velocity'),
    #         ('controller/command', '/free_controller/command'),
    #     ]
        
    # )
    ld.add_action(dynamics)
    ld.add_action(controller)
    # ld.add_action(free_dynamics)
    # ld.add_action(free_controller)
    return ld