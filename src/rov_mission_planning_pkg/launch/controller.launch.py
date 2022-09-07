from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    ld = LaunchDescription()

    controller_node_params = os.path.join(
        get_package_share_directory('rov_mission_planning_pkg'),
        'config',
        'controller_node_params.yaml'
        )

    controller_node = Node(
        namespace='blueye',
        package='rov_mission_planning_pkg',
        executable='controller',
        name='controller_node',
        parameters=[controller_node_params]
    )

    ld.add_action(controller_node)

    return ld