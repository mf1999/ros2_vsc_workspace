from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    ld = LaunchDescription()

    publisher_node_params = os.path.join(
        get_package_share_directory('testing_pkg'),
        'config',
        'pub_node_params.yaml'
        )

    publisher_node = Node(
        namespace='blueye',
        package='testing_pkg',
        executable='position_publisher',
        name='publisher_node',
        parameters=[publisher_node_params]
    )

    ld.add_action(publisher_node)

    return ld