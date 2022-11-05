import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
      get_package_share_directory('maxon_epos_driver'),
      'config',
      'test_1motor.yaml'
      )

    return LaunchDescription([
        Node(
            package='maxon_epos_driver',
            executable='maxon_bringup', # Please DON'T add name to this node
            parameters=[config]
        )
    ])