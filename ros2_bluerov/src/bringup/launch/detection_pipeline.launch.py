"""
Launch file pour pipeline partiel: sonar → traitement → tracking.
Test de la chaîne de détection.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sonar_config = os.path.join(
        get_package_share_directory('sonar'),
        'config',
        'sonar_params.yaml'
    )
    
    traitement_config = os.path.join(
        get_package_share_directory('traitement'),
        'config',
        'traitement_params.yaml'
    )
    
    tracking_config = os.path.join(
        get_package_share_directory('tracking'),
        'config',
        'tracking_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='sonar',
            executable='sonar_mock',
            name='sonar_mock',
            parameters=[sonar_config],
            output='screen'
        ),
        
        Node(
            package='traitement',
            executable='traitement_node',
            name='traitement_node',
            parameters=[traitement_config],
            output='screen'
        ),
        
        Node(
            package='tracking',
            executable='tracking_node',
            name='tracking_node',
            parameters=[tracking_config],
            output='screen'
        ),
    ])
