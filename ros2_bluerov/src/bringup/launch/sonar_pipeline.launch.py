"""
Launch file pour démarrer automatiquement le pipeline avec le vrai sonar:
- sonar_node (driver Oculus M750d)
- sonar_viewer (visualisation)
- traitement_node (avec params)
- blob_tracker_node (avec params)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Chemins des fichiers de config installés
    sonar_node_config = os.path.join(
        get_package_share_directory('sonar'),
        'config',
        'sonar_node_params.yaml'
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

    # Nodes
    sonar_node = Node(
        package='sonar',
        executable='sonar_node',
        name='sonar_node',
        parameters=[sonar_node_config],
        output='screen'
    )

    sonar_viewer = Node(
        package='affichage',
        executable='sonar_viewer',
        name='sonar_viewer',
        output='screen'
    )

    traitement = Node(
        package='traitement',
        executable='traitement_node',
        name='traitement_node',
        parameters=[traitement_config],
        output='screen'
    )

    blob_tracker = Node(
        package='tracking',
        executable='blob_tracker_node',
        name='blob_tracker_node',
        parameters=[tracking_config],
        output='screen'
    )

    return LaunchDescription([
        sonar_node,
        traitement,
        blob_tracker,
        sonar_viewer,
    ])
