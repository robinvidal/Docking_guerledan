"""
Launch file pour démarrer automatiquement les 5 terminaux demandés:
- sonar_mock
- sonar_viewer
- teleop_twist_keyboard (remappé vers /bluerov/cmd_vel)
- traitement_node (avec params)
- blob_tracker_node (avec params)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os


def generate_launch_description():
    # Chemins des fichiers de config installés
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

    # Nodes
    sonar_mock = Node(
        package='sonar',
        executable='sonar_mock',
        name='sonar_mock',
        parameters=[sonar_config],
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

    sonar_viewer = Node(
        package='affichage',
        executable='sonar_viewer',
        name='sonar_viewer',
        output='screen'
    )

    return LaunchDescription([
        sonar_mock,
        traitement,
        blob_tracker,
        sonar_viewer,
    ])
