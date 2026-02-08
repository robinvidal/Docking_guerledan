"""
Launch file pour démarrer le pipeline de test avec le mock sonar:
- sonar_mock
- sonar_viewer
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

    # Nodes
    sonar_mock = Node(
        package='sonar',
        executable='sonar_mock',
        name='sonar_mock',
        parameters=[sonar_config],
        output='screen'
    )

    sonar_viewer = Node(
        package='affichage',
        executable='sonar_viewer',
        name='sonar_viewer',
        output='screen'
    )

    # NOTE: Décommenter blob_tracker OU csrt_tracker selon besoin
    # Sélection bbox: Ctrl+Clic dans Sonar Viewer (image cartésienne)
    return LaunchDescription([
        sonar_mock,
        sonar_viewer
    ])
