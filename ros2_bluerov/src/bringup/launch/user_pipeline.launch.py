"""
Launch file pour démarrer automatiquement le pipeline utilisateur:
- sonar_mock
- sonar_viewer
- traitement_node (avec params)
- blob_tracker_node ou csrt_tracker_node (avec params)
- bbox_selector_node (pour sélection manuelle CSRT)
- localisation
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

    localisation_config = os.path.join(
        get_package_share_directory('localisation'),
        'config',
        'localisation_params.yaml'
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

    # Option 1: Blob tracker (ancien système)
    blob_tracker = Node(
        package='tracking',
        executable='blob_tracker_node',
        name='blob_tracker_node',
        parameters=[tracking_config],
        output='screen'
    )

    # Option 2: CSRT tracker avec sélection manuelle
    csrt_tracker = Node(
        package='tracking',
        executable='csrt_tracker_node',
        name='csrt_tracker_node',
        parameters=[{
            'enable_tracking': True,
            'selection_mode': 'manual',
            'use_hog': True,
            'use_gray': True,
            'use_color_names': False,
            'padding': 3.0,
            'filter_lr': 0.02,
            'psr_threshold': 0.035,
        }],
        output='screen'
    )

    sonar_viewer = Node(
        package='affichage',
        executable='sonar_viewer',
        name='sonar_viewer',
        output='screen'
    )

    localisation = Node(
        package='localisation',
        executable='localisation_node',
        name='localisation_node',
        parameters=[localisation_config],
        output='screen'
    )

    # NOTE: Décommenter blob_tracker OU csrt_tracker selon besoin
    # Sélection bbox: Ctrl+Clic dans Sonar Viewer (image cartésienne)
    return LaunchDescription([
        sonar_mock,
        traitement,
        # blob_tracker,        # Ancien système (décommenter si besoin)
        csrt_tracker,          # Nouveau système CSRT
        sonar_viewer,
        localisation,
    ])
