"""
Launch pipeline avec rosbag de données sonar préenregistrées.

Usage:
  ros2 launch bringup rosbag_pipeline.launch.py bag_path:=/chemin/vers/rosbag_file.db3

Par défaut utilise: /home/maxxlef/Desktop/Docking_guerledan/ros2_bluerov/rosbag/cage_complete2
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Chemin vers le rosbag (relatif au workspace)
    # Depuis le package bringup installé, on remonte vers la racine du workspace
    bringup_share = get_package_share_directory('bringup')
    workspace_root = os.path.join(bringup_share, '..', '..', '..', '..')
    default_bag_path = os.path.abspath(os.path.join(workspace_root, 'rosbag', 'guerledan_02-02-sans-pvc'))
    
    # Argument pour le chemin du rosbag
    # ros2 launch bringup rosbag_pipeline.launch.py bag_path:=/chemin/vers/rosbag_file.db3
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=default_bag_path,
        description='Chemin vers le fichier rosbag (.db3 ou répertoire)'
    )

    # Configs des packages
    traitement_config = os.path.join(
        get_package_share_directory('traitement'),
        'config',
        'traitement_unified_params.yaml'
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

    # ==== Rosbag play ====
    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path'), '--loop', '--topics', '/docking/sonar/raw'],
        output='screen'
    )

    # ==== Pipeline nodes ====
    # Filtrage polaire
    traitement_unified_node = Node(
        package='traitement',
        executable='traitement_unified_node',
        name='traitement_unified_node',
        parameters=[traitement_config],
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
            'padding': 3.0,
            'filter_lr': 0.02,
            'psr_threshold': 0.035,
        }],
        output='screen'
    )

    hough_lines = Node(
        package='tracking',
        executable='hough_lines_node',  # Nom déclaré dans setup.py
        name='hough_lines_node',
        parameters=[tracking_config],
        output='screen'
    )

    localisation = Node(
        package='localisation',
        executable='localisation_node',
        name='localisation_node',
        parameters=[localisation_config],
        output='screen'
    )



    sonar_viewer = Node(
        package='affichage',
        executable='sonar_viewer',
        name='sonar_viewer',
        output='screen'
    )

    # NOTE: Sélection bbox via Ctrl+Clic dans Sonar Viewer
    return LaunchDescription([
        bag_path_arg,
        rosbag_play,
        traitement_unified_node,
        csrt_tracker,
        hough_lines,
        sonar_viewer,
    ])
