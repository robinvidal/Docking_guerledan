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
    # Argument pour le chemin du rosbag
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/home/maxxlef/Desktop/Docking_guerledan/ros2_bluerov/rosbag/cage_complete2',
        description='Chemin vers le fichier rosbag (.db3 ou répertoire)'
    )

    # Configs des packages
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

    # ==== Rosbag play ====
    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path'), '--loop'],
        output='screen'
    )

    # ==== Pipeline nodes ====
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

    return LaunchDescription([
        bag_path_arg,
        rosbag_play,
        traitement,
        blob_tracker,
        localisation,
        sonar_viewer,
    ])
