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
        default_value='/home/thomas/3A/Guerledobe/Docking_guerledan/ros2_bluerov/cage_complete2-20260119T133630Z-3-001/cage_complete2',
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
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path'), '--loop','-r', '0.5'],
        output='screen'
    )

    # ==== Pipeline nodes ====
    # Filtrage polaire
    traitement_polar = Node(
        package='traitement',
        executable='traitement_polar_node',
        name='traitement_polar_node',
        parameters=[traitement_config],
        output='screen'
    )

    # Filtrage cartésien
    traitement_cartesian = Node(
        package='traitement',
        executable='traitement_cartesian_node',
        name='traitement_cartesian_node',
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

    icp_tracker = Node(
        package='tracking',
        executable='icp_tracker_node',
        name='icp_tracker_node',
        parameters=[tracking_config],
        output='screen'
    )

    hough_lines = Node(
        package='tracking',
        executable='hough_lines_node',
        name='hough_lines_node',
        parameters=[tracking_config],
        output='screen'
    )

    csrt_tracker = Node(
        package='tracking',
        executable='csrt_tracker_node',
        name='csrt_tracker_node',
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
        traitement_polar,
        traitement_cartesian,
        hough_lines,
        csrt_tracker,
        #blob_tracker,
        #localisation,
        icp_tracker,
        sonar_viewer,
    ])
