"""
Lance le replay d'une mission avec rosbag et affichage.

Usage:
  ros2 launch bringup replay_mission.launch.py bag_path:=/chemin/vers/rosbag_file.db3

Par défaut utilise: /home/maxxlef/Desktop/Docking_guerledan/ros2_bluerov/rosbag/guerledan_04-02-mission4
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
    default_bag_path = os.path.abspath(os.path.join(workspace_root, 'rosbag', 'guerledan_04-02-15mission4'))
    
    # Argument pour le chemin du rosbag
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=default_bag_path,
        description='Chemin vers le fichier rosbag (.db3 ou répertoire)'
    )

    # ==== Rosbag play ====
    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path'), '--loop'],
        output='screen'
    )

    # ==== Affichage ====
    sonar_viewer = Node(
        package='affichage',
        executable='sonar_viewer',
        name='sonar_viewer',
        output='screen'
    )

    return LaunchDescription([
        bag_path_arg,
        rosbag_play,
        sonar_viewer,
    ])
