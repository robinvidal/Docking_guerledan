"""
Launch file pour pipeline mock complet (développement sans matériel).
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Chemins des fichiers de config
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
    
    control_config = os.path.join(
        get_package_share_directory('control'),
        'config',
        'control_params.yaml'
    )
    
    mission_config = os.path.join(
        get_package_share_directory('mission'),
        'config',
        'mission_params.yaml'
    )
    
    return LaunchDescription([
        # Sonar mock
        Node(
            package='sonar',
            executable='sonar_mock',
            name='sonar_mock',
            parameters=[sonar_config],
            output='screen'
        ),
        
        # Traitement
        Node(
            package='traitement',
            executable='traitement_node',
            name='traitement_node',
            parameters=[traitement_config],
            output='screen'
        ),
        
        # Tracking
        Node(
            package='tracking',
            executable='tracking_node',
            name='tracking_node',
            parameters=[tracking_config],
            output='screen'
        ),
        
        # Localisation
        Node(
            package='localisation',
            executable='localisation_node',
            name='localisation_node',
            parameters=[localisation_config],
            output='screen'
        ),
        
        # Control
        Node(
            package='control',
            executable='control_node',
            name='control_node',
            parameters=[control_config],
            output='screen'
        ),
        
        # Mission
        Node(
            package='mission',
            executable='mission_node',
            name='mission_node',
            parameters=[mission_config],
            output='screen'
        ),
    ])
