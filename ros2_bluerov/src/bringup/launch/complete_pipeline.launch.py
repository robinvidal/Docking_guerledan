"""
Launch complet: sonar + traitement + tracking + affichage
et pile PX4/mavros + control/ihm/tracking inky + joy.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Configs installés des packages
    sonar_node_config = os.path.join(
        get_package_share_directory('sonar'),
        'config',
        'sonar_node_params.yaml'
    )

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

    # ==== Sonar pipeline nodes ====
    sonar_node = Node(
        package='sonar',
        executable='sonar_node',
        name='sonar_node',
        parameters=[sonar_node_config],
        output='screen'
    )
    traitement = Node(
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

    sonar_viewer = Node(
        package='affichage',
        executable='sonar_viewer',
        name='sonar_viewer',
        output='screen'
    )

    # ==== MAVROS / PX4 (inky) ====
    launch_mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('mavros'), 'launch'),
                '/px4.launch',
            ]
        ),
        launch_arguments={
            'fcu_url': 'udp://192.168.2.1:14550@192.168.2.2:14550',
            'tgt_system': '1',
            'tgt_component': '1',
            'fcu_protocol': 'v2.0',
            'gcs_url': '',
            'namespace': 'inky/mavros',
            '/mavros/conn/timesync_rate': '0.0',
        }.items()
    )

    # Config inky (control/ihm/tracking) depuis bluerov_launch0
    inky_config = os.path.join(
        get_package_share_directory('bluerov_launch0'),
        'config',
        'parametre_inky.yaml'
    )

    node_control_inky = Node(
        package='bluerov_control0',
        namespace='inky',
        executable='control_bluerov',
        name='control_inky',
        output='screen',
        emulate_tty=True,
        parameters=[inky_config]
    )

    node_ihm_inky = Node(
        package='bluerov_ihm0',
        namespace='inky',
        executable='ihm',
        name='ihm_inky',
        output='screen',
        emulate_tty=True,
        parameters=[inky_config]
    )

    node_tracking_cam_inky = Node(
        package='bluerov_tracking0',
        namespace='inky',
        executable='tracking_node',
        name='affichage_cam',
        output='screen',
        emulate_tty=True,
        parameters=[inky_config]
    )

    node_joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    return LaunchDescription([
        # Sonar pipeline
        sonar_node,
        traitement,
        csrt_tracker,          # Nouveau système CSRT (sélection: Ctrl+Clic dans Sonar Viewer)
        sonar_viewer,

        # PX4 / MAVROS + control stack
        launch_mavros,
        node_control_inky,
        node_ihm_inky,
        node_tracking_cam_inky,
        node_joy,
    ])
