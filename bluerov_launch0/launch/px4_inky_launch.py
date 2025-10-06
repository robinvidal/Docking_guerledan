import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace

from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import AnyLaunchDescriptionSource


# Fonction appele par ros2 launch pour avoir la liste des nodes à lancer

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('bluerov_launch0'),
        'config', # repertoire (dans le dossier "share/(...)" qui a été generé et non dans le "src")
        'parametre_inky.yaml' # nom du fichier .yaml
    )

    #"""

    
    launch_mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("mavros"), "launch"),
                "/px4.launch",
            ]
        ),
        launch_arguments={
            'fcu_url': 'udp://192.168.2.1:14550@192.168.2.2:14550',
            'tgt_system':'1',
            'tgt_component':'1',
            "fcu_protocol":"v2.0",
            "gcs_url":"", 
            "namespace": "inky/mavros",
            "/mavros/conn/timesync_rate":"0.0",

            
        }.items()

    )


    node_control_inky = Node(
        package="bluerov_control0", 
        namespace='inky',
        executable="control_bluerov", 
        name="control_inky", 
        output="screen",
        emulate_tty= True,
        parameters=[config]

    )

    node_ihm_inky = Node(
        package="bluerov_ihm0", 
        namespace='inky',
        executable="ihm", 
        name="ihm_inky", 
        output="screen",
        emulate_tty= True,
        parameters=[config]
    )


    node_affichage_inky = Node(
        package="bluerov_tracking0", 
        namespace='inky',
        executable="tracking_node", 
        name="affichage_cam", 
        output="screen",
        emulate_tty= True,
        parameters=[config]
    )

    ######## Nodes commun #######


    node_joy = Node(
        package="joy", 
        executable="joy_node", 
        name="joy_node", 
        output="screen"
    )



    # retour de la fonction avec la liste des nodes à lancer
    
    return LaunchDescription([
    
    	launch_mavros,

        node_control_inky,
        node_ihm_inky,
        node_affichage_inky ,

    	node_joy


    ])
    
    
