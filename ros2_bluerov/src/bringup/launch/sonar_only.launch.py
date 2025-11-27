"""
Launch file pour sonar uniquement (test acquisition).
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Argument pour choisir mock ou hardware
    use_mock_arg = DeclareLaunchArgument(
        'use_mock',
        default_value='true',
        description='Utiliser sonar_mock (true) ou sonar_node (false)'
    )
    
    use_mock = LaunchConfiguration('use_mock')
    
    sonar_config = os.path.join(
        get_package_share_directory('sonar'),
        'config',
        'sonar_params.yaml'
    )
    
    return LaunchDescription([
        use_mock_arg,
        
        # Sonar (mock ou hardware selon argument)
        Node(
            package='sonar',
            executable='sonar_mock',  # TODO: condition sur use_mock
            name='sonar',
            parameters=[sonar_config],
            output='screen'
        ),
    ])
