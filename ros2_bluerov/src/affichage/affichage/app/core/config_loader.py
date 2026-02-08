"""
Chargeur de configuration depuis les fichiers YAML ROS2.
Permet de partager les paramètres entre les nœuds ROS2 et l'interface graphique.
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory


def load_tracking_params():
    """
    Charge les paramètres de tracking depuis le fichier YAML.
    
    Returns:
        dict: Dictionnaire contenant les paramètres de tracking.
    """
    try:
        config_path = os.path.join(
            get_package_share_directory('tracking'),
            'config',
            'tracking_params.yaml'
        )
        
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        return config
    except Exception as e:
        print(f"Erreur lors du chargement de tracking_params.yaml: {e}")
        return {}


def get_cage_dimensions():
    """
    Récupère les dimensions de la cage depuis la configuration.
    
    Returns:
        tuple: (cage_width, cage_height) en mètres. Valeurs par défaut si erreur.
    """
    config = load_tracking_params()
    
    try:
        csrt_params = config.get('csrt_tracker_node', {}).get('ros__parameters', {})
        cage_width = float(csrt_params.get('cage_width', 0.9))
        cage_height = float(csrt_params.get('cage_height', 0.5))
        return cage_width, cage_height
    except Exception as e:
        print(f"Erreur lors de la lecture des dimensions de la cage: {e}")
        return 0.9, 0.5  # Valeurs par défaut
