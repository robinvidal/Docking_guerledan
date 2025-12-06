import os
import yaml

def load_yaml_params(package_name: str, filename: str, logger=None):
    """Load a YAML params file from ROS share dir or local config fallback."""
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory(package_name)
        candidates.append(os.path.join(share_dir, 'config', filename))
    except Exception as exc:  # noqa: BLE001
        if logger:
            logger.debug(f"Pas de share pour {package_name}: {exc}")

    here = os.path.dirname(os.path.abspath(__file__))
    src_root = os.path.abspath(os.path.join(here, '..', '..'))
    candidates.append(os.path.join(src_root, package_name, 'config', filename))

    for path in candidates:
        if os.path.exists(path):
            try:
                with open(path, 'r', encoding='utf-8') as handle:
                    return yaml.safe_load(handle) or {}
            except Exception as exc:  # noqa: BLE001
                if logger:
                    logger.warn(f"Lecture YAML échouée {path}: {exc}")
                return {}
    if logger:
        logger.info(f"Fichier YAML introuvable parmi: {candidates}")
    return {}
