from PyQt5.QtCore import QObject, pyqtSignal


class ROSSignals(QObject):
    """Qt signals for thread-safe ROS to Qt communication."""

    new_raw_frame = pyqtSignal(object)
    new_cartesian_filtered_frame = pyqtSignal(object)
    new_detected_lines = pyqtSignal(object)
    new_cage_pose = pyqtSignal(object)
    new_tracked_object = pyqtSignal(object)
    
    # Auto-detect status (True = searching, False = not searching)
    auto_detect_status_changed = pyqtSignal(bool)
    
    # Sonar configuration response (success, message, range, gain, mode, ping_rate)
    sonar_config_response = pyqtSignal(bool, str, float, int, int, int)
