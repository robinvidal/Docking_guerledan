from PyQt5.QtCore import QObject, pyqtSignal


class ROSSignals(QObject):
    """Qt signals for thread-safe ROS to Qt communication."""

    new_raw_frame = pyqtSignal(object)
    new_polar_filtered_frame = pyqtSignal(object)
    new_cartesian_filtered_frame = pyqtSignal(object)
    new_borders = pyqtSignal(object)
    new_detected_lines = pyqtSignal(object)
    new_tracked_object = pyqtSignal(object)
    new_pose = pyqtSignal(object)
    new_state = pyqtSignal(object)
