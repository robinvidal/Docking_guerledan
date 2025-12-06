from PyQt5.QtCore import QObject, pyqtSignal


class ROSSignals(QObject):
    """Qt signals for thread-safe ROS to Qt communication."""

    new_raw_frame = pyqtSignal(object)
    new_filtered_frame = pyqtSignal(object)
    new_borders = pyqtSignal(object)
    new_pose = pyqtSignal(object)
    new_state = pyqtSignal(object)
