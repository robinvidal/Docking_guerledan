import os
import sys

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer
import rclpy

from .core.signals import ROSSignals
from .core.ros_node import SonarViewerNode
from .main_window import MainWindow


def apply_global_style(app):
    style_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'style.qss')
    if os.path.exists(style_path):
        with open(style_path, 'r', encoding='utf-8') as handle:
            app.setStyleSheet(handle.read())


def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    apply_global_style(app)

    signals = ROSSignals()
    ros_node = SonarViewerNode(signals)

    window = MainWindow(ros_node)
    window.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    timer.start(10)

    try:
        exit_code = app.exec_()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
