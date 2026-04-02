from __future__ import annotations

import sys

from PyQt5.QtWidgets import QApplication
from rclpy.utilities import remove_ros_args

from .ros_client import RosClient
from .view_model import UiViewModel
from .widgets.main_window import MainWindow


def main(args: list[str] | None = None) -> None:
    raw_args = list(sys.argv if args is None else args)
    qt_args = remove_ros_args(args=raw_args)

    ros_client = RosClient(args=raw_args)
    app = QApplication(qt_args if qt_args else [raw_args[0]])
    view_model = UiViewModel(config=ros_client.config)
    window = MainWindow(view_model=view_model, ros_client=ros_client)
    app.aboutToQuit.connect(ros_client.shutdown)
    window.show()
    try:
        exit_code = app.exec() if hasattr(app, "exec") else app.exec_()
    finally:
        ros_client.shutdown()
    raise SystemExit(exit_code)
