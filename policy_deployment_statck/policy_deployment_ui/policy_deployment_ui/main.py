from __future__ import annotations

import signal
import sys

from PyQt5.QtCore import QTimer
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
    signal_timer = QTimer()
    signal_timer.setInterval(250)
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start()

    def request_quit(_signum, _frame) -> None:
        app.quit()

    signal.signal(signal.SIGINT, request_quit)
    signal.signal(signal.SIGTERM, request_quit)

    view_model = UiViewModel(
        title=ros_client.config.title,
        recipe_id=ros_client.config.recipe_id,
        profile_id=ros_client.config.profile_id,
        prompt=ros_client.config.prompt,
        policy_profiles_config=ros_client.config.policy_profiles_config,
        processed_preview_topic_prefix=ros_client.config.processed_preview_topic_prefix,
        state=ros_client.state,
        profiles=ros_client.profiles,
    )
    window = MainWindow(view_model=view_model, ros_client=ros_client)
    app.aboutToQuit.connect(ros_client.shutdown)
    window.show()
    try:
        exit_code = app.exec() if hasattr(app, "exec") else app.exec_()
    finally:
        ros_client.shutdown()
    raise SystemExit(exit_code)
