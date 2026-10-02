"""Entry point: `imuview-gui` or `imu gui`. Imports Qt."""

import argparse
import sys
from pathlib import Path

from .gl_common import configure_platform

configure_platform()  # must run before pyqtgraph.opengl is imported

from PySide6.QtCore import QCoreApplication, QStandardPaths, Qt  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from ..pipeline import load_config  # noqa: E402
from . import theme  # noqa: E402
from .controller import AppController  # noqa: E402
from .library import ModelLibrary  # noqa: E402
from .main_window import MainWindow  # noqa: E402


def resource_path(rel: str) -> Path:
    """A file shipped with the program: next to the sources, or inside the PyInstaller bundle."""
    base = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parents[3]))
    return base / rel


def main(argv=None):
    ap = argparse.ArgumentParser(prog="imuview-gui", description="IMU demo GUI")
    ap.add_argument("--config", help="TOML config (default: config.toml if it exists)")
    ap.add_argument("--calib", help="host calibration JSON (default: calib/imu_calib.json)")
    ap.add_argument("--models", help="folder with .stl files and models.toml (default: models/)")
    ap.add_argument("--source", choices=["master", "udp", "serial", "sim"], help="connect at start")
    ap.add_argument("--session", help="open this recorded session in analysis mode")
    args = ap.parse_args(argv)

    config_path = args.config or ("config.toml" if Path("config.toml").exists() else None)
    config = load_config(config_path)
    models = Path(args.models) if args.models else resource_path("models")
    calib = args.calib or (
        "calib/imu_calib.json" if Path("calib/imu_calib.json").exists() else None
    )
    controller = AppController(config, models_dir=models, calib_path=calib)

    # The model dialog is a second window with its own 3D view. Without shared contexts the
    # shader programs compiled for the main window are invalid there (GLError, black view).
    QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    app = QApplication(sys.argv[:1])
    app.setOrganizationName("imuview")
    app.setApplicationName("imuview")
    data_dir = Path(QStandardPaths.writableLocation(QStandardPaths.AppDataLocation)) / "models"
    controller.library = ModelLibrary(data_dir, seed_dir=models)
    for problem in controller.library.problems:
        print(f"[warn] model library: {problem}", flush=True)
    controller.set_registry(controller.library.registry())
    theme.apply_theme(app)
    win = MainWindow(controller)
    win.show()
    if args.session:
        controller.open_session(args.session)
        win.sub.session_label.setText(Path(args.session).name)
        win.set_tab("analysis")
    elif args.source:
        win.top.set_transport(args.source, win.top.port_text())
        win.connect_now()
    win.sync_controls()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
