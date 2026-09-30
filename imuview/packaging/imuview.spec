# PyInstaller spec. Build (from the project root, on the OS you want the program for):
#     uv run pyinstaller packaging/imuview.spec --noconfirm
# Result: dist/imuview.exe on Windows, dist/imuview on Linux (one file, no console window).
# A Windows .exe must be built ON Windows (PyInstaller does not cross-compile).
from pathlib import Path

root = Path(SPECPATH).parent

a = Analysis(
    [str(root / "packaging" / "imuview_gui.py")],
    pathex=[str(root / "src"), str(root / "vendor")],  # the package and vendor/mtdata2_decoder.py
    datas=[
        (str(root / "models"), "models"),  # .stl files and models.toml
        (str(root / "assets"), "assets"),  # fonts (Inter, JetBrains Mono, OFL) and Lucide icons
        (str(root / "config.example.toml"), "."),
    ],
    hiddenimports=["mtdata2_decoder", "pyqtgraph.opengl", "OpenGL.platform.win32",
                   "OpenGL.platform.glx", "OpenGL.platform.egl"],  # fmt: skip
    excludes=["rerun", "rerun_sdk", "tkinter", "matplotlib.tests"],
    noarchive=False,
)
pyz = PYZ(a.pure)
exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name="imuview",
    console=False,  # no terminal window; set True to see errors while testing
    upx=False,
)
