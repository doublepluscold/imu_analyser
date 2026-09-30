"""Entry script for PyInstaller (the package itself has no __main__ for the GUI)."""

import sys

from imuview.gui.app import main

if __name__ == "__main__":
    sys.exit(main())
