"""Entry point of the packaged desktop program (PyInstaller): the configurator in its own window."""
import sys

from wbclient.desktop import main

if __name__ == "__main__":
    sys.exit(main())
