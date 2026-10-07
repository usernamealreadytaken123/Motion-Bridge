"""PyInstaller entry point; imports the same application as the development launcher."""

from pc_app.motion_viewer.desktop import main

if __name__ == "__main__":
    raise SystemExit(main())
