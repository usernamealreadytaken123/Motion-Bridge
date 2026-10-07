# Reproducible Windows one-file, windowed build. Run from the repository root.
from pathlib import Path
import os
import sys
import PySide6

project = Path(SPECPATH).parent
qt_directory = Path(PySide6.__file__).parent
# Do not resolve Windows/system DLL dependencies against unrelated tools on PATH
# (for example a Poppler ICU build is not interchangeable with Windows' ICU shim).
windows_directory = Path(os.environ.get('SystemRoot', 'C:/Windows'))
os.environ['PATH'] = os.pathsep.join(map(str, (
    Path(sys.executable).parent, qt_directory, windows_directory / 'System32', windows_directory,
)))
analysis = Analysis(
    [str(project / 'pc_app/desktop_entry.py')],
    pathex=[str(project)],
    binaries=[], datas=[], hiddenimports=[],
    hookspath=[], hooksconfig={}, runtime_hooks=[],
    excludes=['PyQt5', 'PyQt6', 'PySide2', 'matplotlib', 'scipy', 'IPython', 'jupyter'],
    noarchive=False,
)
# Python 3.12.3 ships VC runtime 14.38; this Qt build ships 14.44.
# Use the same redistributable set from the Qt wheel throughout the bundle,
# including its root (loaded before PySide6). Never change system/runtime installations.
runtime_names = ('VCRUNTIME140.dll', 'VCRUNTIME140_1.dll', 'MSVCP140.dll', 'MSVCP140_1.dll', 'MSVCP140_2.dll')
runtimes = {name.lower(): qt_directory / name for name in runtime_names}
for path in runtimes.values():
    if not path.is_file():
        raise RuntimeError(f'Missing bundled Qt runtime: {path}')
analysis.binaries = [
    (dest, str(runtimes[Path(dest).name.lower()]), kind)
    if Path(dest).name.lower() in runtimes else (dest, source, kind)
    for dest, source, kind in analysis.binaries
]
root_binaries = {dest.lower() for dest, _, _ in analysis.binaries}
analysis.binaries += [(name, str(qt_directory / name), 'BINARY')
                     for name in runtime_names if name.lower() not in root_binaries]
archive = PYZ(analysis.pure)
exe = EXE(
    archive, analysis.scripts, analysis.binaries, analysis.datas, [],
    name='MotionViewer', debug=False, bootloader_ignore_signals=False,
    strip=False, upx=False, console=False, disable_windowed_traceback=False,
)
