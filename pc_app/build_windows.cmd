@echo off
setlocal
cd /d "%~dp0.."
set "PYINSTALLER_CONFIG_DIR=%~dp0build\cache"
if not exist "%~dp0.venv\Scripts\python.exe" (
    echo Create pc_app\.venv and install pc_app\requirements-build.txt first.
    exit /b 1
)
"%~dp0.venv\Scripts\python.exe" -m unittest discover -s pc_app/tests
if errorlevel 1 exit /b 1
"%~dp0.venv\Scripts\python.exe" -m PyInstaller --noconfirm --distpath pc_app/dist --workpath pc_app/build pc_app/MotionViewer.spec
if errorlevel 1 exit /b 1
"%~dp0.venv\Scripts\python.exe" "%~dp0verify_windows_build.py"
if errorlevel 1 exit /b 1
echo Ready: pc_app\dist\MotionViewer.exe
