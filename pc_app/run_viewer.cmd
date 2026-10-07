@echo off
setlocal
cd /d "%~dp0.."
if not exist "%~dp0.venv\Scripts\python.exe" (
    echo GUI environment is missing. Follow pc_app\README.md to install it.
    pause
    exit /b 1
)
"%~dp0.venv\Scripts\python.exe" -m pc_app.motion_viewer gui
if errorlevel 1 pause
